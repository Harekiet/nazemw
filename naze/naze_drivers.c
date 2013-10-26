#include "naze.h"

#include "../drivers/drivers.h"

static sensor_t acc;                       // acc access functions
static sensor_t gyro;                      // gyro access functions
static baro_t baro;               			// barometer access functions
static uint8_t sensorMask = 0;		//What sensors do we have

// Autodetect: turn off BMP085 while initializing ms5611 and check PROM crc to confirm device
#define BMP085_OFF                  digitalLo(BARO_GPIO, BARO_PIN);
#define BMP085_ON                   digitalHi(BARO_GPIO, BARO_PIN);


// AfroFlight32 i2c sensors
void nazeDriversInit( uint8_t accHardware ) {
//    int16_t deg, min;
    drv_adxl345_config_t acc_params;
    bool haveMpu6k = false;

	uint16_t gyro_lpf = 28;
	uint8_t gyro_scale = 1;

    //Assume we always have a gyro
    sensorMask |= BOARD_SENSOR_GYRO;
    // Autodetect gyro hardware. We have MPU3050 or MPU6050.
    if (mpu6050Detect(&acc, &gyro, gyro_lpf, &gyro_scale)) {
        // this filled up  acc.* struct with init values
        haveMpu6k = true;
    } else if (l3g4200dDetect(&gyro, gyro_lpf)) {
        // well, we found our gyro
        ;
    } else if (!mpu3050Detect(&gyro, gyro_lpf)) {
        // if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
    	boardFault( BOARD_FAULT_FATAL );
    }

    // Accelerometer. Fuck it. Let user break shit.
retryAcc:
    switch (accHardware) {
        case 0: // autodetect
        case 1: // MPU6050
            if (haveMpu6k) {
            	//acc struct already filled from previous gyro detection

                // PB13 - MPU_INT output on rev4 hardware
                GPIO_InitTypeDef GPIO_InitStructure;
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
                GPIO_Init(GPIOB, &GPIO_InitStructure);

                goto validAcc;
            }
        case 2: // ADXL345
        	acc_params.useFifo = false;
            acc_params.dataRate = 800; // unused currently
            if (adxl345Detect(&acc_params, &acc) )
                goto validAcc;
            //Fallthrough to the next one
        case 3: // MMA8452
            if (mma8452Detect(&acc)) {
            	//Some gpio magic to trigger an init

                GPIO_InitTypeDef GPIO_InitStructure;

                // PA5 - ACC_INT2 output on rev3/4 hardware
                // OLIMEXINO - The PA5 pin is wired up to LED1, if you need to use an mma8452 on an Olimexino use a different pin and provide support in code.
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
                GPIO_Init(GPIOA, &GPIO_InitStructure);

                goto validAcc;
            }
        default:
        	//nothing found, seems there's no ACC
        	goto skipAcc;
    }
    accHardware++;
    goto retryAcc;
validAcc:
	sensorMask |= BOARD_SENSOR_ACC;
	//Found a valid acc, init it
	acc.init();
skipAcc:

#ifdef BARO

	 GPIO_InitTypeDef GPIO_InitStructure;

	 // PC13 (BMP085's XCLR reset input, which we use to disable it)
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_Init(GPIOC, &GPIO_InitStructure);
	 BMP085_OFF;

    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function
	// ms5611 disables BMP085, and tries to initialize + check PROM crc. if this works, we have a baro
	if ( ms5611Detect(&baro) || bmp085Detect(&baro) ) {


		sensorMask |= BOARD_SENSOR_BARO;
    }
#endif
    // this is safe because either mpu6050 or mpu3050 or lg3d20 sets it, and in case of fail, we never get here.
    gyro.init();

    if ( hmc5883lDetect() ) {
    	sensorMask |= BOARD_SENSOR_MAG;
    }

    // calculate magnetic declination
//    deg = cfg.mag_declination / 100;
//    min = cfg.mag_declination % 100;
//    magneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
}



bool boardReadGyro( int16_t result[3] ) {
	if ( gyro.read ) {
		(*gyro.read)( result );
		return true;
	} else {
		boardFault( BOARD_FAULT_SENSOR );
		return false;
	}

}

bool boardReadAcc( int16_t result[3] ) {
	if ( acc.read ) {
		return true;
	} else {
		boardFault( BOARD_FAULT_SENSOR );
		return false;
	}
}

bool boardReadMagno( int16_t result[3] ) {
	if ( acc.read ) {
		return true;
	} else {
		boardFault( BOARD_FAULT_SENSOR );
		return false;
	}
}

bool boardReadBaro( int16_t result[1] ) {
	if ( acc.read ) {
		return true;
	} else {
		boardFault( BOARD_FAULT_SENSOR );
		return false;
	}
}


