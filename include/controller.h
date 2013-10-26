#pragma once

#define __USE_C99_MATH

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#ifndef NULL
#define	NULL	0
#endif

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) >= 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define RADX10 (M_PI / 1800.0f)                  // 0.001745329252f

//Routines to access all the board functionality

enum {
	BOARD_IO_DISABLE = 0,
	BOARD_IO_PULLUP,
	BOARD_IO_ILLEGAL,
};

enum {
	BOARD_SENSOR_GYRO = 0x001,
	BOARD_SENSOR_ACC = 0x002,
	BOARD_SENSOR_MAG = 0x004,
	BOARD_SENSOR_BARO = 0x008,
};

enum {
	ACC_1G = 256,
};

enum {
	//Illegal parameter given to one of the board calls
	BOARD_FAULT_ILLEGAL,
	//A sensor failed, better land that shit
	BOARD_FAULT_SENSOR,
	//Board encountered a fatal crash, you are doomed!
	BOARD_FAULT_FATAL,
	//Controllers can add their own faults beyond this
	BOARD_FAULT_LAST,
};

//Settings used by the board, store in your own config table, you'll need to supply a pointer to this during init
typedef struct  {


} BoardConfig_t;

//This function will need to be implemented by the controller code, board will call it at startup
void controllerMain();
//This will be called when a fault occurs use it to power down motors or whatever
void controllerFault( uint32_t code );
//Signal that the data is still coming in correctly
void controllerGood();



//Let the board do it's initialisation with the supplied config
void boardInit( BoardConfig_t* config );
//Call this in your main loop to give the board some processing time
void boardLoop();
//Trigger a reset of the board
void boardReset();
//Goto the board's bootloader, else just reset
void boardBootLoader();

//Trigger a failure, will be forwarded to controller code
void boardFault( uint32_t code );

//Save/Load settings into eeprom/flash
//Returns false for too large a size or just failing to write
bool boardSaveSettings( uint16_t version, uint16_t size, const void* data );
//Returns false when there's no valid flash for this version/size
bool boardLoadSettings( uint16_t version, uint16_t size, void* data );

//Delay milliseconds
void delayMilli( uint32_t count );
//Delay microseconds
void delayMicro( uint32_t count );

//Access the internal counters for these that start from 0 at startup
uint32_t timeMilli();
//Micro will overflow so keep that in mind
uint32_t timeMicro();


//Maximum amount of io ports you can access
uint8_t boardGetIOCount();
//Is a certain IO channel still active, they can be disabled if they are used for other purposes
bool boardGetIOValid( uint8_t index );

//Set a specific mode, returns false if this not a valid io channel
bool boardSetIOMode( uint8_t index, uint8_t mode );
//Returns illegal when index is not a valid channel
uint8_t boardGetIOMode( uint8_t index );

//0xff is cppm else there's just regular pwm input
//Return amounts of pwm's actually being available
uint8_t boardPWMInputCount( uint8_t count );
uint16_t boardPWMInputSingle( uint8_t index );
void boardPWMInputRate( uint8_t index, uint16_t rate );
void boardPWMInputMultiple( uint8_t index, uint8_t count, uint16_t* delay );

uint8_t boardPWMSetOutputCount( uint8_t count );
//Set the specific rate of a certain channel
void boardPWMSetOutputRate( uint8_t index, uint16_t rate );
//Set the pwm delay
void boardPWMSetOutputSingle( uint8_t index, uint16_t delay );
void boardPWMSetOutputMultiple( uint8_t index, uint8_t count, const uint16_t* delay );


//Total amount of serial ports
uint8_t serialGetCount();
//Setup port at a specific baudrate
void serialSetup( uint8_t index, uint32_t rate );
//If you write too much data this function will stall
void serialWrite( uint8_t index, uint32_t count, const void* data );
//How many bytes are leeft to read
uint8_t serialAvailable( uint8_t index );
//Read from the serial port, returns amount of bytes remaining
uint8_t serialRead( uint8_t index, uint8_t count, void* data );

//ADC access
uint8_t boardADCCount();
bool boardADCSetup( uint8_t index, bool enabled );
uint8_t boardADCRead( uint8_t index );

//Write the i2c bus
bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data);
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, void* buf);
//bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
//bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf);
uint16_t i2cGetErrorCounter();

//Write to the beeper on the board
void boardBeepOn();
void boardBeepOff();
void boardBeepToggle();

//Interface the leds on the board
uint8_t boardLedCount();
void boardLedToggle( uint8_t index );
void boardLedOn( uint8_t index );
void boardLedOff( uint8_t index );

//Bitmask of available sensors
uint32_t boardSensors();

//Angles are all given in * 100
bool boardReadGyro( int16_t result[3] );
bool boardReadAcc( int16_t result[3] );
bool boardReadMagno( int16_t result[3] );
bool boardReadBaro( int16_t result[1] );






