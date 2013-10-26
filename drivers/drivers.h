/*
 * drivers.h
 *
 *  Created on: 4 May 2013
 *      Author: Sjoerd
 */

#pragma once

#ifndef DRIVERS_H_
#define DRIVERS_H_

#include "../include/controller.h"

typedef void (* sensorInitFuncPtr)(void);                   // sensor init prototype
typedef void (* sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype
typedef void (* baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature);             // baro calculation (filled params are pressure and temperature)

typedef struct sensor_t
{
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
//    sensorReadFuncPtr align;                                // sensor align
    sensorReadFuncPtr temperature;                          // read temperature if available
    float scale;                                            // scalefactor (currently used for gyro only, todo for accel)
} sensor_t;


typedef struct baro_t
{
    uint16_t ut_delay;
    uint16_t up_delay;
    sensorInitFuncPtr start_ut;
    sensorInitFuncPtr get_ut;
    sensorInitFuncPtr start_up;
    sensorInitFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baro_t;

#include "drv_adxl345.h"
#include "drv_hmc5883l.h"
#include "drv_l3g4200d.h"
#include "drv_mma845x.h"
#include "drv_mpu3050.h"
#include "drv_mpu6050.h"
#include "drv_ms5611.h"


#endif /* DRIVERS_H_ */
