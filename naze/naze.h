/*
 * naze.h
 *
 *  Created on: 4 May 2013
 *      Author: Sjoerd
 */

#pragma once

#ifndef NAZE_H_
#define NAZE_H_

// for roundf()
#define __USE_C99_MATH

#include "stm32f10x_conf.h"
#include "core_cm3.h"

#include "../include/controller.h"
#include "../lib/libformat.h"
#include "../lib/libfunctions.h"
#include "../drivers/drivers.h"

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

// Afroflight32
#define LED0_GPIO   GPIOB
#define LED0_PIN    GPIO_Pin_3 // PB3 (LED)
#define LED1_GPIO   GPIOB
#define LED1_PIN    GPIO_Pin_4 // PB4 (LED)
#define BEEP_GPIO   GPIOA
#define BEEP_PIN    GPIO_Pin_12 // PA12 (Buzzer)
#define BARO_GPIO   GPIOC
#define BARO_PIN    GPIO_Pin_13

//Define certain sensors from even being included in the code

#undef SOFT_I2C                 // enable to test software i2c

// Helpful macros
#define LED0_TOGGLE              digitalToggle(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 digitalHi(LED0_GPIO, LED0_PIN);
#define LED0_ON                  digitalLo(LED0_GPIO, LED0_PIN);

#define LED1_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN);
#define LED1_OFF                 digitalHi(LED1_GPIO, LED1_PIN);
#define LED1_ON                  digitalLo(LED1_GPIO, LED1_PIN);

#ifdef BEEP_GPIO
#define BEEP_TOGGLE              digitalToggle(BEEP_GPIO, BEEP_PIN);
#define BEEP_OFF                 digitalHi(BEEP_GPIO, BEEP_PIN);
#define BEEP_ON                  digitalLo(BEEP_GPIO, BEEP_PIN);
#else
#define BEEP_TOGGLE              ;
#define BEEP_OFF                 ;
#define BEEP_ON                  ;
#endif


void nazeI2CInit( I2C_TypeDef *I2C );

#include "naze_timing.h"
#include "naze_pwm.h"
#include "naze_uart.h"
#include "naze_gpio.h"


#endif /* NAZE_H_ */
