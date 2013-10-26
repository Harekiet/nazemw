#pragma once

#include "naze.h"

//3 groups with 16 pins each

#define NAZE_GPIOGROUPS 3
#define NAZE_GPIOMAX ( NAZE_GPIOGROUPS * 16 )

//Create a pin index based on a pin group and sub index
#define NAZE_PIN( _GROUP_, _PIN_ ) ( ( (_GROUP_) << 4) | (_PIN_) )

//Init all the ports and disable the ones that are used internally
void nazeGPIOInit();

//You can only disable pins once
void nazeGPIODisable( uint8_t pin );
//Disable using a gpio def and a pin number
void nazeGPIODisableDef( GPIO_TypeDef *gpio, uint32_t pin );


//Toggle pin to
void nazeGPIODigitalToggle( uint8_t pin );
void nazeGPIODigitalLo( uint8_t pin );
void nazeGPIODigitalHi( uint8_t pin );
