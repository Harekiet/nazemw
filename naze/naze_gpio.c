#include "naze.h"

static GPIO_TypeDef* const gpioGroups[ NAZE_GPIOGROUPS ] = {
	GPIOA, GPIOB, GPIOC
};

//Bitmask indicating what pins are already disabled
static uint16_t pinDisabled[ NAZE_GPIOGROUPS ];

//Accessing illegal pins will cause failure
static void nazeGPIOValidPin( uint8_t pin ) {
	if ( pin < NAZE_GPIOMAX ) {
		uint8_t group = pin >> 4;
		pin &= 0xf;
		//Only return if the pin hasn't bene disabled yet
		if ( !(pinDisabled[ group ] & ( 1 << pin ) ) )
			return;
	}
	boardFault( BOARD_FAULT_ILLEGAL );
}

void nazeGPIODisable( uint8_t pin ) {
	nazeGPIOValidPin( pin );
	uint8_t group = pin >> 4;
	pin &= 0xf;
	uint16_t mask = 1 << pin;
	pinDisabled[ group ] |= mask;
}

void nazeGPIODisableDef(  GPIO_TypeDef *gpio, uint32_t pin ) {
	//TODO

}

void nazeGPIODigitalToggle( uint8_t pin ) {
	nazeGPIOValidPin( pin );
	GPIO_TypeDef* gpio = gpioGroups[ pin >> 4 ];
	uint32_t mask = 1 << ( pin & 0xf );
	digitalToggle(gpio, mask);
}

void nazeGPIODigitalLo( uint8_t pin ) {
	nazeGPIOValidPin( pin );
	GPIO_TypeDef* gpio = gpioGroups[ pin >> 4 ];
	uint32_t mask = 1 << ( pin & 0xf );
	digitalLo(gpio, mask);
}

void nazeGPIODigitalHi( uint8_t pin ) {
	nazeGPIOValidPin( pin );
	GPIO_TypeDef* gpio = gpioGroups[ pin >> 4 ];
	uint32_t mask = 1 << ( pin & 0xf );
	digitalHi(gpio, mask);
}




