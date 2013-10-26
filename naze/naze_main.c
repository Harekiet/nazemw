/*
 * naze_main.c
 *
 *  Created on: 11 May 2013
 *      Author: Sjoerd
 */

#include "naze.h"

typedef struct gpio_config_t
{
    GPIO_TypeDef *gpio;
    uint16_t pin;
    GPIOMode_TypeDef mode;
} gpio_config_t;

static void initDefaults() {
    GPIO_InitTypeDef GPIO_InitStructure;
    uint32_t i;

    static const gpio_config_t const gpio_cfg[] = {
        { LED0_GPIO, LED0_PIN, GPIO_Mode_Out_PP },
        { LED1_GPIO, LED1_PIN, GPIO_Mode_Out_PP },
        { BEEP_GPIO, BEEP_PIN, GPIO_Mode_Out_OD },
    };
    uint8_t gpio_count = sizeof(gpio_cfg) / sizeof(gpio_cfg[0]);

    // Turn on clocks for stuff we use
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_ClearFlag();

    // Make all GPIO in by default to save power and reduce noise
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Turn off JTAG port 'cause we're using the GPIO for leds
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    // Configure gpio
    for (i = 0; i < gpio_count; i++) {
        GPIO_InitStructure.GPIO_Pin = gpio_cfg[i].pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_Mode = gpio_cfg[i].mode;
        GPIO_Init(gpio_cfg[i].gpio, &GPIO_InitStructure);
    }

    // sleep for 100ms
    delayMilli(100);
}


int main(void) {
    // This is needed because some shit inside Keil startup fucks with SystemCoreClock, setting it back to 72MHz even on HSI.
    SystemCoreClockUpdate();
    //Init the timing for delays
	nazeTimingInit();

	//Put the gpio ports in a default state
	initDefaults();

	//Make sure the leds and beeper are off
    LED0_OFF;
    LED1_OFF;
    BEEP_OFF;

	//Init internal devices/buses

    nazeI2CInit( I2C2 );


    //Basic hardware enabled, now check for drivers
    nazeDriversInit( 0 );

    //Hand over control to the controller code

    controllerMain();

    //Should never arrive here
    boardFault( BOARD_FAULT_FATAL );
    return 0;
}


//Fault handler from sysem code

void HardFault_Handler() {
	boardFault( BOARD_FAULT_FATAL );
}
