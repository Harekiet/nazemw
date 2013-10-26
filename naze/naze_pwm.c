#include "board.h"

#define PULSE_1MS       (1000) // 1ms pulse width

/* FreeFlight/Naze32 timer layout
    TIM2_CH1    RC1             PWM1
    TIM2_CH2    RC2             PWM2
    TIM2_CH3    RC3/UA2_TX      PWM3
    TIM2_CH4    RC4/UA2_RX      PWM4
    TIM3_CH1    RC5             PWM5
    TIM3_CH2    RC6             PWM6
    TIM3_CH3    RC7             PWM7
    TIM3_CH4    RC8             PWM8
    TIM1_CH1    PWM1            PWM9
    TIM1_CH4    PWM2            PWM10
    TIM4_CH1    PWM3            PWM11
    TIM4_CH2    PWM4            PWM12
    TIM4_CH3    PWM5            PWM13
    TIM4_CH4    PWM6            PWM14

    // RX1  TIM2_CH1 PA0 [also PPM] [also used for throttle calibration]
    // RX2  TIM2_CH2 PA1
    // RX3  TIM2_CH3 PA2 [also UART2_TX]
    // RX4  TIM2_CH4 PA3 [also UART2_RX]
    // RX5  TIM3_CH1 PA6 [also ADC_IN6]
    // RX6  TIM3_CH2 PA7 [also ADC_IN7]
    // RX7  TIM3_CH3 PB0 [also ADC_IN8]
    // RX8  TIM3_CH4 PB1 [also ADC_IN9]

    // Outputs
    // PWM1 TIM1_CH1 PA8
    // PWM2 TIM1_CH4 PA11
    // PWM3 TIM4_CH1 PB6? [also I2C1_SCL]
    // PWM4 TIM4_CH2 PB7 [also I2C1_SDA]
    // PWM5 TIM4_CH3 PB8
    // PWM6 TIM4_CH4 PB9

    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM2 4 channels
    TIM3 4 channels
    TIM1 2 channels
    TIM4 4 channels

    Configuration maps:

    1) multirotor PPM input
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    2) multirotor PPM input with more servos
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for servos

    2) multirotor PWM input
    PWM1..8 used for input
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    3) airplane / flying wing w/PWM
    PWM1..8 used for input
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos

    4) airplane / flying wing with PPM
    PWM1 used for PPM
    PWM5..8 used for servos
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos
*/

typedef void pwmCallbackPtr(uint8_t port, uint16_t capture);

// This indexes into the read-only hardware definition structure in drv_pwm.c, as well as into pwmPorts[] structure with dynamic data.
enum {
    PWM1 = 0,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    PWM6,
    PWM7,
    PWM8,
    PWM9,
    PWM10,
    PWM11,
    PWM12,
    PWM13,
    PWM14,
    MAX_PORTS
};

//Inputs start from PWM0
//Outputs start from PWM8

#define MAX_OUTPUTS  ( MAX_PORTS - 1 )
#define MAX_INPUTS  8
#define OUTPUT_OVERLAP ( MAX_OUTPUTS - MAX_INPUTS )

typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *gpio;
    uint32_t pin;
    uint8_t channel;
    uint8_t irq;
} pwmHardware_t;

static pwmHardware_t const timerHardware[ MAX_PORTS ]  = {
    { TIM2, GPIOA, GPIO_Pin_0, TIM_Channel_1, TIM2_IRQn, },          // PWM1
    { TIM2, GPIOA, GPIO_Pin_1, TIM_Channel_2, TIM2_IRQn },          // PWM2
    { TIM2, GPIOA, GPIO_Pin_2, TIM_Channel_3, TIM2_IRQn },          // PWM3
    { TIM2, GPIOA, GPIO_Pin_3, TIM_Channel_4, TIM2_IRQn },          // PWM4
    { TIM3, GPIOA, GPIO_Pin_6, TIM_Channel_1, TIM3_IRQn },          // PWM5
    { TIM3, GPIOA, GPIO_Pin_7, TIM_Channel_2, TIM3_IRQn },          // PWM6
    { TIM3, GPIOB, GPIO_Pin_0, TIM_Channel_3, TIM3_IRQn },          // PWM7
    { TIM3, GPIOB, GPIO_Pin_1, TIM_Channel_4, TIM3_IRQn },          // PWM8
    { TIM1, GPIOA, GPIO_Pin_8, TIM_Channel_1, TIM1_CC_IRQn, },       // PWM9
    { TIM1, GPIOA, GPIO_Pin_11, TIM_Channel_4, TIM1_CC_IRQn },      // PWM10
    { TIM4, GPIOB, GPIO_Pin_6, TIM_Channel_1, TIM4_IRQn, },          // PWM11
    { TIM4, GPIOB, GPIO_Pin_7, TIM_Channel_2, TIM4_IRQn, },          // PWM12
    { TIM4, GPIOB, GPIO_Pin_8, TIM_Channel_3, TIM4_IRQn, },          // PWM13
    { TIM4, GPIOB, GPIO_Pin_9, TIM_Channel_4, TIM4_IRQn, },          // PWM14
};


//The currently active captures
static uint16_t captures[MAX_INPUTS];
//ccr pointers of the outputs
static volatile uint16_t *outputs[MAX_OUTPUTS];

static uint8_t numOutputs = 0;
static uint8_t numInputs = 0;
static uint8_t inputCheckMask = 0;
static bool ppmActive = 0;

// external vars (ugh)
extern int16_t failsafeCnt;

//Setup the time base for a timer
static void pwmTimeBase(TIM_TypeDef *tim, uint32_t period)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; // all timers run at 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

//Enable a specific irq
static void pwmNVICConfig(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//Setup 1 of the 4 outputs of a timer
static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = value;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    switch (channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
    }
}

static void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

//Setup a specific gpio pin in in/output pwm mode
static void pwmGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin, bool input)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pin;
    if (input)
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    else
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(gpio, &GPIO_InitStructure);

    //Register the pin as having been disabled
    nazeGPIODisableDef( gpio, pin );
}

static void pwmOutConfig(uint8_t port, uint16_t period, uint16_t value)
{
	volatile uint16_t ** ccr = &outputs[ port ];
    const pwmHardware_t* hw = &timerHardware[ port ];

    pwmTimeBase( hw->tim, period);
    pwmGPIOConfig( hw->gpio, hw->pin, 0);
    pwmOCConfig( hw->tim, hw->channel, value);
    // Needed only on TIM1
    if ( hw->tim == TIM1) {
        TIM_CtrlPWMOutputs( hw->tim, ENABLE);
    }
    TIM_Cmd( hw->tim, ENABLE);

    switch ( hw->channel) {
        case TIM_Channel_1:
            *ccr = &hw->tim->CCR1;
            break;
        case TIM_Channel_2:
            *ccr = &hw->tim->CCR2;
            break;
        case TIM_Channel_3:
            *ccr = &hw->tim->CCR3;
            break;
        case TIM_Channel_4:
            *ccr = &hw->tim->CCR4;
            break;
    }
}

static void pwmInConfig( uint8_t port )
{
    const pwmHardware_t* hw = &timerHardware[ port ];

    pwmTimeBase(hw->tim, 0xFFFF);
    pwmGPIOConfig(hw->gpio, hw->pin, 1);
    pwmICConfig(hw->tim, hw->channel, TIM_ICPolarity_Rising);
    TIM_Cmd(hw->tim, ENABLE);
    pwmNVICConfig(hw->irq);

    switch (hw->channel) {
        case TIM_Channel_1:
            TIM_ITConfig(hw->tim, TIM_IT_CC1, ENABLE);
            break;
        case TIM_Channel_2:
            TIM_ITConfig(hw->tim, TIM_IT_CC2, ENABLE);
            break;
        case TIM_Channel_3:
            TIM_ITConfig(hw->tim, TIM_IT_CC3, ENABLE);
            break;
        case TIM_Channel_4:
            TIM_ITConfig(hw->tim, TIM_IT_CC4, ENABLE);
            break;
    }
}

static void ppmCallback( uint16_t capture)
{
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t chan = 0;

    last = now;
    now = capture;
    diff = now - last;

    if (diff > 2700) { // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960 "So, if you use 2.5ms or higher as being the reset for the PPM stream start, you will be fine. I use 2.7ms just to be safe."
        chan = 0;
    } else {
        if (diff > 750 && diff < 2250 && chan < 8) {   // 750 to 2250 ms is our 'valid' channel range
            captures[chan] = diff;
        }
        chan++;
        if ( chan == 8 ) {
        	// Signal all is good
        	controllerGood();
        }
    }
}

static void pwmCallback(uint8_t port, uint16_t capture)
{
	//Counter value when a rise was detected
	static uint16_t inputRise[ MAX_INPUTS ];
	//Bit mask indicating an active rise
	static uint8_t inputState;
	//Bit mask indicating which ports got correct data
	static uint8_t doneState;

	const pwmHardware_t* hw = &timerHardware[ port ];
    const uint8_t mask = 1 << port;

	if ( inputState & mask ) {
		inputRise[ port ] = capture;
        pwmICConfig(hw->tim, hw->channel, TIM_ICPolarity_Falling);
    } else {
        // compute delay
    	captures[port] = capture - inputRise[port];
        pwmICConfig(hw->tim, hw->channel, TIM_ICPolarity_Rising);

        doneState |= mask;
        //When all inputs have arrived it's time signal all is good
        if ( ( doneState ^ inputCheckMask ) == 0 ) {
        	controllerGood();
        	doneState = 0;
        }
    }
	//Switch state for next time
	inputState ^= mask;
}

//IRQ Handlers
//No pwm inputs on this timer

void TIM1_CC_IRQHandler(void)
{
#if 0
	return;
    uint8_t port;

    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) {
        port = PWM9;
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        pwmPorts[port].callback(port, TIM_GetCapture1(TIM1));
    } else if (TIM_GetITStatus(TIM1, TIM_IT_CC4) == SET) {
        port = PWM10;
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
        pwmPorts[port].callback(port, TIM_GetCapture4(TIM1));
    }
#endif
}

static void pwmTIMxHandler(TIM_TypeDef *tim, uint8_t portBase)
{
    int8_t port;
    uint16_t capture;

    // Generic CC handler for TIM2,3,4
    if (TIM_GetITStatus(tim, TIM_IT_CC1) == SET) {
        port = portBase + 0;
        TIM_ClearITPendingBit(tim, TIM_IT_CC1);
        capture = TIM_GetCapture1(tim);
    } else if (TIM_GetITStatus(tim, TIM_IT_CC2) == SET) {
        port = portBase + 1;
        TIM_ClearITPendingBit(tim, TIM_IT_CC2);
        capture = TIM_GetCapture2(tim);
    } else if (TIM_GetITStatus(tim, TIM_IT_CC3) == SET) {
        port = portBase + 2;
        TIM_ClearITPendingBit(tim, TIM_IT_CC3);
        capture = TIM_GetCapture3(tim);
    } else if (TIM_GetITStatus(tim, TIM_IT_CC4) == SET) {
        port = portBase + 3;
        TIM_ClearITPendingBit(tim, TIM_IT_CC4);
        capture = TIM_GetCapture4(tim);
    } else {
    	return;
    }
    if ( ppmActive ) {
    	ppmCallback( capture );
    } else {
    	pwmCallback( port, capture );
    }
}

void TIM2_IRQHandler(void)
{
    pwmTIMxHandler(TIM2, PWM1); // PWM1..4
}

void TIM3_IRQHandler(void)
{
    pwmTIMxHandler(TIM3, PWM5); // PWM5..8
}

#if 0
//TIM4 can't be used for inputs
void TIM4_IRQHandler(void)
{
    pwmTIMxHandler(TIM4, PWM11); // PWM11..14
}
#endif


//External interface handlers

uint8_t boardPWMInputCount( uint8_t count ) {
	//Can't do a double input
	if ( numInputs ) {
		boardFault( BOARD_FAULT_ILLEGAL );
	}
	if ( count == 0xff ) {
		count = 1;
		ppmActive = true;
	}
	uint8_t left = MAX_PORTS - numOutputs;
	if ( left > MAX_INPUTS )
		left = MAX_INPUTS;
	if ( count > left ) {
		boardFault( BOARD_FAULT_ILLEGAL );
	}

	numInputs = count;
	inputCheckMask = ( 1 << count ) - 1;

	if ( ppmActive ) {
		pwmInConfig( 0 );
		return 8;
	} else {
		uint8_t i;
		for ( i = 0; i < count; i++ ) {
			pwmInConfig( i );
		}
		return count;
	}
}

uint16_t boardPWMInputSingle( uint8_t index ) {
	if ( index < numInputs ) {
		return captures[ index ];
	} else {
		boardFault( BOARD_FAULT_ILLEGAL );
		return 0;
	}
}

void boardPWMInputRate( uint8_t index, uint16_t rate ) {

}

void boardPWMInputMultiple( uint8_t index, uint8_t count, uint16_t* delay ) {
	if ( ( index + count ) < numInputs ) {
		for ( ; count > 0; index++, count-- ) {
			*delay++ = captures[ index ];
		}
	} else {
		boardFault( BOARD_FAULT_ILLEGAL );
	}
}

uint8_t boardPWMSetOutputCount( uint8_t count ) {
	uint8_t i;

	//Can't do a double output
	if ( numOutputs ) {
		boardFault( BOARD_FAULT_ILLEGAL );
	}
	uint8_t left = MAX_OUTPUTS - numInputs;
	if ( left > count ) {
		boardFault( BOARD_FAULT_ILLEGAL );
	}
	numOutputs = count;
	for ( i = 0; i < count; i++ ) {
		pwmOutConfig( i, 1000000 / 500, PULSE_1MS );
	}
	return count;
}

//Set the specific rate of a certain channel
void boardPWMSetOutputRate( uint8_t index, uint16_t rate ) {

}

//Set the pwm delay
void boardPWMSetOutputSingle( uint8_t index, uint16_t delay ) {
	if ( index < numOutputs ) {
		outputs[index][0] = delay;
	} else {
		boardFault( BOARD_FAULT_ILLEGAL );
	}
}

void boardPWMSetOutputMultiple( uint8_t index, uint8_t count, const uint16_t* delay ) {
	if ( ( index + count ) < numOutputs ) {
		for ( ; count > 0; index++, count-- ) {
			outputs[index][0] = *delay++;
		}
	} else {
		boardFault( BOARD_FAULT_ILLEGAL );
	}
}


