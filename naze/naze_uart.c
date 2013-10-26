#include "board.h"

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/
#define UART_RX_SIZE    256
#define UART_TX_SIZE    256

typedef struct {
	volatile uint8_t rx[UART_RX_SIZE];
	volatile uint8_t tx[UART_TX_SIZE];
	uint16_t txTail;
	uint16_t txHead;
	//Position of last read
	uint16_t rxPos;
} UartBuffer_t;

typedef struct {
	UartBuffer_t* buffer;
	USART_TypeDef* usart;
	DMA_Channel_TypeDef* txDMA;
	DMA_Channel_TypeDef* rxDMA;
	uint8_t ioPin;
	uint8_t txIRQ;
} UartControl_t;

static UartBuffer_t uartBuffer[2];
static uint16_t uartTXOverflow;

static const UartControl_t uartControl[2] = {
		{	&uartBuffer[0], USART1, DMA1_Channel4, DMA1_Channel5, 9, DMA1_Channel4_IRQn },
		{	&uartBuffer[1], USART2, DMA1_Channel6, DMA1_Channel7, 2, DMA1_Channel6_IRQn },
};

//Start the TX DMA on data or dma IRQ
static void uartTXDMA( const UartControl_t* control )
{
	DMA_Channel_TypeDef* dma = control->txDMA;
	UartBuffer_t* buffer = control->buffer;

    dma->CMAR = (uint32_t)&buffer->rx[ buffer->txTail ];
    //Crossing the end yes/no
    if ( buffer->txHead > buffer->txTail ) {
        dma->CNDTR = buffer->txHead - buffer->txTail;
        buffer->txTail = buffer->txHead;
    } else {
        dma->CNDTR = UART_TX_SIZE - buffer->txTail;
        buffer->txTail = 0;
    }
    //Enable the channel for more data
    DMA_Cmd(dma, ENABLE);
}

//Called from end of dma irq to check if we need to start up a new transfer
static void uartTXIRQ( const UartControl_t* control ) {
	UartBuffer_t* buffer = control->buffer;

    if ( buffer->txHead != buffer->txTail ) {
    	uartTXDMA( control );
    }
}

static void uartSetSpeed( uint32_t speed, const UartControl_t* control ) {
    USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = speed;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init( control->usart, &USART_InitStructure );
}


static void uartInit( uint32_t speed, const UartControl_t* control )
{
	//	UartBuffer_t* buffer, uint16_t ioPin, USART_TypeDef* usart, DMA_Channel_TypeDef *rxDMA, DMA_Channel_TypeDef* txDMA ) {

    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //Clear the buffer
    memset( control->buffer, 0, sizeof( *control->buffer ) );

    //Assume sequential pins
    GPIO_InitStructure.GPIO_Pin = control->ioPin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = control->ioPin + 1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = control->txIRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //Setup the speed
    uartSetSpeed( speed, control );

    // Receive DMA into a circular buffer
    DMA_DeInit( control->rxDMA );
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(control->usart->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)control->buffer->rx;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_BufferSize = UART_RX_SIZE;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_Init( control->rxDMA, &DMA_InitStructure);

    DMA_Cmd( control->rxDMA, ENABLE);
    USART_DMACmd( control->usart, USART_DMAReq_Rx, ENABLE);
    control->buffer->rxPos = DMA_GetCurrDataCounter( control->rxDMA);

    // Transmit DMA
    DMA_DeInit( control->txDMA );
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(control->usart->DR);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_Init( control->txDMA, &DMA_InitStructure);
    DMA_ITConfig( control->txDMA, DMA_IT_TC, ENABLE);
    control->txDMA->CNDTR = 0;
    USART_DMACmd( control->usart, USART_DMAReq_Tx, ENABLE);

    USART_Cmd( control->usart, ENABLE);
}

static uint16_t uartAvailable( const UartControl_t* control )
{
    return ( DMA_GetCurrDataCounter( control->rxDMA) != control->buffer->rxPos) ? true : false;
}

static bool uartTransmitEmpty( const UartControl_t* control )
{
    return (control->buffer->txTail  == control->buffer->txHead );
}

static uint8_t uartRead( const UartControl_t* control )
{
    uint8_t ch;
    UartBuffer_t* buffer = control->buffer;

    ch = buffer->rx[ UART_RX_SIZE - buffer->rxPos];
    // go back around the buffer
    if (--buffer->rxPos == 0)
    	buffer->rxPos = UART_RX_SIZE;

    return ch;
}

static void uartWrite( const UartControl_t* control, uint8_t ch)
{
    UartBuffer_t* buffer = control->buffer;
    uint16_t next = ( buffer->txHead + 1 ) % UART_TX_SIZE;
    if ( next != buffer->txTail ) {
        buffer->tx[ buffer->txHead ] = ch;
        buffer->txHead = next;
    } else {
    	uartTXOverflow++;
    }

    // if DMA wasn't enabled, fire it up
    if (!(control->txDMA->CCR & 1))
        uartTXDMA( control );
}



//DMA TX IRQ Handlers to restart when there's more data
void DMA1_Channel4_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);

    uartTXIRQ( uartControl + 0 );
}

void DMA1_Channel6_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC6);
    DMA_Cmd(DMA1_Channel6, DISABLE);

    uartTXIRQ( uartControl + 1 );
}


/*
 *
 * Serial interface that is exported to the outside
 */


//Total amount of serial ports
uint8_t serialGetCount() {
	return 2;
}

void serialSetup( uint8_t index, uint32_t rate ) {
	if ( index == 0 ) {
		uartInit( rate, uartControl + 0 );
	} else if ( index == 1 ) {
		//Enable the 2nd uart clock
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	    uartInit( rate, uartControl + 1 );
	} else {
		boardFault( BOARD_FAULT_ILLEGAL );
	}
}

void serialWrite( uint8_t index, uint32_t count, const void* data ) {
	const uint8_t* input = (uint8_t*) data;
	if ( index < 2 ) {
		const UartControl_t* control = uartControl + index;
		while( count-- ) {
			uartWrite( control, *input++ );
		}
	} else {
		boardFault( BOARD_FAULT_ILLEGAL );
	}
}


uint8_t serialRead( uint8_t index, uint8_t count, void* data ) {
	const uint8_t* input = (uint8_t*) data;
	if ( index < 2 ) {
		const UartControl_t* control = uartControl + index;
		while( count-- ) {
			uartWrite( control, *input++ );
		}
	} else {
		boardFault( BOARD_FAULT_ILLEGAL );
	}
}

