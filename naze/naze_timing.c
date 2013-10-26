#include "naze.h"

// cycles per microsecond
static volatile uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;

// SysTick
void SysTick_Handler(void)
{
    sysTickUptime++;
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t timeMicro()
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (72000 - cycle_cnt) / 72;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t timeMilli()
{
    return sysTickUptime;
}


void delayMicro(uint32_t us)
{
    uint32_t now = timeMicro();
    while (timeMicro() - now < us);
}

void delayMilli(uint32_t ms)
{
    while (ms--)
        delayMicro(1000);
}

void nazeTimingInit() {
	//Get the cycles cycles per micro counter
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;

    // SysTick at millisecond intervals
    SysTick_Config(SystemCoreClock / 1000);
}

