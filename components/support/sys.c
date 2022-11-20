#include "sys.h"
#include "cmsis_os.h"


uint32_t get_time_us(void)
{
    return TIM14->CNT;
}

uint32_t get_time_ms(void)
{
    return HAL_GetTick();
}

float get_time_ms_us(void)
{
    return get_time_ms() + get_time_us() / 1000.0f;
}

unsigned long time_us(void)
{
    return get_time_ms() * 1000 + get_time_us();
}

void configureTimerForRunTimeStats(void)
{
    
}

unsigned long getRunTimeCounterValue(void)
{
    return time_us();
}
