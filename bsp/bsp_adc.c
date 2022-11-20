#include "bsp_adc.h"
#include "adc.h"


float get_cpu_temperature(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);

    uint32_t AD_Value = HAL_ADC_GetValue(&hadc1);
    float Vol_Value = (float)AD_Value * ( 3.3f / 4096.0f); // 12bits ADC 转化为电压值
    return (Vol_Value - 0.76f) / 0.0025f + 25.0f; // 电压转化为温度
}
