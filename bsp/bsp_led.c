#include "bsp_led.h"
#include "gpio.h"

/**
  * @brief          点亮LED0（红色）
  * @retval         none
  */
void led_red_on(void)
{
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
}


/**
  * @brief          熄灭LED0（红色）
  * @retval         none
  */
void led_red_off(void)
{
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
}


/**
  * @brief          翻转LED0（红色）
  * @retval         none
  */
void led_red_toggle(void)
{
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
}


/**
  * @brief          点亮LED1（绿色）
  * @retval         none
  */
void led_green_on(void)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
}


/**
  * @brief          熄灭LED1（绿色）
  * @retval         none
  */
void led_green_off(void)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}


/**
  * @brief          翻转LED1（绿色）
  * @retval         none
  */
void led_green_toggle(void)
{
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}
