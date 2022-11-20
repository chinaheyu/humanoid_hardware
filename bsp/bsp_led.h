#ifndef BSP_LED_H
#define BSP_LED_H

#ifdef  __cplusplus
extern "C" {
#endif
    
/**
  * @brief          点亮LED0（红色）
  * @retval         none
  */
extern void led_red_on(void);


/**
  * @brief          熄灭LED0（红色）
  * @retval         none
  */
extern void led_red_off(void);


/**
  * @brief          翻转LED0（红色）
  * @retval         none
  */
extern void led_red_toggle(void);


/**
  * @brief          点亮LED1（绿色）
  * @retval         none
  */
extern void led_green_on(void);


/**
  * @brief          熄灭LED1（绿色）
  * @retval         none
  */
extern void led_green_off(void);


/**
  * @brief          翻转LED1（绿色）
  * @retval         none
  */
extern void led_green_toggle(void);

#ifdef  __cplusplus
}  
#endif

#endif
