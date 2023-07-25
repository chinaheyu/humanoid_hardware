/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "main.h"
#include "can.h"
#include "board.h"
#include "init.h"
#include "os_timer.h"
#include "event_mgr.h"
#include "event.h"
#include "shell.h"

#include "app_manage.h"

#include "log.h"

static long long timestamp_sync;
static long long machine_time_sync;

static struct app_manage *board_app;

static publisher_t dbusPub;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_6)
    {
        if (board_app->user_input_callback)
        {
            board_app->user_input_callback();
        }
    }

    if (GPIO_Pin == GPIO_PIN_0)
    {
        if (board_app->user_key_callback)
        {
            board_app->user_key_callback();
        }
    }
}

/* usart3: receive dbus data */
int32_t dr16_rx_data_by_uart(uint8_t *buff, uint16_t len)
{
    EventMsgPost(&dbusPub, buff, DBUS_MSG_LEN);
    if (board_app->dbus_rx_complete)
    {
        board_app->dbus_rx_complete();
    }
    return 0;
}

int32_t dr16_rx_data_by_can(uint8_t *buff, uint16_t len)
{
    EventMsgPost(&dbusPub, buff, DBUS_MSG_LEN);
    if (board_app->dbus_rx_complete)
    {
        board_app->dbus_rx_complete();
    }
    return 0;
}

/**
  * @brief  usart1 interupt, debug shell
  * @param
  * @retval void
  */
uint32_t usart1_rx_callback(uint8_t *buff, uint16_t len)
{
    shell_interupt(buff, len);
    return 0;
}

static int status_led_period = 300;

/**
  * @brief  type c board init
  * @param
  * @retval void
  */
void board_config(void)
{
    /* system log */
    usart1_manage_init();
    log_printf("\r\n\r\n"
               "************Humanoid Robot Shell************\r\n");
    log_printf("* Copy right: All right reserved.\r\n");
    log_printf("* Release Time: %s.\r\n", __TIME__);
    log_printf("********************************************\r\n");

    board_app = get_current_app();

    soft_timer_init();
    usart6_manage_init();
    can_manage_init();
    pwm_device_init();
    bmi088_device_init();

    /* DBUS message */
    EventPostInit(&dbusPub, DBUS_MSG, DBUS_MSG_LEN);

    //soft_timer_register(usb_tx_flush, NULL, 1);
    soft_timer_register(beep_ctrl_times, NULL, 1);
    soft_timer_register(green_led_toggle, &status_led_period, 5);

    usart1_rx_callback_register(usart1_rx_callback);
}

void set_timestamp(long long microseconds)
{
    timestamp_sync = microseconds;
    machine_time_sync = 1000 * get_machine_time_ms() + get_machine_time_us();
}

uint32_t get_machine_time_us(void)
{
    return TIM9->CNT;
}

uint32_t get_machine_time_ms(void)
{
    return HAL_GetTick();
}

float get_machine_time_ms_us(void)
{
    return get_machine_time_ms() + get_machine_time_us() / 1000.0f;
}

long long get_timestamp(void)
{
    // 不知道为什么STM32和Jetson AGX Orin的时钟不同步，乘个系数校准一下
    long long microseconds = (1000.0 * get_machine_time_ms() + get_machine_time_us() - machine_time_sync) * (1000000.0 / 1000045.0) + timestamp_sync;
    return microseconds;
}
