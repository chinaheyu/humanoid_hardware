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
#include "app_manage.h"
#include "board.h"
#include "init.h"
#include "easyflash.h"
#include "shell.h"
#include "drv_io.h"

#include "sensor_task.h"
#include "communication_task.h"

#include "SEGGER_SYSVIEW.h"

#include "log.h"

void hw_init(void);

uint8_t get_appid_from_flash()
{
    size_t read_len = 0;
    uint8_t application_id;
    ef_get_env_blob(APPID_KEY, &application_id, 1, &read_len);

    if (read_len == 1)
    {
        return application_id;
    }
    return 0;
}

void task_init(void)
{
    communication_task_init();
    uint8_t appid = get_appid_from_flash();
    app_task_init(appid);
    
    // Initializa finished.
    beep_set_tune(500, 150);
    osDelay(150);
    beep_set_tune(400, 100);
    osDelay(150);
    beep_set_tune(300, 50);
    osDelay(150);
    beep_set_tune(0, 0);
}

void sys_task(void)
{
    thread_cli_init();
    soft_timer_FreeRTOS_init();
    sensor_task_init();
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    debug_raw_printf("\r\n %s Stack Over flow! \r\n", pcTaskName);
    while (1)
        ;
}

void vApplicationMallocFailedHook(void)
{
    debug_raw_printf("\r\n FreeRTOS Malloc Failed! \r\n");
}

/**
  * @brief  all task init. This thread is called in main.c
  * @param
  * @retval void
  */
void services_task(void const *argument)
{
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    SEGGER_SYSVIEW_Conf();
    hw_init();
    sys_task();
    task_init();

    log_printf("\r\n>>> ");
    /* USER CODE BEGIN services_task */
    /* Infinite loop */
    for (;;)
    {
        osDelay(100);
    }
    /* USER CODE END services_task */
}

void hw_init(void)
{
    board_config();
    easyflash_init();
}
