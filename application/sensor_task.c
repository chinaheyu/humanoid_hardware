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

#include "board.h"
#include "os_timer.h"
#include "event_mgr.h"
#include "event.h"
#include "cmsis_os.h"
#include "sensor_task.h"
#include "gyro.h"
#include "communication_task.h"

static void sensor_task(void const *argc);

osThreadId sensor_task_t;

struct gyro_device gyro_dev_obj;

void sensor_task_init(void)
{
    osThreadDef(SENSOR_TASK, sensor_task, osPriorityNormal, 0, 512);
    sensor_task_t = osThreadCreate(osThread(SENSOR_TASK), NULL);
}

float ahrs_run_time;

void gyro_feedback(struct ahrs_sensor* sensor)
{
    cmd_gyro_feedback_t gyro_feedback_msg;
    uint8_t* msg_buf = (uint8_t*)alloca(protocol_calculate_frame_size(sizeof(cmd_gyro_feedback_t)));

    gyro_feedback_msg.timestamp = get_timestamp();
    gyro_feedback_msg.roll = (int16_t)(sensor->roll * 1000.0f);
    gyro_feedback_msg.pitch = (int16_t)(sensor->pitch * 1000.0f);
    gyro_feedback_msg.yaw = (int16_t)(sensor->yaw * 1000.0f);
    size_t frame_size = protocol_pack_data_to_buffer(CMD_GYRO_FEEDBACK, (uint8_t*)&gyro_feedback_msg, sizeof(cmd_gyro_feedback_t), msg_buf);
    usb_interface_send(msg_buf, frame_size);
}

/**
  * @brief  sensor publisher
  * @param
  * @retval void
  */
void sensor_task(void const *argc)
{
    /* The parameters are not used. */
    (void)argc;
    TickType_t peroid = osKernelSysTick();
    ;
    struct ahrs_sensor gyro_sensor;

    static publisher_t ahrsPub;

    /* set gyro zero drift */
    bmi088_get_offset();

    imu_temp_ctrl_init();

    EventPostInit(&ahrsPub, AHRS_MSG, AHRS_MSG_LEN);
    
    gyro_device_init(&gyro_dev_obj, "GYRO_ONBOARD");

    while (1)
    {
        uint32_t time_id;

        get_period_start(&time_id);

        ahrs_update(&gyro_sensor, SENSOR_TASK_PERIOD);
        
        gyro_dev_obj.roll = gyro_sensor.roll;
        gyro_dev_obj.pitch = gyro_sensor.pitch;
        gyro_dev_obj.yaw = gyro_sensor.yaw;

        EventMsgPost(&ahrsPub, &gyro_sensor, AHRS_MSG_LEN);
        gyro_feedback(&gyro_sensor);

        ahrs_run_time = get_period_end(time_id);

        osDelayUntil(&peroid, SENSOR_TASK_PERIOD);
    }
}
