#include "communication_task.h"
#include <cmsis_os.h>
#include <alloca.h>
#include "mem_mang.h"
#include "usbd_cdc_if.h"
#include "ahrs.h"
#include "event_mgr.h"
#include "event.h"
#include "os_timer.h"
#include "motor.h"
#include "gyro.h"

#define PROTOCOL_MAX_FRAME_LENGTH (1000)
static protocol_stream_t unpack_stream_object;
static struct ahrs_sensor gyro_sensor_data;
osThreadId communication_task_t;

void sync_machine_time(uint8_t *p_data, size_t len)
{
    // Set timestamp
    cmd_sync_t* msg = (cmd_sync_t*)p_data;
    set_timestamp(msg->timestamp);
    
    // Feedback
    size_t frame_size = protocol_calculate_frame_size(len);
    uint8_t* buffer = (uint8_t*)alloca(frame_size);
    protocol_pack_data_to_buffer(CMD_SYNC, p_data, len, buffer);
    usb_interface_send(buffer, frame_size);
}

void dispatch_frame(uint16_t cmd_id, uint8_t *p_data, size_t len)
{
    cmd_motor_mit_t* motor_mit_msg;
    switch (cmd_id)
    {
    case CMD_SYNC:
        sync_machine_time(p_data, len);
        break;
    case CMD_RESET:
        HAL_NVIC_SystemReset();
        break;
    case CMD_MOTOR_MIT:
        motor_mit_msg = (cmd_motor_mit_t*)p_data;
        motor_mit_control(motor_mit_msg->id, motor_mit_msg->position / 1000.0f, motor_mit_msg->velocity / 1000.0f, motor_mit_msg->kp / 1000.0f, motor_mit_msg->kd / 1000.0f, motor_mit_msg->torque / 1000.0f);
        break;
    case CMD_MOTOR_POSITION:
        motor_position_control(((cmd_motor_position_t*)p_data)->id, ((cmd_motor_position_t*)p_data)->position / 1000.0f);
        break;
    default:
        break;
    }
}

int32_t unpack_bytes(uint8_t *buf, uint32_t len)
{
    for(int i = 0; i < len; ++i)
    {
        // 解析USB字节流
        if(protocol_unpack_byte(&unpack_stream_object, buf[i]))
        {
            // 成功解析数据帧
            dispatch_frame(unpack_stream_object.cmd_id, unpack_stream_object.data, unpack_stream_object.data_len);
        }
    }
    return len;
}

int32_t motor_feedback(void *argc)
{
    struct device *object;
    list_t *node = NULL;
    struct device_information *information;
    
    cmd_motor_feedback_t motor_feedback_msg;
    uint8_t* msg_buf = (uint8_t*)alloca(protocol_calculate_frame_size(sizeof(cmd_motor_feedback_t)));

    var_cpu_sr();
    enter_critical();

    information = get_device_information();
    for (node = information->object_list.next;
            node != &(information->object_list);
            node = node->next)
    {
        object = list_entry(node, struct device, list);

        if (object->type == DEVICE_MOTOR)
        {
            motor_feedback_msg.timestamp = get_timestamp();
            motor_feedback_msg.id = ((motor_device_t)object)->id;
            motor_feedback_msg.position = (uint16_t)(((motor_device_t)object)->data.position * 1000.0f);
            motor_feedback_msg.velocity = (uint16_t)(((motor_device_t)object)->data.velocity * 1000.0f);
            motor_feedback_msg.torque = (uint16_t)(((motor_device_t)object)->data.torque * 1000.0f);
            size_t frame_size = protocol_pack_data_to_buffer(CMD_MOTOR_FEEDBACK, (uint8_t*)&motor_feedback_msg, sizeof(cmd_motor_feedback_t), msg_buf);
            usb_interface_send(msg_buf, frame_size);
        }
    }
        
    /* leave critical */
    exit_critical();

    /* not found */
    return NULL;
}

int32_t gyro_feedback(void *argc)
{
    struct device *object;
    list_t *node = NULL;
    struct device_information *information;
    
    cmd_gyro_feedback_t gyro_feedback_msg;
    uint8_t* msg_buf = (uint8_t*)alloca(protocol_calculate_frame_size(sizeof(cmd_gyro_feedback_t)));

    var_cpu_sr();
    enter_critical();

    information = get_device_information();
    for (node = information->object_list.next;
            node != &(information->object_list);
            node = node->next)
    {
        object = list_entry(node, struct device, list);

        if (object->type == DEVICE_GYRO)
        {
            gyro_feedback_msg.timestamp = get_timestamp();
            gyro_feedback_msg.roll = (uint16_t)(((struct gyro_device*)object)->roll * 1000.0f);
            gyro_feedback_msg.pitch = (uint16_t)(((struct gyro_device*)object)->pitch * 1000.0f);
            gyro_feedback_msg.yaw = (uint16_t)(((struct gyro_device*)object)->yaw * 1000.0f);
            size_t frame_size = protocol_pack_data_to_buffer(CMD_GYRO_FEEDBACK, (uint8_t*)&gyro_feedback_msg, sizeof(cmd_gyro_feedback_t), msg_buf);
            usb_interface_send(msg_buf, frame_size);
        }
    }
        
    /* leave critical */
    exit_critical();

    /* not found */
    return NULL;
}

void communication_task(void const* arg)
{
    protocol_static_create_unpack_stream(&unpack_stream_object, heap_malloc(PROTOCOL_MAX_FRAME_LENGTH), PROTOCOL_MAX_FRAME_LENGTH);
    usb_vcp_rx_callback_register(unpack_bytes);

    soft_timer_register(motor_feedback, NULL, 1);
    soft_timer_register(gyro_feedback, NULL, 10);

    for(;;)
    {
        osDelay(10);
    }
}

void communication_task_init(void)
{
    osThreadDef(COMMUNICATION_TASK, communication_task, osPriorityNormal, 0, 512);
    communication_task_t = osThreadCreate(osThread(COMMUNICATION_TASK), NULL);
}
