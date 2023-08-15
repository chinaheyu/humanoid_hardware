#include "motor.h"
#include "communication_task.h"

void motor_mit_control(uint8_t id, float position, float velocity, float kp, float kd, float torque)
{
    struct device *object;
    list_t *node = NULL;
    struct device_information *information;

    information = get_device_information();
    for (node = information->object_list.next;
            node != &(information->object_list);
            node = node->next)
    {
        object = list_entry(node, struct device, list);

        if (object->type == DEVICE_MOTOR)
        {
            if (((motor_device_t)object)->id == id)
            {
                if (((motor_device_t)object)->type == MOTOR_DM)
                {
                    dm_motor_mit_control((dm_motor_device_t)object, position, velocity, kp, kd, torque);
                }
                if (((motor_device_t)object)->type == MOTOR_TMOTOR)
                {
                    tmotor_motor_mit_control((tmotor_motor_device_t)object, position, velocity, kp, kd, torque);
                }
                break;
            }
        }
    }
}

void motor_position_control(uint8_t id, float position)
{
    struct device *object;
    list_t *node = NULL;
    struct device_information *information;

    information = get_device_information();
    for (node = information->object_list.next;
            node != &(information->object_list);
            node = node->next)
    {
        object = list_entry(node, struct device, list);

        if (object->type == DEVICE_MOTOR)
        {
            if (((motor_device_t)object)->id == id)
            {
                if (((motor_device_t)object)->type == MOTOR_TMOTOR)
                {
                    tmotor_motor_position_control((tmotor_motor_device_t)object, position);
                }
                if (((motor_device_t)object)->type == MOTOR_ZEROERR)
                {
                    if (((zeroerr_motor_device_t)object)->initialized)
                    {
                        ((zeroerr_motor_device_t)object)->target_position = position;
                    }
                }
                break;
            }
        }
    }
}

int32_t motor_feedback(void *argc)
{
    struct device *object;
    list_t *node = NULL;
    struct device_information *information;
    
    cmd_motor_feedback_t motor_feedback_msg;
    uint8_t* msg_buf = (uint8_t*)alloca(protocol_calculate_frame_size(sizeof(cmd_motor_feedback_t)));

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
            motor_feedback_msg.position = (int16_t)(((motor_device_t)object)->data.position * 1000.0f);
            motor_feedback_msg.velocity = (int16_t)(((motor_device_t)object)->data.velocity * 1000.0f);
            motor_feedback_msg.torque = (int16_t)(((motor_device_t)object)->data.torque * 1000.0f);
            size_t frame_size = protocol_pack_data_to_buffer(CMD_MOTOR_FEEDBACK, (uint8_t*)&motor_feedback_msg, sizeof(cmd_motor_feedback_t), msg_buf);
            usb_interface_send(msg_buf, frame_size);
        }
    }
    /* not found */
    return NULL;
}
