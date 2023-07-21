#include "motor.h"

void motor_mit_control(uint8_t id, float position, float velocity, float kp, float kd, float torque)
{
    struct device *object;
    list_t *node = NULL;
    struct device_information *information;

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
        
    /* leave critical */
    exit_critical();
}

void motor_position_control(uint8_t id, float position)
{
    struct device *object;
    list_t *node = NULL;
    struct device_information *information;

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
            if (((motor_device_t)object)->id == id)
            {
                if (((motor_device_t)object)->type == MOTOR_TMOTOR)
                {
                    tmotor_motor_position_control((tmotor_motor_device_t)object, position);
                }
                if (((motor_device_t)object)->type == MOTOR_ZEROERR)
                {
                    zeroerr_motor_absolute_position((zeroerr_motor_device_t)object, position);
                }
                break;
            }
        }
    }
        
    /* leave critical */
    exit_critical();
}
