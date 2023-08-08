#include "motor_tmotor.h"

#define LOG_TAG "drv.motor"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

#define M_PI (3.14159265358979323846)
#define DEG2RAD(x) (M_PI / 180.0 * (x))
#define RAD2DEG(x) (180.0 / M_PI * (x))

static can_manage_obj_t tmotor_can;
static uint16_t tmotor_master_id;
int32_t tmotor_feedback_callback(CAN_RxHeaderTypeDef *header, uint8_t *data);

void tmotor_init(can_manage_obj_t can, uint16_t master_id)
{
    tmotor_can = can;
    tmotor_master_id = master_id;
    can_fifo0_rx_callback_register(tmotor_can, tmotor_feedback_callback);
}

void tmotor_motor_init( tmotor_motor_device_t tmotor_motor,
                        uint16_t can_id,
                        enum tmotor_mode_type mode,
                        float kt,
                        float position_min,
                        float position_max,
                        float velocity_min,
                        float velocity_max,
                        float torque_min,
                        float torque_max,
                        float kp_min,
                        float kp_max,
                        float kd_min,
                        float kd_max
)
{
    tmotor_motor->can_id = can_id;
    tmotor_motor->mode = mode;
    tmotor_motor->kt = kt;
    tmotor_motor->position_min = position_min;
    tmotor_motor->position_max = position_max;
    tmotor_motor->velocity_min = velocity_min;
    tmotor_motor->velocity_max = velocity_max;
    tmotor_motor->torque_min = torque_min;
    tmotor_motor->torque_max = torque_max;
    tmotor_motor->kp_min = kp_min;
    tmotor_motor->kp_max = kp_max;
    tmotor_motor->kd_min = kd_min;
    tmotor_motor->kd_max = kd_max;
    tmotor_motor->parent.type = MOTOR_TMOTOR;
}

void tmotor_motor_enable(tmotor_motor_device_t tmotor_motor)
{
    uint8_t data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
    can_std_transmit(tmotor_can, tmotor_motor->can_id, data, 8);
}

void tmotor_motor_disable(tmotor_motor_device_t tmotor_motor)
{
    uint8_t data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
    can_std_transmit(tmotor_can, tmotor_motor->can_id, data, 8);
}

void tmotor_motor_set_zero_position(tmotor_motor_device_t tmotor_motor)
{
    uint8_t data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
    can_std_transmit(tmotor_can, tmotor_motor->can_id, data, 8);
}

static void tmotor_update_motor_data_servo(tmotor_motor_device_t target, uint8_t* data)
{
    target->data.position_int = (data[0] << 8) | data[1];
    target->data.speed_int = (data[2] << 8) | data[3];
    target->data.current_int = (data[4] << 8) | data[5];
    target->parent.data.position = DEG2RAD(target->data.position_int * 0.1f);  //电机位置deg->rad
    target->parent.data.velocity = (2.0 * M_PI / 60.0) * (target->data.speed_int * 10.0f); //电机速度rpm->rad/s
    target->parent.data.torque = target->data.current_int * 0.01f * target->kt;     //电机扭矩A->Nm
    target->data.temperature = data[6];       //电机温度
    target->data.error = data[7] ;              //电机故障码
}

static void tmotor_update_motor_data_mit(tmotor_motor_device_t target, uint8_t* data)
{
    target->data.id = data[0];
    target->data.position_int = data[1] << 8 | data[2];
    target->data.speed_int = (data[3] << 4) | (data[4] >> 4);
    target->data.current_int = ((data[4] & 0x0f) << 8) | data[5];
    target->parent.data.position = uint_to_float(target->data.position_int, target->position_min, target->position_max, 16);
    target->parent.data.velocity = uint_to_float(target->data.speed_int, target->velocity_min, target->velocity_max, 12);
    target->parent.data.torque = uint_to_float(target->data.current_int, target->torque_min, target->torque_max, 12);
    target->data.temperature = data[6];
    target->data.error = data[7];
}

int32_t tmotor_feedback_callback(CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    tmotor_motor_device_t motor_dev;
    if (header->IDE == CAN_ID_EXT)
    {
        // 伺服模式
        motor_dev = tmotor_motor_find_by_canid(header->ExtId & 0xff);
        if (motor_dev != NULL)
        {
            tmotor_update_motor_data_servo(motor_dev, data);
        }
    }
    else
    {
        // 运动模式 (MIT模式)
        if (header->StdId == tmotor_master_id)
        {
            motor_dev = tmotor_motor_find_by_canid(data[0]);
            if (motor_dev != NULL)
            {
                tmotor_update_motor_data_mit(motor_dev, data);
            }
        }
    }
    return 0;
}

tmotor_motor_device_t tmotor_motor_find_by_canid(uint16_t can_id)
{
    struct device *object;
    list_t *node = NULL;
    struct device_information *information;

    /* try to find device object */
    information = get_device_information();

    for (node = information->object_list.next;
            node != &(information->object_list);
            node = node->next)
    {
        object = list_entry(node, struct device, list);

        if (object->type == DEVICE_MOTOR)
        {
            if (((motor_device_t)object)->type == MOTOR_TMOTOR)
            {
                if (((tmotor_motor_device_t)object)->can_id == can_id)
                {
                    return (tmotor_motor_device_t)object;
                }
            }
        }
    }

    /* not found */
    return NULL;
}

void tmotor_motor_mit_control(tmotor_motor_device_t tmotor_motor, float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    pos_tmp = float_to_uint(_pos, tmotor_motor->position_min, tmotor_motor->position_max, 16);
    vel_tmp = float_to_uint(_vel, tmotor_motor->velocity_min, tmotor_motor->velocity_max, 12);
    kp_tmp  = float_to_uint(_KP, tmotor_motor->kp_min, tmotor_motor->kp_max, 12);
    kd_tmp  = float_to_uint(_KD, tmotor_motor->kd_min, tmotor_motor->kd_max, 12);
    tor_tmp = float_to_uint(_torq, tmotor_motor->torque_min, tmotor_motor->torque_max, 12);
    
    uint8_t data[8];
    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp & 0xff;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    data[4] = kp_tmp & 0xff;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    data[7] = tor_tmp & 0xff;
    
    can_std_transmit(tmotor_can, tmotor_motor->can_id, data, 8);
}

void tmotor_motor_duty_control(tmotor_motor_device_t tmotor_motor, float duty)
{
    uint8_t data[4];
    *(int32_t*)data = (int32_t)(duty * 100000.0f);
    can_ext_transmit(tmotor_can, tmotor_motor->can_id, data, 4);
}

void tmotor_motor_current_control(tmotor_motor_device_t tmotor_motor, float current)
{
    uint8_t data[4];
    *(int32_t*)data = (int32_t)(current * 1000.0f);
    can_ext_transmit(tmotor_can, tmotor_motor->can_id + (1 << 8), data, 4);
}

void tmotor_motor_current_brake_control(tmotor_motor_device_t tmotor_motor, float current)
{
    uint8_t data[4];
    *(int32_t*)data = (int32_t)(current * 1000.0f);
    can_ext_transmit(tmotor_can, tmotor_motor->can_id + (2 << 8), data, 4);
}

void tmotor_motor_rpm_control(tmotor_motor_device_t tmotor_motor, float rpm)
{
    uint8_t data[4];
    *(int32_t*)data = (int32_t)rpm;
    can_ext_transmit(tmotor_can, tmotor_motor->can_id + (3 << 8), data, 4);
}

void tmotor_motor_position_control(tmotor_motor_device_t tmotor_motor, float position)
{
    uint8_t data[4];
    *(int32_t*)data = (int32_t)(position * 10000.0f);
    can_ext_transmit(tmotor_can, tmotor_motor->can_id + (4 << 8), data, 4);
}

void tmotor_motor_set_origin_here(tmotor_motor_device_t tmotor_motor, uint8_t opt)
{
    can_ext_transmit(tmotor_can, tmotor_motor->can_id + (5 << 8), &opt, 1);
}

void tmotor_motor_position_velocity_control(tmotor_motor_device_t tmotor_motor, float position, int16_t velocity, int16_t acceleration)
{
    uint8_t data[8];
    *(int32_t*)data = (int32_t)(position * 10000.0f);
    *(int16_t*)(data + 4) = velocity;
    *(int16_t*)(data + 6) = acceleration;
    can_ext_transmit(tmotor_can, tmotor_motor->can_id + (6 << 8), data, 8);
}
