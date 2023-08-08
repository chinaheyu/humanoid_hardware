#include "motor_dm.h"

#define LOG_TAG "drv.motor"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

#define P_MIN (-12.5f)
#define P_MAX (12.5f)
#define V_MIN (-30.0f)
#define V_MAX (30.0f)
#define KP_MIN (0.0f)
#define KP_MAX (500.0f)
#define KD_MIN (0.0f)
#define KD_MAX (5.0f)
#define T_MIN (-10.0f)
#define T_MAX (10.0f)

static can_manage_obj_t dm_can;
static uint16_t dm_master_id;

static void dm_update_motor_data(dm_motor_device_t target, uint8_t* data);
int32_t dm_feedback_callback(CAN_RxHeaderTypeDef *header, uint8_t *data);

void dm_init(can_manage_obj_t can, uint16_t master_id)
{
    dm_can = can;
    dm_master_id = master_id;
    can_fifo0_rx_callback_register(dm_can, dm_feedback_callback);
}

void dm_motor_init(dm_motor_device_t dm_motor, uint16_t can_id, enum dm_mode_type mode)
{
    dm_motor->can_id = can_id;
    dm_motor->mode = mode;
    dm_motor->parent.type = MOTOR_DM;
}

void dm_motor_enable(dm_motor_device_t dm_motor)
{
    uint8_t data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
    can_std_transmit(dm_can, dm_motor->can_id + 0x100 * dm_motor->mode, data, 8);
}

void dm_motor_disable(dm_motor_device_t dm_motor)
{
    uint8_t data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
    can_std_transmit(dm_can, dm_motor->can_id + 0x100 * dm_motor->mode, data, 8);
}

void dm_motor_set_zero_position(dm_motor_device_t dm_motor)
{
    uint8_t data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
    can_std_transmit(dm_can, dm_motor->can_id + 0x100 * dm_motor->mode, data, 8);
}

void dm_motor_mit_control(dm_motor_device_t dm_motor, float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
    
    uint8_t data[8];
    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    data[7] = tor_tmp;
    
    can_std_transmit(dm_can, dm_motor->can_id, data, 8);
}

void dm_motor_position_velocity_control(dm_motor_device_t dm_motor, float _pos, float _vel)
{
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;
    
    uint8_t data[8];
    data[0] = *pbuf;
    data[1] = *(pbuf+1);
    data[2] = *(pbuf+2);
    data[3] = *(pbuf+3);
    data[4] = *vbuf;
    data[5] = *(vbuf+1);
    data[6] = *(vbuf+2);
    data[7] = *(vbuf+3);
    
    can_std_transmit(dm_can, dm_motor->can_id + 0x100, data, 8);
}

void dm_motor_velocity_control(dm_motor_device_t dm_motor, float _vel)
{
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;
    
    uint8_t data[4];
    data[0] = *vbuf;
    data[1] = *(vbuf+1);
    data[2] = *(vbuf+2);
    data[3] = *(vbuf+3);
    
    can_std_transmit(dm_can, dm_motor->can_id + 0x200, data, 4);
}

int32_t dm_feedback_callback(CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    dm_motor_device_t motor_dev;
    if (header->StdId == dm_master_id)
    {
        motor_dev = dm_motor_find_by_canid(data[0] & 0x0f);
        if (motor_dev != NULL)
        {
            dm_update_motor_data(motor_dev, data);
        }
    }
    return 0;
}

static void dm_update_motor_data(dm_motor_device_t target, uint8_t* data)
{
    target->data.id = data[0] & 0x0f;
    target->data.error = data[0] >> 4;
    target->data.position_int = data[1] << 8 | data[2];
    target->data.velocity_int = (data[3] << 4) | (data[4] >> 4);
    target->data.torque_int = ((data[4] & 0x0f) << 8) | data[5];
    target->parent.data.position = uint_to_float(target->data.position_int, P_MIN, P_MAX, 16);
    target->parent.data.velocity = uint_to_float(target->data.velocity_int, V_MIN, V_MAX, 12);
    target->parent.data.torque = uint_to_float(target->data.torque_int, T_MIN, T_MAX, 12);
    target->data.mos_temperature = data[6];
    target->data.rotor_temperature = data[7];
}

dm_motor_device_t dm_motor_find_by_canid(uint16_t can_id)
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
            if (((motor_device_t)object)->type == MOTOR_DM)
            {
                if (((dm_motor_device_t)object)->can_id == can_id)
                {
                    return (dm_motor_device_t)object;
                }
            }
        }
    }

    /* not found */
    return NULL;
}
