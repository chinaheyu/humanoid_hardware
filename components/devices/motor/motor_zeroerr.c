#include "motor_zeroerr.h"

#define LOG_TAG "drv.motor"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

#include <cmsis_os.h>

#define M_2_PI (6.283185307179586476925286766559f)

static can_manage_obj_t zeroerr_can;
static xTaskHandle zeroerr_task_handle;
static BaseType_t higher_priority_task_woken;

int32_t zeroerr_feedback_callback(CAN_RxHeaderTypeDef *header, uint8_t *data);
uint32_t zeroerr_can_std_transmit(zeroerr_motor_device_t zeroerr_motor, can_manage_obj_t m_obj, uint16_t std_id, uint8_t *data, uint16_t len);
int32_t bigEndianToLittleEndian(int32_t value);
int32_t littleEndianToBigEndian(int32_t value);

static inline int rad_to_encoder(zeroerr_motor_device_t zeroerr_motor, float rad)
{
    return (float)zeroerr_motor->encoder_count * rad / M_2_PI;
}

static inline float encoder_to_rad(zeroerr_motor_device_t zeroerr_motor, int enc)
{
    return M_2_PI * (float)enc / (float)zeroerr_motor->encoder_count;
}

// 初始化电机驱动
void zeroerr_init(can_manage_obj_t can)
{
    zeroerr_task_handle = xTaskGetCurrentTaskHandle();
    zeroerr_can = can;
    can_fifo0_rx_callback_register(zeroerr_can, zeroerr_feedback_callback);
}

// 初始化电机对象
void zeroerr_motor_init(zeroerr_motor_device_t zeroerr_motor, uint16_t can_id, uint32_t encoder_count)
{
    zeroerr_motor->can_id = can_id;
    zeroerr_motor->encoder_count = encoder_count;
    zeroerr_motor->parent.type = MOTOR_ZEROERR;
}

// 设置电机工作模式
void zeroerr_motor_set_mode(zeroerr_motor_device_t zeroerr_motor, enum zeroerr_mode_type mode)
{
    uint8_t data[6];
    
    zeroerr_motor->data.return_type = ZEROERR_RETURN_WRITE_FINISNED;
    
    if (mode == ZEROERR_MODE_ABSOLUTE_POSITION || mode == ZEROERR_MODE_RELATIVE_POSITION || mode == ZEROERR_MODE_CONTINUOUS_POSITION)
    {
        data[0] = 0x00;
        data[1] = 0x4E;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = 0x00;
        data[5] = 0x03;
        zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
        
        data[1] = 0x8D;
        if (mode == ZEROERR_MODE_CONTINUOUS_POSITION)
        {
            data[5] = 0x00;
        }
        else
        {
            data[5] = 0x01;
        }
        zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
    }
    
    if (mode == ZEROERR_MODE_VELOCITY)
    {
        data[0] = 0x00;
        data[1] = 0x4E;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = 0x00;
        data[5] = 0x02;
        zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
        
        data[0] = 0x01;
        data[1] = 0x12;
        data[5] = 0x00;
        zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
    
        data[1] = 0xFD;
        zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
    }
    
    if (mode == ZEROERR_MODE_FORCE)
    {
        data[0] = 0x00;
        data[1] = 0x4E;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = 0x00;
        data[5] = 0x01;
        zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
        
        data[0] = 0x01;
        data[1] = 0x12;
        data[5] = 0x00;
        zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
    
        data[1] = 0xFD;
        zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
    }
    
    zeroerr_motor->mode = mode;
}

// 使能电机
void zeroerr_motor_enable(zeroerr_motor_device_t zeroerr_motor)
{
    uint8_t data[6] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x01 };
    zeroerr_motor->data.return_type = ZEROERR_RETURN_WRITE_FINISNED;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
}

// 失能电机
void zeroerr_motor_disable(zeroerr_motor_device_t zeroerr_motor)
{
    uint8_t data[6] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
    zeroerr_motor->data.return_type = ZEROERR_RETURN_WRITE_FINISNED;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
}

// 开始运动
void zeroerr_motor_move(zeroerr_motor_device_t zeroerr_motor)
{
    uint8_t data[2] = { 0x00, 0x83 };
    zeroerr_motor->data.return_type = ZEROERR_RETURN_WRITE_FINISNED;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 2);
}

// 停止运动
void zeroerr_motor_stop(zeroerr_motor_device_t zeroerr_motor)
{
    uint8_t data[2] = { 0x00, 0x84 };
    zeroerr_motor->data.return_type = ZEROERR_RETURN_WRITE_FINISNED;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 2);
}

// 设置电机加速度（rad/s^2）
void zeroerr_motor_acceleration(zeroerr_motor_device_t zeroerr_motor, float acceleration)
{
    uint8_t data[6] = { 0x00, 0x88 };
    *(int *)(data + 2) = littleEndianToBigEndian(rad_to_encoder(zeroerr_motor, acceleration));
    zeroerr_motor->data.return_type = ZEROERR_RETURN_WRITE_FINISNED;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
}

// 设置电机减速度（rad/s^2）
void zeroerr_motor_deceleration(zeroerr_motor_device_t zeroerr_motor, float deceleration)
{
    uint8_t data[6] = { 0x00, 0x89 };
    *(int *)(data + 2) = littleEndianToBigEndian(rad_to_encoder(zeroerr_motor, deceleration));
    zeroerr_motor->data.return_type = ZEROERR_RETURN_WRITE_FINISNED;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
}

// 设置电机速度（rad/s）
void zeroerr_motor_velocity(zeroerr_motor_device_t zeroerr_motor, float velocity)
{
    uint8_t data[6] = { 0x00, 0x8A };
    *(int *)(data + 2) = littleEndianToBigEndian(rad_to_encoder(zeroerr_motor, velocity));
    zeroerr_motor->data.return_type = ZEROERR_RETURN_WRITE_FINISNED;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
}

// 设置电机相对位置（rad）
void zeroerr_motor_relative_position(zeroerr_motor_device_t zeroerr_motor, float relative_position)
{
    uint8_t data[6] = { 0x00, 0x87 };
    *(int *)(data + 2) = littleEndianToBigEndian(rad_to_encoder(zeroerr_motor, relative_position));
    zeroerr_motor->data.return_type = ZEROERR_RETURN_WRITE_FINISNED;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
}

// 设置电机绝对位置（rad）
void zeroerr_motor_absolute_position(zeroerr_motor_device_t zeroerr_motor, float absolute_position)
{
    uint8_t data[6] = { 0x00, 0x86 };
    *(int *)(data + 2) = littleEndianToBigEndian(rad_to_encoder(zeroerr_motor, absolute_position + zeroerr_motor->position_offset));
    zeroerr_motor->data.return_type = ZEROERR_RETURN_WRITE_FINISNED;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
}

// 设置电机模拟量（速度模式rad/s，力矩模式mA）
void zeroerr_motor_analog_value(zeroerr_motor_device_t zeroerr_motor, float analog_value)
{
    uint8_t data[6] = { 0x01, 0xFE };
    if (zeroerr_motor->mode == ZEROERR_MODE_VELOCITY)
        *(int *)(data + 2) = littleEndianToBigEndian(rad_to_encoder(zeroerr_motor, analog_value));
    if (zeroerr_motor->mode == ZEROERR_MODE_FORCE)
        *(int *)(data + 2) = littleEndianToBigEndian(analog_value);
    zeroerr_motor->data.return_type = ZEROERR_RETURN_WRITE_FINISNED;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 6);
}

void zeroerr_motor_read_position(zeroerr_motor_device_t zeroerr_motor)
{
    uint8_t data[2] = { 0x00, 0x02 };
    zeroerr_motor->data.return_type = ZEROERR_RETURN_READ_POSITION;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 2);
}

void zeroerr_motor_read_velocity(zeroerr_motor_device_t zeroerr_motor)
{
    uint8_t data[4] = { 0x00, 0x05, 0x00, 0x01 };
    zeroerr_motor->data.return_type = ZEROERR_RETURN_READ_VELOCITY;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 4);
}

void zeroerr_motor_read_torque(zeroerr_motor_device_t zeroerr_motor)
{
    uint8_t data[2] = { 0x02, 0x0D };
    zeroerr_motor->data.return_type = ZEROERR_RETURN_READ_TORQUE;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 2);
}

void zeroerr_motor_read_state(zeroerr_motor_device_t zeroerr_motor)
{
    uint8_t data[2] = { 0x00, 0x20 };
    zeroerr_motor->data.return_type = ZEROERR_RETURN_READ_STATE;
    zeroerr_can_std_transmit(zeroerr_motor, zeroerr_can, 0x640 + zeroerr_motor->can_id, data, 2);
}

// 查找电机对象
zeroerr_motor_device_t zeroerr_motor_find_by_canid(uint16_t can_id)
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
            if (((motor_device_t)object)->type == MOTOR_ZEROERR)
            {
                if (((zeroerr_motor_device_t)object)->can_id == can_id)
                {
                    return (zeroerr_motor_device_t)object;
                }
            }
        }
    }

    /* not found */
    return NULL;
}

int32_t zeroerr_feedback_callback(CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    zeroerr_motor_device_t motor_dev;
    motor_dev = zeroerr_motor_find_by_canid(header->StdId - 0x05C0);
    if (motor_dev != NULL)
    {
        switch (motor_dev->data.return_type)
        {
            case ZEROERR_RETURN_WRITE_FINISNED:
                if (header->DLC == 1 && data[0] == 0x3E)
                {
                    motor_dev->data.success = 1;
                }
                break;
            case ZEROERR_RETURN_READ_STATE:
                if (header->DLC == 5 && data[4] == 0x3E)
                {
                    motor_dev->data.state = bigEndianToLittleEndian(*(int*)data);
                    motor_dev->data.success = 1;
                }
                else
                {
                    motor_dev->data.success = 0;
                }
                break;
            case ZEROERR_RETURN_READ_POSITION:
                if (header->DLC == 5 && data[4] == 0x3E)
                {
                    motor_dev->data.position_int = bigEndianToLittleEndian(*(int*)data);
                    if (!motor_dev->initialized)
                    {
                        float position = encoder_to_rad(motor_dev, motor_dev->data.position_int);
                        float offset_position = fmodf(position, M_2_PI);
                        if (offset_position > M_2_PI / 2.0f)
                        {
                            offset_position -= M_2_PI;
                        }
                        if (offset_position < -M_2_PI / 2.0f)
                        {
                            offset_position += M_2_PI;
                        }
                        motor_dev->position_offset = position - offset_position;
                        motor_dev->parent.data.position = encoder_to_rad(motor_dev, motor_dev->data.position_int) - motor_dev->position_offset;
                        motor_dev->target_position = motor_dev->parent.data.position;
                    }
                    else
                    {
                        motor_dev->parent.data.position = encoder_to_rad(motor_dev, motor_dev->data.position_int) - motor_dev->position_offset;
                    }
                    motor_dev->data.success = 1;
                }
                else
                {
                    motor_dev->data.success = 0;
                }
                break;
            case ZEROERR_RETURN_READ_VELOCITY:
                if (header->DLC == 5 && data[4] == 0x3E)
                {
                    motor_dev->data.velocity_int = bigEndianToLittleEndian(*(int*)data);
                    motor_dev->parent.data.velocity = encoder_to_rad(motor_dev, motor_dev->data.velocity_int);
                    motor_dev->data.success = 1;
                }
                else
                {
                    motor_dev->data.success = 0;
                }
                break;
            case ZEROERR_RETURN_READ_TORQUE:
                if (header->DLC == 5 && data[4] == 0x3E)
                {
                    motor_dev->data.torque_int = bigEndianToLittleEndian(*(int*)data);
                    motor_dev->parent.data.torque = (float)motor_dev->data.torque_int / 1000.0f;
                    motor_dev->data.success = 1;
                }
                else
                {
                    motor_dev->data.success = 0;
                }
                break;
            default:
                motor_dev->data.success = 0;
                break;
        };
        
        higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(zeroerr_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
    return 0;
}

uint32_t zeroerr_can_std_transmit(zeroerr_motor_device_t zeroerr_motor, can_manage_obj_t m_obj, uint16_t std_id, uint8_t *data, uint16_t len)
{
    while (1)
    {
        zeroerr_motor->data.success = 0;
        can_std_transmit(m_obj, std_id, data, len);
        ulTaskNotifyTake(pdTRUE, 10);
        if (zeroerr_motor->data.success)
        {
            break;
        }
    }
    return 0;
}

int32_t littleEndianToBigEndian(int32_t value) {
    int32_t result = ((value & 0xFF) << 24) |
                     (((value >> 8) & 0xFF) << 16) |
                     (((value >> 16) & 0xFF) << 8) |
                     ((value >> 24) & 0xFF);
    return result;
}

int32_t bigEndianToLittleEndian(int32_t value) {
    int32_t result = ((value >> 24) & 0xFF) |
                     (((value >> 16) & 0xFF) << 8) |
                     (((value >> 8) & 0xFF) << 16) |
                     ((value & 0xFF) << 24);
    return result;
}
