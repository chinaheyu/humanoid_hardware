#ifndef __MOTOR_ZEROERR_H__
#define __MOTOR_ZEROERR_H__

#include "motor_base.h"
#include "drv_can.h"

enum zeroerr_mode_type
{
    ZEROERR_MODE_ABSOLUTE_POSITION = 0,
    ZEROERR_MODE_RELATIVE_POSITION,
    ZEROERR_MODE_CONTINUOUS_POSITION,
    ZEROERR_MODE_VELOCITY,
    ZEROERR_MODE_FORCE
};

enum zeroerr_return_type_e
{
    ZEROERR_RETURN_WRITE_FINISNED,
    ZEROERR_RETURN_READ_STATE,
    ZEROERR_RETURN_READ_POSITION,
    ZEROERR_RETURN_READ_VELOCITY,
    ZEROERR_RETURN_READ_TORQUE,
};

struct zeroerr_motor_data
{
    enum zeroerr_return_type_e return_type;
    int success;
    int state;      // 0：停止运动 1：运动中 3：重复停止中
    int position_int;
    int velocity_int;
    int torque_int;
};

struct zeroerr_motor_device
{
    struct motor_device parent;     // inherit motor_device
    uint16_t can_id;                // can id
    uint32_t encoder_count;         // 编码器单圈值
    enum zeroerr_mode_type mode;    // 工作模式
    struct zeroerr_motor_data data; // 反馈数据
    
    uint8_t initialized;            // 初始化
    float position_offset;
    float target_position;          // 目标位置
};

typedef struct zeroerr_motor_device* zeroerr_motor_device_t;

// 初始化电机驱动
void zeroerr_init(can_manage_obj_t can);

// 初始化电机对象
void zeroerr_motor_init(zeroerr_motor_device_t zeroerr_motor, uint16_t can_id, uint32_t encoder_count);

// 设置电机工作模式
void zeroerr_motor_set_mode(zeroerr_motor_device_t zeroerr_motor, enum zeroerr_mode_type mode);

// 使能电机
void zeroerr_motor_enable(zeroerr_motor_device_t zeroerr_motor);

// 失能电机
void zeroerr_motor_disable(zeroerr_motor_device_t zeroerr_motor);

// 开始运动
void zeroerr_motor_move(zeroerr_motor_device_t zeroerr_motor);

// 停止运动
void zeroerr_motor_stop(zeroerr_motor_device_t zeroerr_motor);

// 设置电机加速度（rad/s^2）
void zeroerr_motor_acceleration(zeroerr_motor_device_t zeroerr_motor, float acceleration);

// 设置电机减速度（rad/s^2）
void zeroerr_motor_deceleration(zeroerr_motor_device_t zeroerr_motor, float deceleration);

// 设置电机速度（rad/s）
void zeroerr_motor_velocity(zeroerr_motor_device_t zeroerr_motor, float velocity);

// 设置电机相对位置（rad）
void zeroerr_motor_relative_position(zeroerr_motor_device_t zeroerr_motor, float relative_position);

// 设置电机绝对位置（rad）
void zeroerr_motor_absolute_position(zeroerr_motor_device_t zeroerr_motor, float absolute_position);

// 设置电机模拟量（速度模式rad/s，力矩模式mA）
void zeroerr_motor_analog_value(zeroerr_motor_device_t zeroerr_motor, float analog_value);

void zeroerr_motor_read_position(zeroerr_motor_device_t zeroerr_motor);

void zeroerr_motor_read_velocity(zeroerr_motor_device_t zeroerr_motor);

void zeroerr_motor_read_torque(zeroerr_motor_device_t zeroerr_motor);

void zeroerr_motor_read_state(zeroerr_motor_device_t zeroerr_motor);

// 查找电机对象
zeroerr_motor_device_t zeroerr_motor_find_by_canid(uint16_t can_id);

#endif // __MOTOR_ZEROERR_H__
