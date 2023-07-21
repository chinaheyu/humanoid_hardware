#ifndef __MOTOR_TMOTOR_H__
#define __MOTOR_TMOTOR_H__

#include "motor_base.h"
#include "drv_can.h"


enum tmotor_mode_type {
    TMOTOR_MODE_SERVO,              // 伺服模式
    TMOTOR_MODE_MIT                 // MIT模式
};

struct tmotor_motor_data
{
    uint8_t id;
    uint8_t error;
    int16_t position_int;
    int16_t speed_int;
    int16_t current_int;
    float temperature;          // Celsius degree
};

struct tmotor_motor_device
{
    struct motor_device parent;     // inherit motor_device
    uint16_t can_id;                // can id
    float kt;                       // 扭矩常数Nm/A
    
    /* 数值范围，用于浮点数与整数转化 */
    float position_min;
    float position_max;
    float velocity_min;
    float velocity_max;
    float torque_min;
    float torque_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    
    enum tmotor_mode_type mode;     // 电机工作模式
    struct tmotor_motor_data data;  // 电机反馈数据
};

typedef struct tmotor_motor_device* tmotor_motor_device_t;

// 初始化tmotor电机驱动
void tmotor_init(can_manage_obj_t can, uint16_t master_id);

// 初始化tmotor电机对象
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
);

// MIT模式
void tmotor_motor_enable(tmotor_motor_device_t tmotor_motor);
void tmotor_motor_disable(tmotor_motor_device_t tmotor_motor);
void tmotor_motor_set_zero_position(tmotor_motor_device_t tmotor_motor);
void tmotor_motor_mit_control(tmotor_motor_device_t tmotor_motor, float _pos, float _vel, float _KP, float _KD, float _torq);

// 伺服模式
void tmotor_motor_duty_control(tmotor_motor_device_t tmotor_motor, float duty);                 // percentage
void tmotor_motor_current_control(tmotor_motor_device_t tmotor_motor, float current);           // A
void tmotor_motor_current_brake_control(tmotor_motor_device_t tmotor_motor, float current);     // A
void tmotor_motor_rpm_control(tmotor_motor_device_t tmotor_motor, float rpm);                   // rpm
void tmotor_motor_position_control(tmotor_motor_device_t tmotor_motor, float position);         // degree
void tmotor_motor_set_origin_here(tmotor_motor_device_t tmotor_motor, uint8_t opt);             // option
void tmotor_motor_position_velocity_control(tmotor_motor_device_t tmotor_motor, float position, int16_t velocity, int16_t acceleration);    // degree

// 查找tmotor电机
tmotor_motor_device_t tmotor_motor_find_by_canid(uint16_t can_id);

#endif // __MOTOR_TMOTOR_H__
