#ifndef __MOTOR_DM_H__
#define __MOTOR_DM_H__

#include "motor_base.h"
#include "drv_can.h"

enum dm_mode_type
{
    DM_MODE_MIT = 0,
    DM_MODE_POSITION_VELOCITY,
    DM_MODE_VELOCITY
};

struct dm_motor_data
{
    uint8_t id;
    uint8_t error;
    int position_int;
    int velocity_int;
    int torque_int;
    float mos_temperature;          // Celsius degree
    float rotor_temperature;        // Celsius degree
};

struct dm_motor_device
{
    struct motor_device parent;     // inherit motor_device
    uint16_t can_id;
    enum dm_mode_type mode;
    struct dm_motor_data data;
};

typedef struct dm_motor_device* dm_motor_device_t;

// 初始化达妙电机驱动
void dm_init(can_manage_obj_t can, uint16_t master_id);

// 初始化达妙电机对象
void dm_motor_init(dm_motor_device_t dm_motor, uint16_t can_id, enum dm_mode_type mode);

// 查找达妙电机对象
dm_motor_device_t dm_motor_find_by_canid(uint16_t can_id);

// MIT模式
void dm_motor_enable(dm_motor_device_t dm_motor);
void dm_motor_disable(dm_motor_device_t dm_motor);
void dm_motor_set_zero_position(dm_motor_device_t dm_motor);
void dm_motor_mit_control(dm_motor_device_t dm_motor, float _pos, float _vel, float _KP, float _KD, float _torq);

// 伺服模式
void dm_motor_position_velocity_control(dm_motor_device_t dm_motor, float _pos, float _vel);
void dm_motor_velocity_control(dm_motor_device_t dm_motor, float _vel);

#endif // __MOTOR_DM_H__
