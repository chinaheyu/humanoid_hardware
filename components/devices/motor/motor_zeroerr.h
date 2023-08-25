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
    int state;      // 0��ֹͣ�˶� 1���˶��� 3���ظ�ֹͣ��
    int position_int;
    int velocity_int;
    int torque_int;
};

struct zeroerr_motor_device
{
    struct motor_device parent;     // inherit motor_device
    uint16_t can_id;                // can id
    uint32_t encoder_count;         // ��������Ȧֵ
    enum zeroerr_mode_type mode;    // ����ģʽ
    struct zeroerr_motor_data data; // ��������
    
    uint8_t initialized;            // ��ʼ��
    float position_offset;
    float target_position;          // Ŀ��λ��
};

typedef struct zeroerr_motor_device* zeroerr_motor_device_t;

// ��ʼ���������
void zeroerr_init(can_manage_obj_t can);

// ��ʼ���������
void zeroerr_motor_init(zeroerr_motor_device_t zeroerr_motor, uint16_t can_id, uint32_t encoder_count);

// ���õ������ģʽ
void zeroerr_motor_set_mode(zeroerr_motor_device_t zeroerr_motor, enum zeroerr_mode_type mode);

// ʹ�ܵ��
void zeroerr_motor_enable(zeroerr_motor_device_t zeroerr_motor);

// ʧ�ܵ��
void zeroerr_motor_disable(zeroerr_motor_device_t zeroerr_motor);

// ��ʼ�˶�
void zeroerr_motor_move(zeroerr_motor_device_t zeroerr_motor);

// ֹͣ�˶�
void zeroerr_motor_stop(zeroerr_motor_device_t zeroerr_motor);

// ���õ�����ٶȣ�rad/s^2��
void zeroerr_motor_acceleration(zeroerr_motor_device_t zeroerr_motor, float acceleration);

// ���õ�����ٶȣ�rad/s^2��
void zeroerr_motor_deceleration(zeroerr_motor_device_t zeroerr_motor, float deceleration);

// ���õ���ٶȣ�rad/s��
void zeroerr_motor_velocity(zeroerr_motor_device_t zeroerr_motor, float velocity);

// ���õ�����λ�ã�rad��
void zeroerr_motor_relative_position(zeroerr_motor_device_t zeroerr_motor, float relative_position);

// ���õ������λ�ã�rad��
void zeroerr_motor_absolute_position(zeroerr_motor_device_t zeroerr_motor, float absolute_position);

// ���õ��ģ�������ٶ�ģʽrad/s������ģʽmA��
void zeroerr_motor_analog_value(zeroerr_motor_device_t zeroerr_motor, float analog_value);

void zeroerr_motor_read_position(zeroerr_motor_device_t zeroerr_motor);

void zeroerr_motor_read_velocity(zeroerr_motor_device_t zeroerr_motor);

void zeroerr_motor_read_torque(zeroerr_motor_device_t zeroerr_motor);

void zeroerr_motor_read_state(zeroerr_motor_device_t zeroerr_motor);

// ���ҵ������
zeroerr_motor_device_t zeroerr_motor_find_by_canid(uint16_t can_id);

#endif // __MOTOR_ZEROERR_H__
