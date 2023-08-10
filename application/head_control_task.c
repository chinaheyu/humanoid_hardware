#include "head_control_task.h"
#include "communication_task.h"
#include <cmsis_os.h>
#include "app_manage.h"
#include "maestro.h"
#include "motor.h"
#include "tim.h"
#include "os_timer.h"

#define LOG_TAG "head_control"
#define LOG_OUTPUT_LEVEL 4
#include "log.h"

#define M_PI (3.14159265358979323846)

static struct dm_motor_device yaw_motor;
static float pitch_motor_speed;


void pitch_motor_set_speed(float speed)
{
    int ccr = speed * (float)TIM1->ARR;
    if (ccr > TIM1->ARR)
        ccr = TIM1->ARR;
    if (ccr < 0)
        ccr = 0;
    TIM1->CCR1 = ccr;
}

void pitch_motor_stop()
{
    HAL_GPIO_WritePin(IO6_GPIO_Port, IO6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IO8_GPIO_Port, IO8_Pin, GPIO_PIN_RESET);
}

void pitch_motor_forward()
{
    HAL_GPIO_WritePin(IO6_GPIO_Port, IO6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IO8_GPIO_Port, IO8_Pin, GPIO_PIN_SET);
}

void pitch_motor_backward()
{
    HAL_GPIO_WritePin(IO6_GPIO_Port, IO6_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IO8_GPIO_Port, IO8_Pin, GPIO_PIN_RESET);
}

PROTOCOL_CALLBACK_FUNCTION(cmd_head_servo_callback)
{
    cmd_head_servo_t* msg = (cmd_head_servo_t*)p_data;
    uint16_t face_motor_targets[11];
    for (int i = 0; i < 11; ++i)
    {
        uint16_t target = msg->pulse_width[i] * 4;
        if (target < 500 * 4)
            target = 500 * 4;
        if (target > 2500 * 4)
            target = 2500 * 4;
        face_motor_targets[i] = target;
    }
    MAESTRO_SetMultipleTargets(11, 0, face_motor_targets);
}

PROTOCOL_CALLBACK_FUNCTION(cmd_neck_motor_callback)
{
    cmd_neck_motor_t* msg = (cmd_neck_motor_t*)p_data;
    dm_motor_position_velocity_control(&yaw_motor, msg->yaw_angle / 1000.0f, msg->yaw_max_velocity / 1000.0f);
    pitch_motor_speed = msg->pitch_velocity / 1000.0f;
    if (msg->pitch_velocity == 0)
    {
        pitch_motor_stop();
    }
    else if (msg->pitch_velocity > 0)
    {
        pitch_motor_forward();
    }
    else
    {
        pitch_motor_backward();
    }
    pitch_motor_set_speed(fabs(pitch_motor_speed));
}

void reset_servo_position(void)
{
    uint16_t face_motor_targets[11];
    face_motor_targets[0] = 1000 * 4;
    for (int i = 1; i < 11; ++i)
    {
        face_motor_targets[i] = 1500 * 4;
    }
    MAESTRO_SetMultipleTargets(11, 0, face_motor_targets);
}

int32_t head_state_feedback(void *argc)
{
    cmd_head_feedback_t msg;
    msg.timestamp = get_timestamp();
    for (int i = 0; i < 11; ++i)
    {
        msg.pulse_width[i] = MAESTRO_GetTarget(i) / 4;
    }
    msg.pitch_velocity = pitch_motor_speed * 1000;
    msg.yaw_angle = yaw_motor.parent.data.position * 1000;
    
    size_t frame_size = protocol_calculate_frame_size(sizeof(cmd_head_feedback_t));
    uint8_t* buffer = (uint8_t*)alloca(frame_size);
    protocol_pack_data_to_buffer(CMD_HEAD_FEEDBACK, (uint8_t*)&msg, sizeof(cmd_head_feedback_t), buffer);
    usb_interface_send(buffer, frame_size);
}

int32_t check_pitch_limit(void *argc)
{
    int pitch_forward_limit_state = HAL_GPIO_ReadPin(PITCH_LIMIT1_GPIO_Port, PITCH_LIMIT1_Pin) == GPIO_PIN_SET;
    int pitch_backward_limit_state = HAL_GPIO_ReadPin(PITCH_LIMIT2_GPIO_Port, PITCH_LIMIT2_Pin) == GPIO_PIN_SET;
    if (pitch_forward_limit_state || pitch_backward_limit_state)
    {
        beep_set_times(3);
        if (pitch_motor_speed > 0)
        {
            if (pitch_forward_limit_state)
            {
                pitch_motor_stop();
                pitch_motor_set_speed(0);
                pitch_motor_speed = 0;
            }
        }
        if (pitch_motor_speed < 0)
        {
            if (pitch_backward_limit_state)
            {
                pitch_motor_stop();
                pitch_motor_set_speed(0);
                pitch_motor_speed = 0;
            }
        }
    }
    else
    {
        beep_set_times(0);
    }
    return 0;
}

void head_control_task_init(void)
{
    log_i("Head control task start.");
    
    struct app_manage* app = get_current_app();
    app->user_key_callback = reset_servo_position;
    
    reset_servo_position();
    pitch_motor_stop();
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_1);
    
    dm_init(&can1_manage, 0);
    dm_motor_init(&yaw_motor, 1, DM_MODE_POSITION_VELOCITY);
    motor_register(&yaw_motor.parent, "MOTOR_NECK_YAW");
    dm_motor_enable(&yaw_motor);
    
    register_cmd_callback(CMD_HEAD_SERVO, cmd_head_servo_callback);
    register_cmd_callback(CMD_NECK_MOTOR, cmd_neck_motor_callback);
    
    soft_timer_register(head_state_feedback, NULL, 10);
    soft_timer_register(check_pitch_limit, NULL, 1);
}
