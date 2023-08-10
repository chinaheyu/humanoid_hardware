#include "leg_control_task.h"
#include <cmsis_os.h>
#include "app_manage.h"
#include "communication_task.h"
#include "motor.h"
#include "os_timer.h"

#define LOG_TAG "leg_control"
#define LOG_OUTPUT_LEVEL 4
#include "log.h"

osThreadId leg_control_task_t;

PROTOCOL_CALLBACK_FUNCTION(motor_mit_control_callback)
{
    cmd_motor_mit_t* motor_mit_msg;
    motor_mit_msg = (cmd_motor_mit_t*)p_data;
    motor_mit_control(motor_mit_msg->id, motor_mit_msg->position / 1000.0f, motor_mit_msg->velocity / 1000.0f, motor_mit_msg->kp / 1000.0f, motor_mit_msg->kd / 1000.0f, motor_mit_msg->torque / 1000.0f);
}

PROTOCOL_CALLBACK_FUNCTION(motor_position_control_callback)
{
    motor_position_control(((cmd_motor_position_t*)p_data)->id, ((cmd_motor_position_t*)p_data)->position / 1000.0f);
}

void leg_control_task(void const* arg)
{
    log_i("Leg control task start.");
    
    struct app_manage *app = get_current_app();
    if (app->app_id == 2)
    {
        // LEFT_LEG_CONTROL
    }
    if (app->app_id == 3)
    {
        // RIGHT_LEG_CONTROL
    }
    if (app->app_id == 4)
    {
        // WAIST_CONTROL
    }
    
    register_cmd_callback(CMD_MOTOR_MIT, motor_mit_control_callback);
    register_cmd_callback(CMD_MOTOR_POSITION, motor_position_control_callback);
    
    // 1kHz motor feedback frequency.
    soft_timer_register(motor_feedback, NULL, 1);
    
    for(;;)
    {
        osDelay(10);
    }
}

void leg_control_task_init(void)
{
    osThreadDef(LEG_CONTROL_TASK, leg_control_task, osPriorityNormal, 0, 512);
    leg_control_task_t = osThreadCreate(osThread(LEG_CONTROL_TASK), NULL);
}
