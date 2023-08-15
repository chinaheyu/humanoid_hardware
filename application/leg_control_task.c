#include "leg_control_task.h"
#include <cmsis_os.h>
#include "app_manage.h"
#include "communication_task.h"
#include "motor.h"
#include "os_timer.h"

#define LOG_TAG "leg_control"
#define LOG_OUTPUT_LEVEL 4
#include "log.h"

// motor devices
struct tmotor_motor_device tmotor[3];
struct dm_motor_device dm[2];
struct zeroerr_motor_device zeroerr[3];

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
        dm_init(&can1_manage, 0);
        tmotor_init(&can2_manage, 0);
        
        // motor id 6
        dm[0].parent.id = 6;
        dm_motor_init(&dm[0], 6, DM_MODE_MIT);
        motor_register(&dm[0].parent, "MOTOR_6");
        dm_motor_enable(&dm[0]);
        
        // motor id 7
        dm[1].parent.id = 7;
        dm_motor_init(&dm[1], 7, DM_MODE_MIT);
        motor_register(&dm[1].parent, "MOTOR_7");
        dm_motor_enable(&dm[1]);
        
        // motor id 3
        tmotor[0].parent.id = 3;
        tmotor_motor_init(&tmotor[0], 3, TMOTOR_MODE_MIT, 0.105, -12.5, 12.5, -50.0, 50.0, -18.0, 18.0, 0, 500, 0, 5);
        motor_register(&tmotor[0].parent, "MOTOR_3");
        tmotor_motor_enable(&tmotor[0]);
        
        // motor id 4
        tmotor[1].parent.id = 4;
        tmotor_motor_init(&tmotor[1], 4, TMOTOR_MODE_MIT, 0.136, -12.5, 12.5, -8.0, 8.0, -144.0, 144.0, 0, 500, 0, 5);
        motor_register(&tmotor[1].parent, "MOTOR_4");
        tmotor_motor_enable(&tmotor[1]);
        
        // motor id 5
        tmotor[2].parent.id = 5;
        tmotor_motor_init(&tmotor[2], 5, TMOTOR_MODE_MIT, 0.136, -12.5, 12.5, -8.0, 8.0, -144.0, 144.0, 0, 500, 0, 5);
        motor_register(&tmotor[2].parent, "MOTOR_5");
        tmotor_motor_enable(&tmotor[2]);
        
    }
    if (app->app_id == 3)
    {
        // RIGHT_LEG_CONTROL
        dm_init(&can1_manage, 0);
        tmotor_init(&can2_manage, 0);
        
        // motor id 12
        dm[0].parent.id = 12;
        dm_motor_init(&dm[0], 12, DM_MODE_MIT);
        motor_register(&dm[0].parent, "MOTOR_12");
        dm_motor_enable(&dm[0]);
        
        // motor id 13
        dm[1].parent.id = 13;
        dm_motor_init(&dm[1], 13, DM_MODE_MIT);
        motor_register(&dm[1].parent, "MOTOR_13");
        dm_motor_enable(&dm[1]);
        
        // motor id 9
        tmotor[0].parent.id = 9;
        tmotor_motor_init(&tmotor[0], 9, TMOTOR_MODE_MIT, 0.105, -12.5, 12.5, -50.0, 50.0, -18.0, 18.0, 0, 500, 0, 5);
        motor_register(&tmotor[0].parent, "MOTOR_9");
        tmotor_motor_enable(&tmotor[0]);
        
        // motor id 10
        tmotor[1].parent.id = 10;
        tmotor_motor_init(&tmotor[1], 10, TMOTOR_MODE_MIT, 0.136, -12.5, 12.5, -8.0, 8.0, -144.0, 144.0, 0, 500, 0, 5);
        motor_register(&tmotor[1].parent, "MOTOR_10");
        tmotor_motor_enable(&tmotor[1]);
        
        // motor id 11
        tmotor[2].parent.id = 11;
        tmotor_motor_init(&tmotor[2], 11, TMOTOR_MODE_MIT, 0.136, -12.5, 12.5, -8.0, 8.0, -144.0, 144.0, 0, 500, 0, 5);
        motor_register(&tmotor[2].parent, "MOTOR_11");
        tmotor_motor_enable(&tmotor[2]);
    }
    if (app->app_id == 4)
    {
        // WAIST_CONTROL
        zeroerr_init(&can1_manage);
        
        float max_vel = 30 / 9.55;
        
        // motor id 1
        zeroerr[0].parent.id = 1;
        zeroerr[0].initialized = 0;
        zeroerr[2].moving = 0;
        motor_register(&zeroerr[0].parent, "MOTOR_1");
        zeroerr_motor_init(&zeroerr[0], 1, ZEROERR_MODE_ABSOLUTE_POSITION, 10, 1 << 19);
        zeroerr_motor_acceleration(&zeroerr[0], 3 * max_vel);
        zeroerr_motor_deceleration(&zeroerr[0], 3 * max_vel);
        zeroerr_motor_velocity(&zeroerr[0], max_vel);
        zeroerr_motor_enable(&zeroerr[0]);
        
        // motor id 2
        zeroerr[1].parent.id = 2;
        zeroerr[1].initialized = 0;
        zeroerr[2].moving = 0;
        motor_register(&zeroerr[1].parent, "MOTOR_2");
        zeroerr_motor_init(&zeroerr[1], 2, ZEROERR_MODE_ABSOLUTE_POSITION, 10, 1 << 19);
        zeroerr_motor_acceleration(&zeroerr[1], 3 * max_vel);
        zeroerr_motor_deceleration(&zeroerr[1], 3 * max_vel);
        zeroerr_motor_velocity(&zeroerr[1], max_vel);
        zeroerr_motor_enable(&zeroerr[1]);
        
        // motor id 8
        zeroerr[2].parent.id = 8;
        zeroerr[2].initialized = 0;
        zeroerr[2].moving = 0;
        motor_register(&zeroerr[2].parent, "MOTOR_8");
        zeroerr_motor_init(&zeroerr[2], 8, ZEROERR_MODE_ABSOLUTE_POSITION, 10, 1 << 19);
        zeroerr_motor_acceleration(&zeroerr[2], 3 * max_vel);
        zeroerr_motor_deceleration(&zeroerr[2], 3 * max_vel);
        zeroerr_motor_velocity(&zeroerr[2], max_vel);
        zeroerr_motor_enable(&zeroerr[2]);
    }
    
    register_cmd_callback(CMD_MOTOR_MIT, motor_mit_control_callback);
    register_cmd_callback(CMD_MOTOR_POSITION, motor_position_control_callback);
    
    // 1kHz motor feedback frequency.
    soft_timer_register(motor_feedback, NULL, 1);
    
    for(;;)
    {
        if (app->app_id == 4)
        {
            // 零差电机狗都不用
            for (int i = 0; i < 3; i++)
            {
                if (zeroerr[i].moving)
                {
                    if (fabs(zeroerr[i].parent.data.position - zeroerr[i].target_position) < 0.001)
                    {
                        zeroerr_motor_stop(&zeroerr[i]);
                        zeroerr[i].moving = 0;
                    }
                }
                else
                {
                    if (fabs(zeroerr[i].parent.data.position - zeroerr[i].target_position) > 0.001)
                    {
                        zeroerr_motor_relative_position(&zeroerr[i], 0);
                        zeroerr_motor_absolute_position(&zeroerr[i], zeroerr[i].target_position);
                        zeroerr_motor_move(&zeroerr[i]);
                    }
                }
            }
        }
        osDelay(10);
    }
}

void leg_control_task_init(void)
{
    osThreadDef(LEG_CONTROL_TASK, leg_control_task, osPriorityNormal, 0, 512);
    leg_control_task_t = osThreadCreate(osThread(LEG_CONTROL_TASK), NULL);
}
