#include "test.h"
#include <cmsis_os.h>
#include "motor.h"
#include "app_manage.h"

osThreadId test_task_t;

motor_device_t motor_obj;

void test_zeroerr()
{
    struct zeroerr_motor_device zeroerr;
    motor_obj = (motor_device_t)&zeroerr;
    
    zeroerr_init(&can1_manage);
    motor_register(&zeroerr.parent, "MOTOR_1");

    zeroerr_motor_init(&zeroerr, 10, ZEROERR_MODE_VELOCITY, 10, 1 << 19);
    zeroerr_motor_acceleration(&zeroerr, 2);
    zeroerr_motor_deceleration(&zeroerr, 2);
    zeroerr_motor_analog_value(&zeroerr, 5);
    zeroerr_motor_enable(&zeroerr);
    
    for(;;)
    {
        osDelay(10);
    }
}

void test_dm()
{
    struct dm_motor_device dm;
    motor_obj = (motor_device_t)&dm;
    
    dm_init(&can1_manage, 0);
    dm_motor_init(&dm, 1, DM_MODE_MIT);
    motor_register(&dm.parent, "MOTOR_1");
    dm_motor_enable(&dm);
    
    for(;;)
    {
        dm_motor_mit_control(&dm, 0, 5, 0, 1, 0);
        osDelay(10);
    }
}

void test_tmotor()
{
    struct tmotor_motor_device tmotor;
    motor_obj = (motor_device_t)&tmotor;
    
    tmotor_init(&can2_manage, 0);
    tmotor_motor_init(&tmotor, 1, TMOTOR_MODE_MIT, 0.125, -12.5, 12.5, -50.0, 50.0, -65.0, 65.0, 0, 500, 0, 5);
    motor_register(&tmotor.parent, "MOTOR_1");
    tmotor_motor_enable(&tmotor);
    
    for(;;)
    {
        tmotor_motor_mit_control(&tmotor, 0, 5, 0, 2, 0);
        osDelay(10);
    }
}

void test_task(void const* arg)
{
    test_zeroerr();
}

void test_task_init(void)
{
    osThreadDef(TEST_TASK, test_task, osPriorityNormal, 0, 512);
    test_task_t = osThreadCreate(osThread(TEST_TASK), NULL);
}
