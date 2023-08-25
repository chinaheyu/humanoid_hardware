#include "test.h"
#include <cmsis_os.h>
#include "motor.h"
#include "app_manage.h"

osThreadId test_task_t;

void test_task(void const* arg)
{
    for(;;)
    {
        osDelay(10);
    }
}

void test_task_init(void)
{
    osThreadDef(TEST_TASK, test_task, osPriorityNormal, 0, 512);
    test_task_t = osThreadCreate(osThread(TEST_TASK), NULL);
}
