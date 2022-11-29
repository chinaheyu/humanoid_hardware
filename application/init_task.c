#include "usb_device.h"
#include "dwt_stm32_delay.h"
#include "w25qxx.h"
#include "bsp_usart.h"
#include "log.h"
#include "ee24.h"
#include "os_timer.h"


/*
启动顺序：
init_task_before_kernel_start
init_task_after_kernel_start

优先级
IDLE 0
TimerService 2
SystemServices 8
其他任务 >= 24
*/


osThreadId_t shellTaskHandle;
const osThreadAttr_t shellTask_attributes = {
  .name = "ShellTask",
  .stack_size = 256 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t keyTaskHandle;
const osThreadAttr_t keyTask_attributes = {
  .name = "KeyTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t communicateTaskHandle;
const osThreadAttr_t communicateTask_attributes = {
  .name = "CommunicateTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t handTaskHandle;
const osThreadAttr_t handTask_attributes = {
  .name = "HandTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osTimerId_t offlineServiceHandle;
const osTimerAttr_t offlineService_attributes = {
  .name = "OfflineService"
};

osEventFlagsId_t keyPressEventHandle;
const osEventFlagsAttr_t keyPressEvent_attributes = {
  .name = "keyPressEvent"
};

osEventFlagsId_t shellReceivedEventHandle;
const osEventFlagsAttr_t shellReceivedEvent_attributes = {
  .name = "shellReceivedEvent"
};

osEventFlagsId_t cmdReceivedEventHandle;
const osEventFlagsAttr_t cmdReceivedEvent_attributes = {
  .name = "cmdReceivedEvent"
};


extern void shell_task(void *argument);
extern void key_task(void *argument);
extern void communicate_task(void *argument);
extern void hand_task(void *argument);
extern void offline_service(void *argument);


// 在main函数中，内核启动前的初始化代码
void init_task_before_kernel_start(void)
{
    // 初始化USB
    MX_USB_DEVICE_Init();
    kernel_log("Initialize USB device success.\r\n");
    
    // 初始化DWT延迟
    DWT_Delay_Init();
    kernel_log("Initialize DWT success.\r\n");
    
    // 初始化串口
    usart1_manage_init();
    usart2_manage_init();
    kernel_log("Initialize USART manage success.\r\n");
    
    kernel_log("Starting system kernel...\r\n");
}


// 基于DAEMON_TASK_STARTUP_HOOK，osKernelStart之后会在prvTimerTask启动时执行一次
// 用于在调度器启动后初始化相关代码
void init_task_after_kernel_start(void)
{
    kernel_log("Kernel started.\r\n");
    
    // 初始化EEPROM
    while(!ee24_isConnected())
    {
        kernel_log("EEPROM initialize failure, retrying...\r\n");
    }
    kernel_log("Initialize EEPROM success.\r\n");
    
    // 初始化FLASH
    while(!W25qxx_Init())
    {
        kernel_log("SPI flash initialize failure, retrying...\r\n");
    }
    kernel_log("Initialize SPI flash success.\r\n");
    
    // 初始化 Mutex 和 Event
    keyPressEventHandle = osEventFlagsNew(&keyPressEvent_attributes);
    shellReceivedEventHandle = osEventFlagsNew(&shellReceivedEvent_attributes);
    cmdReceivedEventHandle = osEventFlagsNew(&cmdReceivedEvent_attributes);
    kernel_log("Initialize mutex and event success.\r\n");
    
    // 初始化日志
    while(!initialize_log())
    {
        kernel_log("LOG initialize failure, retrying...\r\n");
    }
    kernel_log("Initialize LOG success.\r\n");

    // System Timer
    offlineServiceHandle = osTimerNew(offline_service, osTimerPeriodic, NULL, &offlineService_attributes);
    osTimerStart(offlineServiceHandle, 200);
    kernel_log("Initialize offline service success.\r\n");
    
    // Soft Timer
    soft_timer_init();
    soft_timer_FreeRTOS_init();
    kernel_log("Initialize soft timer success.\r\n");

    // Comman task
    keyTaskHandle = osThreadNew(key_task, NULL, &keyTask_attributes);
    communicateTaskHandle = osThreadNew(communicate_task, NULL, &communicateTask_attributes);
    handTaskHandle = osThreadNew(hand_task, NULL, &handTask_attributes);
    kernel_log("Initialize application tasks success.\r\n");
    
    kernel_log("Total initialization duration: %.2fms\r\n", get_time_ms_us());
    
    shellTaskHandle = osThreadNew(shell_task, NULL, &shellTask_attributes);
}


void services_task(void *argument)
{
    
    for(;;)
    {
        osDelay(1000);
    }
}
