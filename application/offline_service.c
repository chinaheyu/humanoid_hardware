#include "offline_service.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "sys.h"


#define OFFLINE_MANAGE_MAX_SIZE 10

struct offline_manage_obj
{
    uint8_t enable;
    offline_t online_func;
    offline_t offline_func;
    uint32_t last_time;
    uint32_t offline_time;
};

static struct offline_manage_obj offline_manage[OFFLINE_MANAGE_MAX_SIZE];

static uint8_t offline_service_init = 0;

int register_offline_callback(offline_t online_func, offline_t offline_func, uint32_t offline_time)
{
    if(offline_service_init)
    {
        for(int i = 0; i < OFFLINE_MANAGE_MAX_SIZE; ++i)
        {
            if(offline_manage[i].enable == 0)
            {
                offline_manage[i].enable = 1;
                offline_manage[i].last_time = get_time_ms();
                offline_manage[i].online_func = online_func;
                offline_manage[i].offline_func = offline_func;
                offline_manage[i].offline_time = offline_time;
                return i;
            }
        }
    }
    return -1;
}

void delete_offline_callback(int id)
{
    if(offline_service_init && id > -1 && id < OFFLINE_MANAGE_MAX_SIZE)
    {
        offline_manage[id].enable = 0;
        offline_manage[id].online_func = NULL;
        offline_manage[id].offline_func = NULL;
        offline_manage[id].last_time = get_time_ms();
        offline_manage[id].offline_time = 0;
    }
}

void offline_hook(int id)
{
    if(offline_service_init && id > -1 && id < OFFLINE_MANAGE_MAX_SIZE)
    {
        if(offline_manage[id].enable)
        {
            offline_manage[id].last_time = get_time_ms();
        }
    }
}

/**
  * @brief          设备离线服务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void offline_service(void *argument)
{
    led_green_toggle();
    
    uint32_t current_time_ms = get_time_ms();
    
    if(!offline_service_init)
    {
        for(int i = 0; i < OFFLINE_MANAGE_MAX_SIZE; ++i)
        {
            offline_manage[i].enable = 0;
            offline_manage[i].online_func = NULL;
            offline_manage[i].offline_func = NULL;
            offline_manage[i].last_time = current_time_ms;
            offline_manage[i].offline_time = 0;
        }
        
        offline_service_init = 1;
    }
    
    for(int i = 0; i < OFFLINE_MANAGE_MAX_SIZE; ++i)
    {
        if(offline_manage[i].enable)
        {
            if((current_time_ms - offline_manage[i].last_time) > offline_manage[i].offline_time)
            {
                if(offline_manage[i].offline_func != NULL)
                {
                    offline_manage[i].offline_func();
                }
            }
            else
            {
                if(offline_manage[i].online_func != NULL)
                {
                    offline_manage[i].online_func();
                }
            }
        }
    }
}
