#include "cmsis_os.h"
#include "bsp_key.h"
#include "log.h"
#include "communicate_task.h"


void key_task(void *argument)
{
    key_callback_flag = 1;
    
    while(1)
    {
        uint32_t flags  = osEventFlagsWait(keyPressEventHandle, 0x00000003U, osFlagsWaitAny, osWaitForever);
        
        if(flags & 0x80000000U)
        {
            // 最高位为1 说明发生错误
            
            log_printf("KEY event error.");
            osDelay(200);
        }
        
        if(flags & 0x00000001U)
        {
            // 处理按键KEY0
            
            // echo 测试
            const char* str = "Hello World!";
            float st = get_time_ms_us();
            RPCResponse res;
            if(remote_procedure_call(CMD_ECHO_REQUEST, (uint8_t*)str, strlen(str) + 1, CMD_ECHO_RESPONSE, &res, 5000))
            {
                log_printf("Echo response using %fms: \"%s\".", get_time_ms_us() - st, res.data);
            }
            else
            {
                log_printf("Echo response timeout.");
            }
            
            osDelay(200);
        }
        
        if(flags & 0x00000002U)
        {
            // 处理按键KEY_UP
            
            log_printf("KEY_UP is pressed.");
            osDelay(200);
        }

        key_callback_flag = 1;
    }
}

