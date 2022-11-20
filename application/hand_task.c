#include "cmsis_os.h"
#include "linear_actuator.h"
#include "fifo.h"
#include "bsp_usart.h"


static LA_UnpackStream* LA_stream;
static fifo_s_t LA_rx_fifo;
static char* LA_rx_fifo_buf;


uint32_t LA_rx_callback(uint8_t *buff, uint16_t len)
{
    len = fifo_s_puts(&LA_rx_fifo, (char *)buff, len);
    return len;
}

void hand_task(void *argument)
{
    LA_FreeRTOS_Init();
    
    LA_stream = (LA_UnpackStream*)pvPortMalloc(sizeof(LA_UnpackStream));
    LA_InitializeUnpackStream(LA_stream);
    
    LA_rx_fifo_buf = (char*)pvPortMalloc(1024);
    fifo_s_init(&LA_rx_fifo, LA_rx_fifo_buf, 1024);
    
    usart2_rx_callback_register(LA_rx_callback);
    
    while(1)
    {
        while(!fifo_s_isempty(&LA_rx_fifo))
        {
            int ch = fifo_s_get(&LA_rx_fifo);
            
            // ½âÎöÊý¾Ý
            LA_UnpackByte(LA_stream, ch);
        }
        
        osDelay(1);
    }
}
