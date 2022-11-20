#ifndef __LOG_H__
#define __LOG_H__

#ifdef  __cplusplus
extern "C" {
#endif


#include <stdarg.h>
#include "bsp_usart.h"
#include "sys.h"
#include "cmsis_os.h"

int log_printf_to_buffer(char *buff, int size, char *fmt, ...);
int initialize_log(void);
        
#define LOG_OUTPUT_MAX_LEN  256
extern char* log_str;

// 日志，用于调试
#define verbose_log(...) \
do \
{ \
    if(log_str != NULL) \
    { \
        int len = 0; \
        len += snprintf(&log_str[len], LOG_OUTPUT_MAX_LEN - len, "\r\n[Time]: %d.%03ds\r\n[File]: %s:%d\r\n[Func]: %s()\r\n>> ", (int)get_time_ms() / 1000, (int)get_time_ms() % 1000, __FILE__, __LINE__, __FUNCTION__); \
        len += log_printf_to_buffer(&log_str[len], LOG_OUTPUT_MAX_LEN - len, __VA_ARGS__); \
        len += snprintf(&log_str[len], LOG_OUTPUT_MAX_LEN - len, "\r\nstm32>> "); \
        usart1_transmit((uint8_t *)log_str, len); \
    } \
}while(0)

#define short_log(...) \
do \
{ \
    if(log_str != NULL) \
    { \
        int len = 0; \
        len += snprintf(&log_str[len], LOG_OUTPUT_MAX_LEN - len, "\r\n[%d.%03d] ", (int)get_time_ms() / 1000, (int)get_time_ms() % 1000); \
        len += log_printf_to_buffer(&log_str[len], LOG_OUTPUT_MAX_LEN - len, __VA_ARGS__); \
        usart1_transmit((uint8_t *)log_str, len); \
    } \
}while(0)

// 内核日志，用于在操作系统启动前调试
#define kernel_log(...) \
do \
{ \
    debug_raw_printf("[%d.%03d] ", (int)get_time_ms() / 1000, (int)get_time_ms() % 1000); \
    debug_raw_printf(__VA_ARGS__); \
}while(0)

#ifdef  __cplusplus
}  
#endif

#endif // __LOG_H__
