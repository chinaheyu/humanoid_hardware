#ifndef __LOG_H__
#define __LOG_H__

#ifdef  __cplusplus
extern "C" {
#endif


#include <stdarg.h>
#include <string.h>
#include "sys.h"
#include "cmsis_os.h"

int log_printf_to_buffer(char *buff, int size, char *fmt, ...);
int initialize_log(void);
void log_send(uint8_t *data, uint16_t len);
void log_save_to_file(const char* file_name);
        
#define LOG_OUTPUT_MAX_LEN  512
extern char* log_str;


// 日志
#define log_printf(...) \
do \
{ \
    osThreadId_t thread_id = osThreadGetId(); \
    if(log_str != NULL) \
    { \
        int len = 0; \
        if (thread_id != NULL) { \
            len += log_printf_to_buffer(&log_str[len], LOG_OUTPUT_MAX_LEN - len, "%s,", osThreadGetName(thread_id)); \
        } else { \
            len += log_printf_to_buffer(&log_str[len], LOG_OUTPUT_MAX_LEN - len, "None,"); \
        } \
        len += snprintf(&log_str[len], LOG_OUTPUT_MAX_LEN - len, "%d.%03ds,%s:%d,%s,\"", (int)get_time_ms() / 1000, (int)get_time_ms() % 1000, __FILE__, __LINE__, __FUNCTION__); \
        len += log_printf_to_buffer(&log_str[len], LOG_OUTPUT_MAX_LEN - len, __VA_ARGS__); \
        len += log_printf_to_buffer(&log_str[len], LOG_OUTPUT_MAX_LEN - len, "\"\r\n"); \
        log_send((uint8_t *)log_str, len); \
    } \
}while(0)

// 内核日志，直接输出到串口
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
