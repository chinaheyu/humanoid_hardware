#include "log.h"


char* log_str = NULL;


int initialize_log(void)
{
    if(log_str == NULL)
    {
        log_str = pvPortMalloc(LOG_OUTPUT_MAX_LEN);
    }
    return log_str != NULL;
}


int log_printf_to_buffer(char *buff, int size, char *fmt, ...)
{
    int len = 0;
    va_list arg;
    va_start(arg, fmt);
    len += vsnprintf(buff, size, fmt, arg);
    va_end(arg);
    return len;
}
