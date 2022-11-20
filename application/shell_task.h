#ifndef SHELL_TASK_H
#define SHELL_TASK_H

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>


#define cli_send_str(out_str) cli_send_nbytes((const char*)(out_str), (strlen(out_str)))
void cli_send_nbytes(const char *out_str, uint32_t len);


#ifdef  __cplusplus
}  
#endif

#endif // SHELL_TASK_H
