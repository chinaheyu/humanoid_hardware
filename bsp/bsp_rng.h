#ifndef BSP_RNG_H
#define BSP_RNG_H

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint32_t RNG_get_random_num(void);
extern int32_t RNG_get_random_rangle(int min, int max);

#ifdef  __cplusplus
}  
#endif

#endif
