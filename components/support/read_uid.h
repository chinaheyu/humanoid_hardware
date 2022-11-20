#ifndef READ_UID_H
#define READ_UID_H
#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>
    
#define UNIQUE_ID_BYTE_SIZE 12
#define UNIQUE_ID_BASE_ADDRESS ((volatile uint8_t*)0x1FFF7A10)
    
extern void read_unique_device_ID(uint8_t* buf, uint32_t len);
    
#ifdef  __cplusplus
}  
#endif
#endif // READ_UID_H
