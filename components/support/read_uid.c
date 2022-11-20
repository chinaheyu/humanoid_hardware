#include "read_uid.h"


// 读取UID，长度不能大于UNIQUE_ID_BYTE_SIZE
void read_unique_device_ID(uint8_t* buf, uint32_t len)
{
    for(int i = 0; i < len && i < UNIQUE_ID_BYTE_SIZE; ++i)
    {
        buf[i] = UNIQUE_ID_BASE_ADDRESS[i];
    }
}



