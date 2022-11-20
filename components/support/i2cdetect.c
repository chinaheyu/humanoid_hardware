#include "i2cdetect.h"
#include <string.h>

/**
  * @brief          检测i2c总线上的所有设备
  * @param[in]      hi2c_ptr: i2c句柄指针
  * @param[out]     out: 输出字符串地址
  * @retval         none
  */
void i2cdetect(I2C_HandleTypeDef* hi2c_ptr, char* out)
{
    int len = 0;
    
    len += sprintf(out + len, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    
    for(int i = 0; i < 8; ++i)
    {
        len += sprintf(out + len, "%02x: ", i << 4);
        for(int j = 0; j < 16; ++j)
        {
            uint8_t address = (i << 4) + j;
            
            if(address < 0x03 || address > 0x77)
            {
                len += sprintf(out + len, "  ");
            }
            else
            {
                if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 3u, 10u))
                {
                    len += sprintf(out + len, "%02x", address);
                }
                else
                {
                    len += sprintf(out + len, "--");
                }
            }
            
            if(j < 15)
            {
                len += sprintf(out + len, " ");
            }
            else
            {
                len += sprintf(out + len, "\r\n");
            }
            
        }
    }
}
