#ifndef __i2CDETECT_H__
#define __i2CDETECT_H__
#ifdef  __cplusplus
extern "C" {
#endif
    
    
#include "i2c.h"
    
/**
  * @brief          检测i2c总线上的所有设备
  * @param[in]      hi2c_ptr: i2c句柄指针
  * @param[out]     out: 输出字符串地址
  * @retval         none
  */
void i2cdetect(I2C_HandleTypeDef* hi2c_ptr, char* out);
    
    
#ifdef  __cplusplus
}  
#endif
#endif
