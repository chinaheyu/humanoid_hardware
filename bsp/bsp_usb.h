#ifndef BSP_USB_H
#define BSP_USB_H

#ifdef  __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usbd_cdc_if.h"

/**
  * @brief          通过usb虚拟串口发送数据
  * @param[in]      data: 数据指针
  * @param[in]      len: 数据长度
  * @retval         none
  */
extern void usb_interface_send(uint8_t *data, uint16_t len);
   
/**
  * @brief          刷新tx缓冲区，发送所有数据
  * @retval         none
  */    
extern int32_t usb_tx_flush(void *argc);

/**
  * @brief          重置rx缓冲区
  * @retval         none
  */
extern void usb_reset_rx_buf(void);

/**
  * @brief          重置tx缓冲区
  * @retval         none
  */
extern void usb_reset_tx_buf(void);

/**
  * @brief          read a byte from the serial port. -1 = failure
  * @param[in]      data: 数据指针
  * @param[in]      len: 数据长度
  * @retval         none
  */
extern int usb_rx_fifo_read(void);

#ifdef  __cplusplus
}  
#endif

#endif
