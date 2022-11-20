#include "bsp_usb.h"
#include "fifo.h"

extern fifo_s_t usb_rx_fifo;
extern fifo_s_t usb_tx_fifo;

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/**
  * @brief          通过usb虚拟串口发送数据，但不会直接发送
  * @param[in]      data: 数据指针
  * @param[in]      len: 数据长度
  * @retval         none
  */
void usb_interface_send(uint8_t *data, uint16_t len)
{
    CDC_Transmit_FS(data, len);
}

/**
  * @brief          刷新tx缓冲区，发送所有数据
  * @retval         none
  */
int32_t usb_tx_flush(void *argc)
{
    var_cpu_sr();

    uint8_t result = USBD_OK;
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if (hcdc->TxState != 0)
    {
        return USBD_BUSY;
    }
    else
    {
        uint32_t send_num;

        enter_critical();
        send_num = usb_tx_fifo.used_num;
        fifo_s_gets_noprotect(&usb_tx_fifo, (char *)UserTxBufferFS, send_num);
        exit_critical();

        USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, send_num);
        result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        return result;
    }
}

/**
  * @brief          重置rx缓冲区
  * @retval         none
  */
void usb_reset_rx_buf(void)
{
    fifo_s_flush(&usb_rx_fifo);
}

/**
  * @brief          重置tx缓冲区
  * @retval         none
  */
void usb_reset_tx_buf(void)
{
    fifo_s_flush(&usb_tx_fifo);
}

/**
  * @brief          read a byte from the serial port. -1 = failure
  * @param[in]      data: 数据指针
  * @param[in]      len: 数据长度
  * @retval         none
  */
int usb_rx_fifo_read(void)
{
    if(fifo_s_isempty(&usb_rx_fifo))
    {
        return -1;
    }
    return (uint8_t)fifo_s_get(&usb_rx_fifo);
}
