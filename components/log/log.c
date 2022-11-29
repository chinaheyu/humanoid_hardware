#include "log.h"
#include "bsp_usart.h"
#include "w25qxx.h"
#include "fatfs.h"


char* log_str = NULL;
static uint8_t* sector_buf;

uint32_t log_start_address;
uint32_t log_end_address;


int initialize_log(void)
{
    log_end_address = log_start_address = 0;
    if(log_str == NULL)
    {
        log_str = pvPortMalloc(LOG_OUTPUT_MAX_LEN);
    }
    if(sector_buf == NULL)
    {
        sector_buf = pvPortMalloc(w25qxx.SectorSize);
    }
    if((log_str == NULL) || (sector_buf == NULL))
    {
        
        return 0;
    }
    
    int len = log_printf_to_buffer(log_str, LOG_OUTPUT_MAX_LEN, "Thread,Time,File,Func,Log\r\n");
    log_send((uint8_t *)log_str, len);
    return 1;
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

void log_send(uint8_t *data, uint16_t len)
{
    // log to usart1
    usart1_transmit(data, len);
    
    // log to flash
    uint32_t sector_idx = log_end_address / w25qxx.SectorSize;
    uint32_t sector_offset = log_end_address % w25qxx.SectorSize;
    uint32_t written_data = 0;
    uint32_t sector_remain = w25qxx.SectorSize - sector_offset;

    while(written_data < len)
    {
        W25qxx_ReadSector(sector_buf, sector_idx, 0, w25qxx.SectorSize);
        
        int i;
        for(i = 0; i < sector_remain; ++i)
        {
            if(sector_buf[sector_offset + i] != 0xffu)
                break;
        }
        if(i < sector_remain)
        {
            W25qxx_EraseSector(sector_idx);
            
            // 丢弃扇区后面的数据
            memset(sector_buf + sector_offset, 0xffu, sector_remain);
        }
        
        if(len < sector_remain)
        {
            memcpy(sector_buf + sector_offset, data + written_data, len);
            written_data += len;
        }
        else
        {
            memcpy(sector_buf + sector_offset, data + written_data, sector_remain);
            written_data += sector_remain;
        }
        
        W25qxx_WriteSector(sector_buf, sector_idx, 0, w25qxx.SectorSize);
        
        sector_idx += 1;
        sector_offset = 0;
        sector_remain = w25qxx.SectorSize - sector_offset;
    }
    
    log_end_address += len;
}

void log_save_to_file(const char* file_name)
{
    uint8_t* log_data = pvPortMalloc(log_end_address - log_start_address);
    W25qxx_ReadBytes(log_data, log_start_address, log_end_address - log_start_address);
    
    FRESULT result;
    FIL file;
    UINT bw;
    result = f_open(&file, file_name, FA_CREATE_ALWAYS|FA_WRITE);
    if(result == FR_OK)
    {
        f_write(&file, log_data, log_end_address - log_start_address, &bw);
        f_close(&file);
    }
    
    vPortFree(log_data);
}

