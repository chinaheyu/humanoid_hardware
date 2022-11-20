#include "bsp_sram.h"
#include "stm32f4xx.h"
#include "cmsis_os.h"

//在指定地址(WriteAddr+Bank1_SRAM3_ADDR)开始,连续清0
//WriteAddr:要写入的地址
//n:要写入的字节数
void FSMC_SRAM_ClearBuffer(uint32_t WriteAddr, uint32_t n)
{
	for(;n!=0;n--)
	{
		*(__IO uint8_t*)(Bank1_SRAM3_ADDR+WriteAddr) = 0;
		WriteAddr++;
	}
}

//在指定地址(WriteAddr+Bank1_SRAM3_ADDR)开始,连续写入n个字节.
//pBuffer:字节指针
//WriteAddr:要写入的地址
//n:要写入的字节数
void FSMC_SRAM_WriteBuffer(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t n)
{
	for(;n!=0;n--)
	{
		*(__IO uint8_t*)(Bank1_SRAM3_ADDR+WriteAddr)=*pBuffer;
		WriteAddr++;
		pBuffer++;
	}
}

//在指定地址((WriteAddr+Bank1_SRAM3_ADDR))开始,连续读出n个字节.
//pBuffer:字节指针
//ReadAddr:要读出的起始地址
//n:要写入的字节数
void FSMC_SRAM_ReadBuffer(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t n)
{
	for(;n!=0;n--)
	{
		*pBuffer++=*(__IO uint8_t*)(Bank1_SRAM3_ADDR+ReadAddr);
		ReadAddr++;
	}
}

uint32_t FSMC_SRAM_TestSize(void)
{
    uint32_t i;
    uint32_t strip;
	uint8_t temp[256];   // 原有的数据
    uint8_t wridx;       // 准备写入的数据
	uint8_t sval;	     // 在地址i读到的数据
    
    // 256 * 4k 测试
    __disable_irq();
    strip = 4 * 1024;
    for(i = 0, wridx = 0; i < 256 * strip; i += strip)
    {
        FSMC_SRAM_ReadBuffer(temp + wridx, i, 1);
        FSMC_SRAM_WriteBuffer(&wridx, i, 1);
        wridx++;
    }
    for(i = 0, wridx = 0; i < 256 * strip; i += strip)
    {
        FSMC_SRAM_ReadBuffer(&sval, i, 1);
        if(sval != wridx)
            break;
        FSMC_SRAM_WriteBuffer(temp + wridx, i, 1);
        wridx++;
    }
    __enable_irq();
    
    return i;
}
