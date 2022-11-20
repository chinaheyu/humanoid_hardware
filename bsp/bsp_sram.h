#ifndef BSP_SRAM_H
#define BSP_SRAM_H

#include <stdint.h>

#define Bank1_SRAM3_ADDR    ((uint32_t)(0x68000000))

extern void FSMC_SRAM_ClearBuffer(uint32_t WriteAddr, uint32_t n);
extern void FSMC_SRAM_WriteBuffer(uint8_t *pBuffer,uint32_t WriteAddr,uint32_t n);
extern void FSMC_SRAM_ReadBuffer(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t n);
extern uint32_t FSMC_SRAM_TestSize(void);

#endif // BSP_SRAM_H
