#include "shell_commands.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

#include "bsp_sram.h"
#include "bsp_adc.h"
#include "w25qxx.h"
#include "read_uid.h"
#include "fatfs.h"
#include "ee24.h"

#include "i2cdetect.h"


extern void fsResultVarToString(FRESULT r, char* out);


static BaseType_t prvReadUIDCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvTestExternalSramCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvTestExternalFlashCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvEraseFlashChipCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvFlashReadPageCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvReadEEPROMCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvHeapDetailCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvMeasureTempCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvI2CDetectCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );


static const CLI_Command_Definition_t xReadUID =
{
	"read-uid", /* The command string to type. */
	"\r\nread-uid:\r\n Read the Unique Device ID of stm32 chip. \r\n",
	prvReadUIDCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xTestExternalSram =
{
	"test-external-sram", /* The command string to type. */
	"\r\ntest-external-sram:\r\n Test the external sram size. \r\n",
	prvTestExternalSramCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xTestExternalFlash =
{
	"test-external-flash", /* The command string to type. */
	"\r\ntest-external-flash:\r\n Test the external flash size. \r\n",
	prvTestExternalFlashCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xEraseFlashChip =
{
	"erase-flash-chip", /* The command string to type. */
	"\r\nerase-flash-chip:\r\n Erase the entile flash chip. This operation takes a long time. \r\n",
	prvEraseFlashChipCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xFlashReadPage =
{
	"flash-read-page", /* The command string to type. */
	"\r\nflash-read-page <page-num>:\r\n Read the specified page of enternal flash. \r\n",
	prvFlashReadPageCommand, /* The function to run. */
	1 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xReadEEPROM =
{
	"read-eeprom", /* The command string to type. */
	"\r\nread-eeprom:\r\n Read the enternal EEPROM. \r\n",
	prvReadEEPROMCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xHeapDetail =
{
	"heap-detail", /* The command string to type. */
	"\r\nheap-detail:\r\n Print the details of heap state. \r\n",
	prvHeapDetailCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xMeasureTemp =
{
	"measure-temp", /* The command string to type. */
	"\r\nmeasure-temp:\r\n Measure the temperature of MCU through the inside sensor. \r\n",
	prvMeasureTempCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xI2CDetect =
{
	"i2c-detect", /* The command string to type. */
	"\r\ni2c-detect:\r\n Detect all devive connected through i2c bus.\r\n",
	prvI2CDetectCommand, /* The function to run. */
	0 /* No parameters are expected. */
};


static BaseType_t prvReadUIDCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
    
    int len = 0;
    int i;
    
    uint8_t buf[UNIQUE_ID_BYTE_SIZE];
    
    read_unique_device_ID(buf, UNIQUE_ID_BYTE_SIZE);
    
    for(i = 0; i < UNIQUE_ID_BYTE_SIZE - 1; ++i)
    {
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "%02X-", buf[i]);
    }
    len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "%02X\r\n", buf[i]);

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

static BaseType_t prvTestExternalSramCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
    
    const char *const pcHeader = "External SRAM Memory Size: ";
    strcpy(pcWriteBuffer, pcHeader);
    
    sprintf(pcWriteBuffer + strlen(pcHeader), "%d bytes\r\n", FSMC_SRAM_TestSize());//显示内存容量

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

static BaseType_t prvTestExternalFlashCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	configASSERT( pcWriteBuffer );
    
    int len = 0;
    int i;
    
    switch (w25qxx.ID)
	{
	case W25Q512: // 	w25q512
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Device: W25Q512\r\n");
		break;
	case W25Q256: // 	w25q256
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Device: W25Q256\r\n");
		break;
	case W25Q128: // 	w25q128
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Device: W25Q128\r\n");
		break;
	case W25Q64: //	w25q64
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Device: W25Q64\r\n");
		break;
	case W25Q32: //	w25q32
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Device: W25Q32\r\n");
		break;
	case W25Q16: //	w25q16
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Device: W25Q16\r\n");
		break;
	case W25Q80: //	w25q80
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Device: W25Q80\r\n");
		break;
	case W25Q40: //	w25q40
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Device: W25Q40\r\n");
		break;
	case W25Q20: //	w25q20
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Device: W25Q20\r\n");
		break;
	case W25Q10: //	w25q10
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Device: W25Q10\r\n");
		break;
	default: // w25qxx Unknown ID
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Device: Unknown\r\n");
        break;
	}
    
    uint64_t Uid = 0;
    for(i = 0; i < 8; ++i)
    {
        Uid += w25qxx.UniqID[i];
        Uid <<= 8;
    }
    len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "UniqID: 0x%016llx\r\n", Uid);
    
    len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "PageSize: %d\r\n", w25qxx.PageSize);
    
    len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "PageCount: %d\r\n", w25qxx.PageCount);
    
    len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "SectorSize: %d\r\n", w25qxx.SectorSize);
    
    len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "SectorCount: %d\r\n", w25qxx.SectorCount);
    
    len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "BlockSize: %d\r\n", w25qxx.BlockSize);
    
    len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "BlockCount: %d\r\n", w25qxx.BlockCount);
    
    len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "Capacity: %d kbytes\r\n", w25qxx.CapacityInKiloByte);

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

static BaseType_t prvEraseFlashChipCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
    
    W25qxx_EraseChip();
    sprintf(pcWriteBuffer, "Erase finished.\r\n");
    
    return pdFALSE;
}

static BaseType_t prvFlashReadPageCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter;
    BaseType_t xParameterStringLength;
    configASSERT( pcWriteBuffer );
    
    BaseType_t ret = pdTRUE;
    
	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter
					(
						pcCommandString,		/* The command string itself. */
						1,						/* Return the first parameter. */
						&xParameterStringLength	/* Store the parameter string length. */
					);
    
    static uint32_t current_address = 0;
    static uint8_t unfinished_flag = 0;
    static uint32_t page_idx = 0;
    
    int len = 0;
    int i;
    
    if(!unfinished_flag)
    {
        current_address = 0;
        unfinished_flag = 1;
        page_idx = atoi(pcParameter);
    }
    
    if(current_address < w25qxx.PageSize)
    {
        uint8_t buf[16];
        W25qxx_ReadBytes(buf, page_idx * w25qxx.PageSize + current_address, 16);
        
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "0x%08X: ", page_idx * w25qxx.PageSize + current_address);
        
        for(i = 0; i < 8; ++i)
        {
            len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "%02X ", buf[i]);
        }
        
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "- ");
        
        for(i = 8; i < 16; ++i)
        {
            len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "%02X ", buf[i]);
        }
        
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, " | ");
        
        for(i = 0; i < 16; ++i)
        {
            if(buf[i] >= 32 && buf[i] <= 126)
            {
                pcWriteBuffer[len + i] = buf[i];
            }
            else
            {
                pcWriteBuffer[len + i] = '.';
            }
        }
        len += 16;
        
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "\r\n");

        current_address += 16;
    }
    else
    {
        pcWriteBuffer[0] = '\0';
        unfinished_flag = 0;
        ret = pdFALSE;
    }
    
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return ret;
}

static BaseType_t prvReadEEPROMCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter;
    BaseType_t xParameterStringLength;
    configASSERT( pcWriteBuffer );
    
    UNUSED(xParameterStringLength);
    UNUSED(pcParameter);
    
    BaseType_t ret = pdTRUE;
    
    static uint32_t current_address = 0;
    static uint8_t unfinished_flag = 0;
    
    int len = 0;
    int i;
    
    if(!unfinished_flag)
    {
        current_address = 0;
        unfinished_flag = 1;
    }
    
    if(current_address < _EEPROM_SIZE_KBIT * 128)
    {
        uint8_t buf[16];
        ee24_read(current_address, buf, 16, 100);
        
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "0x%08X: ", current_address);
        
        for(i = 0; i < 8; ++i)
        {
            len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "%02X ", buf[i]);
        }
        
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "- ");
        
        for(i = 8; i < 16; ++i)
        {
            len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "%02X ", buf[i]);
        }
        
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, " | ");
        
        for(i = 0; i < 16; ++i)
        {
            if(buf[i] >= 32 && buf[i] <= 126)
            {
                pcWriteBuffer[len + i] = buf[i];
            }
            else
            {
                pcWriteBuffer[len + i] = '.';
            }
        }
        len += 16;
        
        len += snprintf(pcWriteBuffer + len, xWriteBufferLen - len, "\r\n");

        current_address += 16;
    }
    else
    {
        pcWriteBuffer[0] = '\0';
        unfinished_flag = 0;
        ret = pdFALSE;
    }
    
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return ret;
}

static BaseType_t prvHeapDetailCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );
    
    HeapStats_t heap_state;
    vPortGetHeapStats(&heap_state);

    sprintf( pcWriteBuffer,
    "Available Heap Space: %d bytes\r\n"
    "Size Of Largest Free Block: %d bytes\r\n"
    "Size Of Smallest Free Block: %d bytes\r\n"
    "Number Of Free Blocks: %d\r\n"
    "Minimum Ever Free: %d bytes\r\n"
    "Number Of Successful Allocations: %d\r\n"
    "Number Of Successful Frees: %d\r\n",
    heap_state.xAvailableHeapSpaceInBytes,
    heap_state.xSizeOfLargestFreeBlockInBytes,
    heap_state.xSizeOfSmallestFreeBlockInBytes,
    heap_state.xNumberOfFreeBlocks,
    heap_state.xMinimumEverFreeBytesRemaining,
    heap_state.xNumberOfSuccessfulAllocations,
    heap_state.xNumberOfSuccessfulFrees
    );

    /* There is no more data to return after this single string, so return
    pdFALSE. */
    return pdFALSE;
}

static BaseType_t prvMeasureTempCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
    
    const char *const pcHeader = "Temperature: ";
    strcpy(pcWriteBuffer, pcHeader);
    
    sprintf(pcWriteBuffer + strlen(pcHeader), "%f C\r\n", get_cpu_temperature());//显示温度

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

static BaseType_t prvI2CDetectCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
    
    int len = sprintf(pcWriteBuffer, "             I2C1 (SCL: PB8, SDA: PB9)\r\n");
    i2cdetect(&hi2c1, pcWriteBuffer + len);
    
    //int len = sprintf(pcWriteBuffer, "            I2C2 (SCL: PB10, SDA: PB11)\r\n");
    //i2cdetect(&hi2c2, pcWriteBuffer + len);

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

void register_shell_commands(void)
{
    FreeRTOS_CLIRegisterCommand( &xReadUID );
    FreeRTOS_CLIRegisterCommand( &xTestExternalSram );
    FreeRTOS_CLIRegisterCommand( &xTestExternalFlash );
    FreeRTOS_CLIRegisterCommand( &xEraseFlashChip );
    FreeRTOS_CLIRegisterCommand( &xFlashReadPage );
    FreeRTOS_CLIRegisterCommand( &xReadEEPROM );
    FreeRTOS_CLIRegisterCommand( &xHeapDetail );
    FreeRTOS_CLIRegisterCommand( &xMeasureTemp );
    FreeRTOS_CLIRegisterCommand( &xI2CDetect );
    
}
