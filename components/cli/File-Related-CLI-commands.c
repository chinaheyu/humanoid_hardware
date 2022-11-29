/*
 * FreeRTOS V202112.00
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */


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

/* File system includes. */
#include "fatfs.h"
#include "w25qxx.h"
#include "ee24.h"
#include "log.h"


#ifdef _WINDOWS_
	#define snprintf _snprintf
#endif

#define cliNEW_LINE		"\r\n"

/*******************************************************************************
 * See the URL in the comments within main.c for the location of the online
 * documentation.
 ******************************************************************************/

/*
 * Print out information on a single file.
 */
static void prvCreateFileInfoString( char *pcBuffer, FILINFO *pxFindStruct );
void fsResultVarToString(FRESULT r, char* out);

/*
 * Copies an existing file into a newly created file.
 */
static BaseType_t prvPerformCopy( const char *pcSourceFile,
									int32_t lSourceFileLength,
									const char *pcDestinationFile,
									char *pxWriteBuffer,
									size_t xWriteBufferLen );

//static BaseType_t prvMOUNTCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
                                    
/*
 * Implements the DIR command.
 */
static BaseType_t prvLSCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements the CD command.
 */
static BaseType_t prvCDCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements the DEL command.
 */
static BaseType_t prvRMCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements the TYPE command.
 */
static BaseType_t prvCATCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements the COPY command.
 */
static BaseType_t prvCPCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements the PWD command.
 */
static BaseType_t prvPWDommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements the MKDIR command.
 */
static BaseType_t prvMKDIRCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static BaseType_t prvWriteFileToFlashCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvWriteFileToEEPROMCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvWriteLogToFileCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/* Structure that defines the DIR command line command, which lists all the
files in the current directory. */
static const CLI_Command_Definition_t xLS =
{
	"ls", /* The command string to type. */
	"\r\nls:\r\n Lists the files in the current directory\r\n",
	prvLSCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* Structure that defines the CD command line command, which changes the
working directory. */
static const CLI_Command_Definition_t xCD =
{
	"cd", /* The command string to type. */
	"\r\ncd <dir name>:\r\n Changes the working directory\r\n",
	prvCDCommand, /* The function to run. */
	1 /* One parameter is expected. */
};

/* Structure that defines the TYPE command line command, which prints the
contents of a file to the console. */
static const CLI_Command_Definition_t xCAT =
{
	"cat", /* The command string to type. */
	"\r\ncat <filename>:\r\n Prints file contents to the terminal\r\n",
	prvCATCommand, /* The function to run. */
	1 /* One parameter is expected. */
};

/* Structure that defines the DEL command line command, which deletes a file. */
static const CLI_Command_Definition_t xRM =
{
	"rm", /* The command string to type. */
	"\r\nrm <filename>:\r\n deletes a file or directory\r\n",
	prvRMCommand, /* The function to run. */
	1 /* One parameter is expected. */
};

/* Structure that defines the COPY command line command, which deletes a file. */
static const CLI_Command_Definition_t xCP =
{
	"cp", /* The command string to type. */
	"\r\ncp <source file> <dest file>:\r\n Copies <source file> to <dest file>\r\n",
	prvCPCommand, /* The function to run. */
	2 /* Two parameters are expected. */
};

static const CLI_Command_Definition_t xPWD =
{
	"pwd", /* The command string to type. */
	"\r\npwd:\r\nPrint your current work directory.\r\n",
	prvPWDommand, /* The function to run. */
	0 /* Two parameters are expected. */
};

static const CLI_Command_Definition_t xMKDIR =
{
	"mkdir", /* The command string to type. */
	"\r\nmkdir:\r\nCreate a directory.\r\n",
	prvMKDIRCommand, /* The function to run. */
	1 /* Two parameters are expected. */
};

static const CLI_Command_Definition_t xWriteFileToFlash =
{
	"write-file-to-flash", /* The command string to type. */
	"\r\nwrite-file-to-flash <write-address> <filename>:\r\n Write the file context to enternal flash. \r\n",
	prvWriteFileToFlashCommand, /* The function to run. */
	2 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xWriteFileToEEPROM =
{
	"write-file-to-eeprom", /* The command string to type. */
	"\r\nwrite-file-to-eeprom <write-address> <filename>:\r\n Write the file context to enternal EEPROM. \r\n",
	prvWriteFileToEEPROMCommand, /* The function to run. */
	2 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xWriteLogToFile =
{
	"write-log-to-file", /* The command string to type. */
	"\r\nwrite-log-to-file <filename>:\r\n Write the log from flash to file.\r\n",
	prvWriteLogToFileCommand, /* The function to run. */
	1 /* No parameters are expected. */
};

//static const CLI_Command_Definition_t xMOUNT =
//{
//	"mount", /* The command string to type. */
//	"\r\nmount <dev>:\r\nMount storage device.\r\n",
//	prvMOUNTCommand, /* The function to run. */
//	0 /* Two parameters are expected. */
//};

/*-----------------------------------------------------------*/

void vRegisterFileSystemCLICommands( void )
{
	/* Register all the command line commands defined immediately above. */
	FreeRTOS_CLIRegisterCommand( &xLS );
	FreeRTOS_CLIRegisterCommand( &xCD );
	FreeRTOS_CLIRegisterCommand( &xCAT );
	FreeRTOS_CLIRegisterCommand( &xRM );
	FreeRTOS_CLIRegisterCommand( &xCP );
    FreeRTOS_CLIRegisterCommand( &xPWD );
    FreeRTOS_CLIRegisterCommand( &xMKDIR );
    FreeRTOS_CLIRegisterCommand( &xWriteFileToFlash );
    FreeRTOS_CLIRegisterCommand( &xWriteFileToEEPROM );
    FreeRTOS_CLIRegisterCommand( &xWriteLogToFile );
    // FreeRTOS_CLIRegisterCommand( &xMOUNT );

}
/*-----------------------------------------------------------*/

static BaseType_t prvCATCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
const char *pcParameter;
BaseType_t xParameterStringLength, xReturn = pdTRUE;
static FIL *pxFile = NULL;
int iChar;
size_t xByte;
size_t xColumns = 50U;

	/* Ensure there is always a null terminator after each character written. */
	memset( pcWriteBuffer, 0x00, xWriteBufferLen );

	/* Ensure the buffer leaves space for the \r\n. */
	configASSERT( xWriteBufferLen > ( strlen( cliNEW_LINE ) * 2 ) );
	xWriteBufferLen -= strlen( cliNEW_LINE );

	if( xWriteBufferLen < xColumns )
	{
		/* Ensure the loop that uses xColumns as an end condition does not
		write off the end of the buffer. */
		xColumns = xWriteBufferLen;
	}

	if( pxFile == NULL )
	{
		/* The file has not been opened yet.  Find the file name. */
		pcParameter = FreeRTOS_CLIGetParameter
						(
							pcCommandString,		/* The command string itself. */
							1,						/* Return the first parameter. */
							&xParameterStringLength	/* Store the parameter string length. */
						);

		/* Sanity check something was returned. */
		configASSERT( pcParameter );

		/* Attempt to open the requested file. */
        pxFile = pvPortMalloc(sizeof(FIL));
		retSD = f_open( pxFile, pcParameter, FA_READ );
        
        if(retSD != FR_OK)
        {
            fsResultVarToString(retSD, pcWriteBuffer);
            vPortFree(pxFile);
            pxFile = NULL;
        }
	}

	if( pxFile != NULL )
	{
		/* Read the next chunk of data from the file. */
		for( xByte = 0; xByte < xColumns; xByte++ )
		{
            UINT br;
            retSD = f_read(pxFile, &iChar, 1, &br);
            
            if(retSD != FR_OK || br == 0)
            {
                iChar = -1;
            }

			if( iChar == -1 )
			{
				/* No more characters to return. */
                vPortFree(pxFile);
				f_close( pxFile );
				pxFile = NULL;
				break;
			}
			else
			{
				pcWriteBuffer[ xByte ] = ( char ) iChar;
			}
		}
	}

	if( pxFile == NULL )
	{
		/* Either the file was not opened, or all the data from the file has
		been returned and the file is now closed. */
        strcat( pcWriteBuffer, cliNEW_LINE );
		xReturn = pdFALSE;
	}
    
	return xReturn;
}
/*-----------------------------------------------------------*/

static BaseType_t prvCDCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
const char *pcParameter;
BaseType_t xParameterStringLength;
size_t xStringLength;

	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter
					(
						pcCommandString,		/* The command string itself. */
						1,						/* Return the first parameter. */
						&xParameterStringLength	/* Store the parameter string length. */
					);

	/* Sanity check something was returned. */
	configASSERT( pcParameter );

	/* Attempt to move to the requested directory. */
	retSD = f_chdir( pcParameter );

	if( retSD == FR_OK )
	{
		sprintf( pcWriteBuffer, "In: " );
		xStringLength = strlen( pcWriteBuffer );
		f_getcwd( &( pcWriteBuffer[ xStringLength ] ), ( unsigned char ) ( xWriteBufferLen - xStringLength ) );
	}
	else
	{
		fsResultVarToString(retSD, pcWriteBuffer);
	}

	strcat( pcWriteBuffer, cliNEW_LINE );

	return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvLSCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
static FILINFO *pxFindStruct = NULL;
BaseType_t xReturn = pdFALSE;
static DIR *dp = NULL;
TCHAR* currnet_path;

	/* This assumes pcWriteBuffer is long enough. */
	( void ) pcCommandString;

	/* Ensure the buffer leaves space for the \r\n. */
	configASSERT( xWriteBufferLen > ( strlen( cliNEW_LINE ) * 2 ) );
	xWriteBufferLen -= strlen( cliNEW_LINE );
    
    // 每次读取当前路径
    currnet_path = pvPortMalloc(256 * sizeof(TCHAR));
    retSD = f_getcwd(currnet_path, 256 * sizeof(TCHAR));
    
    if(retSD != FR_OK)
    {
        vPortFree(currnet_path);
        fsResultVarToString(retSD, pcWriteBuffer);
        strcat( pcWriteBuffer, cliNEW_LINE );
        return pdFALSE;
    }

	if( pxFindStruct == NULL )
	{
		/* This is the first time this function has been executed since the Dir
		command was run.  Create the find structure. */
		pxFindStruct = ( FILINFO * ) pvPortMalloc( sizeof( FILINFO ) );
        dp = pvPortMalloc(sizeof(DIR));

		if( pxFindStruct != NULL )
		{
			retSD = f_findfirst( dp, pxFindStruct, currnet_path, "*" );

			if( retSD == FR_OK )
			{
				prvCreateFileInfoString( pcWriteBuffer, pxFindStruct );
				xReturn = pdPASS;
			}
			else
			{
				snprintf( pcWriteBuffer, xWriteBufferLen, "Error: f_findfirst() failed." );
			}
		}
		else
		{
			snprintf( pcWriteBuffer, xWriteBufferLen, "Failed to allocate RAM (using heap_4.c will prevent fragmentation)." );
		}
	}
	else
	{
		/* The find struct has already been created.  Find the next file in
		the directory. */
		retSD = f_findnext( dp, pxFindStruct );

		if( retSD == FR_OK && pxFindStruct->fname[0] != 0 )
		{
			prvCreateFileInfoString( pcWriteBuffer, pxFindStruct );
			xReturn = pdPASS;
		}
		else
		{
			/* There are no more files.  Free the find structure. */
			vPortFree( pxFindStruct );
            vPortFree(dp);
            dp = NULL;
			pxFindStruct = NULL;

			/* No string to return. */
			pcWriteBuffer[ 0 ] = 0x00;
		}
	}

	strcat( pcWriteBuffer, cliNEW_LINE );
    
    // 释放内存
    vPortFree(currnet_path);

	return xReturn;
}
/*-----------------------------------------------------------*/

static BaseType_t prvRMCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
const char *pcParameter;
BaseType_t xParameterStringLength;

	/* This function assumes xWriteBufferLen is large enough! */
	( void ) xWriteBufferLen;

	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter
					(
						pcCommandString,		/* The command string itself. */
						1,						/* Return the first parameter. */
						&xParameterStringLength	/* Store the parameter string length. */
					);

	/* Sanity check something was returned. */
	configASSERT( pcParameter );

	/* Attempt to delete the file. */
	retSD = f_unlink( pcParameter );

	if( retSD == FR_OK )
	{
		sprintf( pcWriteBuffer, "%s was deleted", pcParameter );
	}
	else
	{
		fsResultVarToString(retSD, pcWriteBuffer);
	}

	strcat( pcWriteBuffer, cliNEW_LINE );

	return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvCPCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
char *pcSourceFile, *pcDestinationFile;
BaseType_t xParameterStringLength;
long lSourceLength, lDestinationLength = 0;
    
	/* Obtain the name of the destination file. */
	pcDestinationFile = ( char * ) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										2,						/* Return the second parameter. */
										&xParameterStringLength	/* Store the parameter string length. */
									);

	/* Sanity check something was returned. */
	configASSERT( pcDestinationFile );

	/* Obtain the name of the source file. */
	pcSourceFile = ( char * ) FreeRTOS_CLIGetParameter
								(
									pcCommandString,		/* The command string itself. */
									1,						/* Return the first parameter. */
									&xParameterStringLength	/* Store the parameter string length. */
								);

	/* Sanity check something was returned. */
	configASSERT( pcSourceFile );

	/* Terminate the string. */
	pcSourceFile[ xParameterStringLength ] = 0x00;

	/* See if the source file exists, obtain its length if it does. */
    FILINFO fno;
    retSD = f_stat(pcSourceFile, &fno);
    
    if(retSD == FR_OK)
    {
        lSourceLength = fno.fsize;
    }
    else
    {
        lSourceLength = 0;
    }
        

	if( lSourceLength == 0 )
	{
		sprintf( pcWriteBuffer, "Source file does not exist" );
	}
	else
	{
		/* See if the destination file exists. */
        FILINFO fno;
        retSD = f_stat(pcDestinationFile, &fno);
        if(retSD != FR_NO_FILE)
        {
            lDestinationLength = fno.fsize;
        }
        else
        {
            lDestinationLength = 0;
        }

		if( lDestinationLength != 0 )
		{
			sprintf( pcWriteBuffer, "Error: Destination file already exists" );
		}
	}

	/* Continue only if the source file exists and the destination file does
	not exist. */
	if( ( lSourceLength != 0 ) && ( lDestinationLength == 0 ) )
	{
		if( prvPerformCopy( pcSourceFile, lSourceLength, pcDestinationFile, pcWriteBuffer, xWriteBufferLen ) == pdPASS )
		{
			sprintf( pcWriteBuffer, "Copy made" );
		}
		else
		{
			sprintf( pcWriteBuffer, "Error during copy" );
		}
	}

	strcat( pcWriteBuffer, cliNEW_LINE );

	return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvPerformCopy( const char *pcSourceFile,
									int32_t lSourceFileLength,
									const char *pcDestinationFile,
									char *pxWriteBuffer,
									size_t xWriteBufferLen )
{
int32_t lBytesRead = 0, lBytesToRead, lBytesRemaining;
FIL *pxFile;
BaseType_t xReturn = pdPASS;
FRESULT result;

    pxFile = pvPortMalloc(sizeof(FIL));
    
	/* NOTE:  Error handling has been omitted for clarity. */

	while( lBytesRead < lSourceFileLength )
	{
		/* How many bytes are left? */
		lBytesRemaining = lSourceFileLength - lBytesRead;

		/* How many bytes should be read this time around the loop.  Can't
		read more bytes than will fit into the buffer. */
		if( lBytesRemaining > ( long ) xWriteBufferLen )
		{
			lBytesToRead = ( long ) xWriteBufferLen;
		}
		else
		{
			lBytesToRead = lBytesRemaining;
		}

		/* Open the source file, seek past the data that has already been
		read from the file, read the next block of data, then close the
		file again so the destination file can be opened. */
		result = f_open( pxFile, pcSourceFile, FA_READ );
		if( result == FR_OK )
		{
			f_lseek( pxFile, lBytesRead );
			f_read( pxFile, pxWriteBuffer, lBytesToRead, (UINT*)&lBytesToRead );
			f_close( pxFile );
		}
		else
		{
			xReturn = pdFAIL;
			break;
		}

		/* Open the destination file and write the block of data to the end of
		the file. */
		result = f_open( pxFile, pcDestinationFile, FA_OPEN_APPEND|FA_WRITE );
		if( result == FR_OK )
		{
			f_write( pxFile, pxWriteBuffer, lBytesToRead, (UINT*)&lBytesToRead );
			f_close( pxFile );
		}
		else
		{
			xReturn = pdFAIL;
			break;
		}

		lBytesRead += lBytesToRead;
	}
    
    vPortFree(pxFile);

	return xReturn;
}
/*-----------------------------------------------------------*/

static void prvCreateFileInfoString( char *pcBuffer, FILINFO *pxFindStruct )
{
const char *pcWritableFile = "writable file", *pcReadOnlyFile = "read only file", *pcDirectory = "directory";
const char * pcAttrib;

	/* Point pcAttrib to a string that describes the file. */
	if( ( pxFindStruct->fattrib & AM_DIR ) != 0 )
	{
		pcAttrib = pcDirectory;
	}
	else if( pxFindStruct->fattrib & AM_RDO )
	{
		pcAttrib = pcReadOnlyFile;
	}
	else
	{
		pcAttrib = pcWritableFile;
	}

	/* Create a string that includes the file name, the file size and the
	attributes string. */
	sprintf( pcBuffer, "%s [%s] [size=%d]", pxFindStruct->fname, pcAttrib, ( int ) pxFindStruct->fsize );
}

static BaseType_t prvPWDommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
    
    retSD = f_getcwd(pcWriteBuffer, xWriteBufferLen);
    
    if(retSD != FR_OK)
    {
        fsResultVarToString(retSD, pcWriteBuffer);
    }
    
    strcat( pcWriteBuffer, cliNEW_LINE );
    

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

void fsResultVarToString(FRESULT r, char* out)
{
    switch(r)
    {
        case FR_OK:
            sprintf(out, "Succeeded");
        break;
        case FR_DISK_ERR:
            sprintf(out, "A hard error occurred in the low level disk I/O layer");
        break;
        case FR_INT_ERR:
            sprintf(out, "Assertion failed");
        break;
        case FR_NOT_READY:
            sprintf(out, "The physical drive cannot work");
        break;
        case FR_NO_FILE:
            sprintf(out, "Could not find the file");
        break;
        case FR_NO_PATH:
            sprintf(out, "Could not find the path");
        break;
        case FR_INVALID_NAME:
            sprintf(out, "The path name format is invalid");
        break;
        case FR_DENIED:
            sprintf(out, "Access denied due to prohibited access or directory full");
        break;
        case FR_EXIST:
            sprintf(out, "Access denied due to prohibited access");
        break;
        case FR_INVALID_OBJECT:
            sprintf(out, "The file/directory object is invalid");
        break;
        case FR_WRITE_PROTECTED:
            sprintf(out, "The physical drive is write protected");
        break;
        case FR_INVALID_DRIVE:
            sprintf(out, "The logical drive number is invalid");
        break;
        case FR_NOT_ENABLED:
            sprintf(out, "The volume has no work area");
        break;
        case FR_NO_FILESYSTEM:
            sprintf(out, "There is no valid FAT volume");
        break;
        case FR_MKFS_ABORTED:
            sprintf(out, "The f_mkfs() aborted due to any problem");
        break;
        case FR_TIMEOUT:
            sprintf(out, "Could not get a grant to access the volume within defined period");
        break;
        case FR_LOCKED:
            sprintf(out, "The operation is rejected according to the file sharing policy");
        break;
        case FR_NOT_ENOUGH_CORE:
            sprintf(out, "LFN working buffer could not be allocated");
        break;
        case FR_TOO_MANY_OPEN_FILES:
            sprintf(out, "Number of open files > _FS_LOCK");
        break;
        case FR_INVALID_PARAMETER:
            sprintf(out, "Given parameter is invalid");
        break;
        default:
            sprintf(out, "Unknown error.");
    }
}

static BaseType_t prvWriteFileToFlashCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *write_address;
    const char *filename;
    BaseType_t xParameterStringLength;
    configASSERT( pcWriteBuffer );
    
	/* Obtain the parameter string. */
	write_address = FreeRTOS_CLIGetParameter
					(
						pcCommandString,		/* The command string itself. */
						1,						/* Return the first parameter. */
						&xParameterStringLength	/* Store the parameter string length. */
					);
    filename = FreeRTOS_CLIGetParameter
					(
						pcCommandString,		/* The command string itself. */
						2,						/* Return the first parameter. */
						&xParameterStringLength	/* Store the parameter string length. */
					);

    FIL fp;

    retSD = f_open(&fp, filename, FA_READ);
    if(retSD != FR_OK)
    {
        fsResultVarToString(retSD, pcWriteBuffer);
        strcat( pcWriteBuffer, "\r\n" );
    }
    else
    {
        uint32_t file_size = f_size(&fp);
        
        uint8_t* buf = pvPortMalloc(file_size);

        uint32_t written_size;
        uint32_t addr = atof(write_address);

        retSD = f_read(&fp, buf, file_size, &written_size);
        W25qxx_WriteAnyBytes(addr, written_size, buf);
        
        snprintf(pcWriteBuffer, xWriteBufferLen, "Flash Finished.\r\nTotal file size: %d bytes. Successfully written size: %d bytes\r\nYou can check the result at page %d address 0x%08X\r\n", file_size, written_size, addr / w25qxx.PageSize, addr);

        vPortFree(buf);
    }
    
    retSD = f_close(&fp);

    return pdFALSE;
}

static BaseType_t prvWriteFileToEEPROMCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *write_address;
    const char *filename;
    BaseType_t xParameterStringLength;
    configASSERT( pcWriteBuffer );
    
	/* Obtain the parameter string. */
	write_address = FreeRTOS_CLIGetParameter
					(
						pcCommandString,		/* The command string itself. */
						1,						/* Return the first parameter. */
						&xParameterStringLength	/* Store the parameter string length. */
					);
    filename = FreeRTOS_CLIGetParameter
					(
						pcCommandString,		/* The command string itself. */
						2,						/* Return the first parameter. */
						&xParameterStringLength	/* Store the parameter string length. */
					);

    FIL fp;

    retSD = f_open(&fp, filename, FA_READ);
    if(retSD != FR_OK)
    {
        fsResultVarToString(retSD, pcWriteBuffer);
        strcat( pcWriteBuffer, "\r\n" );
    }
    else
    {
        uint32_t file_size = f_size(&fp);
        
        uint8_t* buf = pvPortMalloc(file_size);

        uint32_t written_size;
        uint32_t addr = atof(write_address);

        retSD = f_read(&fp, buf, file_size, &written_size);
        
        uint32_t max_size = _EEPROM_SIZE_KBIT * 128 - addr;
        if(written_size > max_size)
        {
            written_size = max_size;
        }
        ee24_write(addr, buf, written_size, 100);
        
        snprintf(pcWriteBuffer, xWriteBufferLen, "Write Finished.\r\nTotal file size: %d bytes. Successfully written size: %d bytes\r\n", file_size, written_size);

        vPortFree(buf);
    }
    
    retSD = f_close(&fp);

    return pdFALSE;
}


//static BaseType_t prvMOUNTCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
//{
//BaseType_t xParameterStringLength;

//	/* This function assumes xWriteBufferLen is large enough! */
//	( void ) xWriteBufferLen;
//    ( void ) pcCommandString;

//    
//    retSD = f_mount(&SDFatFS, SDPath, 1);
//    fsResultVarToString(retSD, pcWriteBuffer);
//    strcat( pcWriteBuffer, cliNEW_LINE );

//	/* There is no more data to return after this single string, so return
//	pdFALSE. */
//	return pdFALSE;
//}

static BaseType_t prvMKDIRCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
const char *pcParameter;
BaseType_t xParameterStringLength;

	/* This function assumes xWriteBufferLen is large enough! */
	( void ) xWriteBufferLen;

	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter
					(
						pcCommandString,		/* The command string itself. */
						1,						/* Return the first parameter. */
						&xParameterStringLength	/* Store the parameter string length. */
					);
    
    retSD = f_mkdir(pcParameter);
    fsResultVarToString(retSD, pcWriteBuffer);
    strcat( pcWriteBuffer, cliNEW_LINE );
    
    return pdFALSE;
}

static BaseType_t prvWriteLogToFileCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *filename;
    BaseType_t xParameterStringLength;
    configASSERT( pcWriteBuffer );
    
	/* Obtain the parameter string. */
    filename = FreeRTOS_CLIGetParameter
					(
						pcCommandString,		/* The command string itself. */
						1,						/* Return the first parameter. */
						&xParameterStringLength	/* Store the parameter string length. */
					);

    log_save_to_file(filename);
    
    sprintf(pcWriteBuffer, "Finished.\r\n");

    return pdFALSE;
}
