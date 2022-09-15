/*
 * SPI.h
 *
 *  Created on: 2012. 11. 9.
 *      Author: destinPower
 */

#ifndef SPIA_H_
#define SPIA_H_

#include "LGS_Common.h"

typedef enum {
	BOOT_FLASH,
	DATA_FLASH
}SPI_SELECT;

public void SPIA_init();

public void SPIA_bootFlash_ENB();
public void SPIA_dataFlash_ENB();

public Bool SPIA_dataFlash_writeInt16(UInt32 address/*24bit*/,char* const pData, Uint16 size);
public Bool SPIA_dataFlash_readByteInt16(UInt32 address,char* const pData, Uint16 size);

public Bool SPIA_dataFlash_writeByte(UInt32 address/*24bit*/,char* const pData, Uint16 size);
public Bool SPIA_dataFlash_readByte(UInt32 address,char* const pData, Uint16 size);
public Bool SPIA_dataFlash_Erase();
public int SPIA_dataFlash_ReadStatus();

// DATA_FLASH
#define DF_SECTOR0 0x000000L
#define DF_SECTOR1 0x010000L
#define DF_SECTOR2 0x020000L
#define DF_SECTOR3 0x030000L
#define DF_SECTOR4 0x040000L
#define DF_SECTOR5 0x050000L
#define DF_SECTOR6 0x060000L
#define DF_SECTOR7 0x070000L
#define DF_SECTOR8 0x080000L
#define DF_SECTOR9 0x090000L
#define DF_SECTOR10 0x0A0000L
#define DF_SECTOR11 0x0B0000L
#define DF_SECTOR12 0x0C0000L
#define DF_SECTOR13 0x0D0000L
#define DF_SECTOR14 0x0E0000L
#define DF_SECTOR15 0x0F0000L
#define DF_SECTOR16 0x100000L
#define DF_SECTOR17 0x110000L
#define DF_SECTOR18 0x120000L
#define DF_SECTOR19 0x130000L
#define DF_SECTOR20 0x140000L

#define FC_WREN 0x06
#define FC_WRDI 0x04 // Write Disable
#define FC_RDSR 0x05
#define FC_WRSR 0x01
#define FC_READ 0x03
#define FC_PP	0x02
#define FC_SE 0xD8
#define FC_DUMMY 0x00

typedef union
{
	unsigned int unsVal;
	struct
	{
		unsigned int WIP:1; // write in progress
		unsigned int WEL:1; // write enable latch bit. write enable 되어 있는지 확인
		unsigned int BP0:1; // block protect bit
		unsigned int BP1:1;
		unsigned int BP2:1;
		unsigned int P_ERR:1; 	// Programming Error Occurred
		unsigned int E_ERR:1;  	// Erase Error Occurred
		unsigned int SRWD:1; 	// Status Register write protect
		unsigned int REV__:8;
	}BitVal;
}FlashStatusRegister;
#endif /* SPIA_H_ */
