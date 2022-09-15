/*
 * FlashWrite.h
 *
 *  Created on: 2012. 11. 21.
 *      Author: destinPower
 */

#ifndef FLASHWRITE_H_
#define FLASHWRITE_H_

#include "LGS_Common.h"
#include <stdio.h>
#include "DSP2834x_Device.h"
#include "DSP2834x_Examples.h"

#define	START_CHAR			':'
#define START_CHAR_ADDR		0
#define BYTE_COUNT_ADDR		1
#define END_RECODER_ADDR	7
#define END_RECORD_MARK		1
#define DATA_OFFSET			9		/* First Data posion in boot code text (Intel format) */
#define UNIT				2		/* 1 byte = 2 character */
#define BUFF_SIZE			101		/* File Read Buffer size */
#define STATUS_REG_EXFLASH	0x0000	/* Status Register Init Value */
//+
#define TYPE_RECORD_TYPE_LSB_OFFSET 8
#define TYPE_RECORD_EXTEND_SEG_ADDR_LSB_MARK '2'
#define TYPE_RECORD_EXTEND_LIN_ADDR_LSB_MARK '4'


#define STATE_ON						0x0000
#define STATE_HEXA_FILE_OPEN			0x0001
#define STATE_DOWNLOAD_ONE_LINE			0x0002
#define STATE_CHECK_INTEL_FORMAT		0x0003
#define STATE_READ_NUMBER_OF_DATA		0x0004
#define STATE_WRITE_DATA_TO_EXFLASH		0x0005
#define STATE_READ_DATA_FROM_EXFLASH	0x0006
#define STATE_VERIFY_DATA				0x0007
#define STATE_CHECK_END_OF_WRITE		0x0008
#define STATE_CHECK_END_RECORD_MARK		0x0009
#define STATE_END_PROGRAM				0x000A
#define STATE_WRITE_TO_PROGRAM_AREA		0x000B
#define STATE_FAIL						0x000C

public void FW_UpdateViaSCI();



#endif /* FLASHWRITE_H_ */
