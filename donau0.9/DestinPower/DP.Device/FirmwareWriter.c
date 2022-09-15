/*
 * FlashWrite.c
 *
 *  Created on: 2012. 11. 21.
 *      Author: destinPower
 */
#define BOOT_EXM_GLOBAL
//-#define DEBUG_PRINT

#include "FirmwareWriter.h"
#include "DSP2834x_M25P40.h"
#include "SCI.h"
#include "SPIa.h"
#include <string.h>
#include <math.h>
#include <ti/sysbios/knl/Task.h>

#define SECTOR0 0x00000L
#define SECTOR1	0x10000L
#define SECTOR2	0x20000L
#define SECTOR3	0x30000L
#define SECTOR4	0x40000L
#define SECTOR5	0x50000L
#define SECTOR6	0x60000L
#define SECTOR7	0x70000L

Uint32 ExFlashSector[8] = { SECTOR0, SECTOR1, SECTOR2, SECTOR3, SECTOR4, SECTOR5, SECTOR6, SECTOR7 };

#define STX_MSG '{'
#define END_MSG '}'
#define ACK_NEXT '>'
#define ACK_PREV '<'
#define ACK_PROGRESS '.'
#define ACK_FAIL '!'
#define ACK_FINISH '~'


enum _MsgList{
	MSG_ERASE=0,
	MSG_ERASE_SUCC,
	MSG_DOWN_READY,
	MSG_UP_FAIL,
	MSG_NOT_INTEL_FAIL,
	MSG_UPDATE_COMPLETE,
	MSG_CHECK_HEX_FILE,
	MSG_UPDATE_TO_PROGRAM_AREA,
	MSG_STATE_HEXA_FILE_OPEN,
	MSG_STATE_READ_ONE_LINE,
	MSG_STATE_CHECK_INTEL_FORMAT,
	MSG_STATE_READ_NUMBER_OF_DATA,
	MSG_STATE_WRITE_DATA_TO_EXFLASH,
	MSG_STATE_READ_DATA_FROM_EXFLASH,
	MSG_STATE_VERIFY_DATA,
	MSG_STATE_CHECK_END_OF_WRITE,
	MSG_STATE_CHECK_END_RECORD_MARK,
	MSG_STATE_WRITE_TO_PROGRAM_AREA
}MsgList;



char* Msg[] =
                {
                		"Sector erasing...\r\n",
                		"Erase process is succeed\r\n",
                		"READY_\r\n",
                		"Update Fail. SEQ: \r\n",
                		"Fail!! Use Intel Format.\r\n",
                		"System update complete.\r\n",
                		"Fail!!Check your Hex file\n",
                		"Firmware Download Complete. Writing to Program flash.",
                		"STATE_HEXA_FILE_OPEN",
                		"STATE_READ_ONE_LINE",
                		"STATE_CHECK_INTEL_FORMAT",
                		"STATE_READ_NUMBER_OF_DATA",
                		"STATE_WRITE_DATA_TO_EXFLASH",
                		"STATE_READ_DATA_FROM_EXFLASH",
                		"STATE_VERIFY_DATA",
                		"STATE_CHECK_END_OF_WRITE",
                		"STATE_CHECK_END_RECORD_MARK",
                		"STATE_WRITE_TO_PROGRAM_AREA"
                };

#define PRINTSCI(a) \
	SCI_writeString(SCI_RS232, Msg[(a)], strlen(Msg[(a)]));\
	SendFlush();
#define READ_CH(a)\
		SCI_read(SCI_RS232, (a))

private Bool FW_WriteToProgramArea(Uint32* const AddressLast);
private void FW_WriteData(Uint32 Address, Uint16* pData);
private Bool FW_ReadData(Uint32 Address, Uint16* pData);

public void FW_UpdateViaSCI()
{
	Uint16 uTempData1, uTempData2, uTempData3;
	Uint16 State = STATE_ON;
	Uint16 StatusRegister = 0;
	Uint16 WriteCounter = 0;
	Uint16 ReadData = 0;
	Uint32 Address = 0;
	Uint32 AddressLast = 0;
	char cBuffer[BUFF_SIZE];
	Uint16 uCheckSum;
	Uint16 uWriteData=0;
	Uint16 uIndex=0;
	//-Uint16 uReadPassed = 0;
	Uint16 uErrorState = 0;
	Uint16 i = 0;

	ReadData = 0;
	State = STATE_ON;

	InitExFlash();

	// TODO SPI 레지스터를 건들여서 write-read 속도를 개선해보기.
//	State = 0xb;

	for(;;)
	{
		uIndex = 0;
		switch(State)
		{
			case STATE_ON:
				/*---------------------------------------------------------------------------
					Start
				---------------------------------------------------------------------------	*/
				PRINTSCI(MSG_ERASE);
				/*
				 * 0~3 은 기존 Firmware 가 존재. 4~7에 일단 데이터를 기록 한다.
				 * 만일 프로그램 사이즈가 256k가 넘어가면 이 방식은 사용할 수 없다.
				 * !! hex 파일의 크가기 약 500k이상이 되면 안된다.
				*/

				/* Sector 4의 Address 는 0x40000 이다. */
				Address = 0x40000;
				for(uIndex=4;uIndex<8;uIndex++)
				{
					WriteEnableExFlash();
					SectorEraseExFlash(ExFlashSector[uIndex]);
				}
				PRINTSCI(MSG_ERASE_SUCC);
				PRINTSCI(MSG_DOWN_READY);
				State = STATE_DOWNLOAD_ONE_LINE;

				break;

			case STATE_DOWNLOAD_ONE_LINE:
				/*---------------------------------------------------------------------------
					Read One Line
				---------------------------------------------------------------------------*/
				SCI_write(SCI_RS232,ACK_NEXT);
				SendFlush();

//				iDelay = 0;
				while(1)
				{
					//TODO 1분간 수신되는 데이터가 없으면 종료 한다.
//					if( iDelay >= 0x59682F00 )
//					{
//						State = STATE_FAIL;
//					}
					//수신 데이터가 있을 때까지 대기
					if(	!READ_CH(cBuffer) )
						continue;

					if(cBuffer[0] == STX_MSG)
					{
						break;
					}
					else
					{
						Task_sleep(1);
						continue;
					}
				}

//				iDelay = 0;
				while(1)
				{
					//수신 데이터가 있을 때까지 대기
					//송/수신 중 통신이 두절되면 hardware reset 해야 함.
					while(SCI_haveRxBufferChar(SCI_RS232) == FALSE)
					{
						//TODO 약 10초간 수신되는 데이터가 없으면 종료 한다.
//						if( iDelay++ >= 0x59682F00 )
//						{
//							State = STATE_FAIL;
//						}
						continue;
					}

					READ_CH(&cBuffer[uIndex++]);

					if((cBuffer[uIndex-1] == END_MSG) || uIndex == 76) /*END 문자를 잃었을 경우를 대비하여. 1line의 최대 문자수는 76*/
					{
						cBuffer[uIndex-1] = '\0';
						break;
					}
					else
					{
						continue;
					}
				}

				i = 1;
				uCheckSum = 0xFF;
				while(1)
				{
					/* 무조건 2 ascii가 16비트 변수로 들어감. 0x00 ~ 0xFF. 0~255 */
					uTempData1 = AtoI(cBuffer[i])<<4;
					uTempData2 = AtoI(cBuffer[i + 1]);
					uTempData3 = (uTempData1 + uTempData2);

					uCheckSum += uTempData3;

					i+=UNIT;
					if(i >= uIndex-3)
					{
						uTempData1 = AtoI(cBuffer[i])<<4;
						uTempData2 = AtoI(cBuffer[i + 1]);
						uTempData3 = (uTempData1 + uTempData2); // Check Sum DATA
						break;
					}
				}
				uCheckSum = ~uCheckSum & 0xFF;

				// 체크 섬
				if( uCheckSum != uTempData3 )
				{
#ifdef DEBUG_PRINT
					System_printf("체크섬 에러!");
#endif
					SCI_write(SCI_RS232,ACK_PREV);
					SendFlush();
					State = STATE_CHECK_INTEL_FORMAT;
					break;
				}

				State = STATE_CHECK_INTEL_FORMAT;
				break;

			case STATE_CHECK_INTEL_FORMAT:
				/*---------------------------------------------------------------------------
					Check Intel Format
				---------------------------------------------------------------------------	*/
				if(cBuffer[START_CHAR_ADDR] != START_CHAR)
				{
					PRINTSCI(MSG_NOT_INTEL_FAIL);

					State = STATE_END_PROGRAM;
					break;
				}

				State = STATE_READ_NUMBER_OF_DATA;

				// Data Record가 아닌 Extended Segment Address Records or extended linear address record 일 경우 다음줄을 읽는다.
				// 주의: Data Record인지만을 확인하여 처리할 경우 File End 일 경우에도 다음줄을 읽으려 하여 오류 발생
				if(
				(cBuffer[TYPE_RECORD_TYPE_LSB_OFFSET] == TYPE_RECORD_EXTEND_SEG_ADDR_LSB_MARK) ||
				(cBuffer[TYPE_RECORD_TYPE_LSB_OFFSET] == TYPE_RECORD_EXTEND_LIN_ADDR_LSB_MARK)	)//+
				{
					State = STATE_DOWNLOAD_ONE_LINE;
				}

				break;

			case STATE_READ_NUMBER_OF_DATA:
				/*---------------------------------------------------------------------------
					Read Total Number of Data in Byte

					Note>> Byte Count
					if         byte count is 20(hex)
					then       2*16+0 = 2<<4+0 = 32 byte
				---------------------------------------------------------------------------*/
				uTempData1 = AtoI(cBuffer[BYTE_COUNT_ADDR])<<4;
				uTempData2 = AtoI(cBuffer[BYTE_COUNT_ADDR+1]);
				uTempData3 = (uTempData1 + uTempData2)<<1;// * 2

				/*---------------------------------------------------------------------------
					Check End of Data
				---------------------------------------------------------------------------*/
				if(uTempData3/*Byte Size*/ == 0)
				{
					uTempData1 = AtoI(cBuffer[END_RECODER_ADDR])<<4;
					uTempData2 = AtoI(cBuffer[END_RECODER_ADDR+1]);
					uTempData1 = uTempData1 + uTempData2;

					State = STATE_CHECK_END_RECORD_MARK;
				}
				else
				{
					WriteCounter = 0;
					State = STATE_WRITE_DATA_TO_EXFLASH;
				}

				break;

			case STATE_WRITE_DATA_TO_EXFLASH:
				/*---------------------------------------------------------------------------
					Write Data EXFLASH
				---------------------------------------------------------------------------*/
				WriteEnableExFlash();
				uTempData1 = AtoI(cBuffer[DATA_OFFSET + WriteCounter])<<4;
				uTempData2 = AtoI(cBuffer[DATA_OFFSET + WriteCounter + 1]);
				uWriteData = (uTempData1 + uTempData2);

				PageProgramExFlash(Address, uWriteData);
				State = STATE_READ_DATA_FROM_EXFLASH;

				break;

			case STATE_READ_DATA_FROM_EXFLASH:
				/*---------------------------------------------------------------------------
					Read Data from EXFLASH
				---------------------------------------------------------------------------*/
				StatusRegister = ReadStatusRegisterExFlash();
				StatusRegister = StatusRegister & 0x0001;

				if(StatusRegister == 0x0000)
				{
					ReadData = ReadExFlash(Address);
					ReadData = ReadData & 0x00FF;
					State = STATE_VERIFY_DATA;
				}
				else
				{
					uErrorState = MSG_STATE_READ_DATA_FROM_EXFLASH;
					State = STATE_FAIL;
				}
				break;
			case STATE_VERIFY_DATA:
				/*---------------------------------------------------------------------------
					Verify Data
				---------------------------------------------------------------------------*/
				if(uWriteData != ReadData)
				{
					uErrorState = MSG_STATE_VERIFY_DATA;
					State = STATE_FAIL;
				}
				else
				{
					Address++;
					WriteCounter += UNIT;

					if( 0 == ((Address) % 1024) )//+
					{
#ifdef DEBUG_PRINT
						System_printf("Writing... %ldKByte\n", ((Address)/1024) - 256 /*KB*/);
				    	System_flush();
#endif
					}

					State = STATE_CHECK_END_OF_WRITE;
				}

				break;
			case STATE_CHECK_END_OF_WRITE:
				/*---------------------------------------------------------------------------
					Check End of Write
				---------------------------------------------------------------------------*/
				if(WriteCounter == uTempData3)
				{
					State = STATE_DOWNLOAD_ONE_LINE;
				}
				else
				{
					State = STATE_WRITE_DATA_TO_EXFLASH;
				}
				break;

			case STATE_CHECK_END_RECORD_MARK:
				/*---------------------------------------------------------------------------
					Check End Record Mark
				---------------------------------------------------------------------------*/
				if(uTempData1 == END_RECORD_MARK)
				{
#ifdef DEBUG_PRINT
			    	System_printf("\nSuccess EXFLASH program %ldKByte / 512KByte \n", ((Address)/1024)-256);
			    	System_flush();
#endif
			    	PRINTSCI(MSG_UPDATE_TO_PROGRAM_AREA);
					State = STATE_WRITE_TO_PROGRAM_AREA;
				}
				else
				{
					uErrorState = MSG_CHECK_HEX_FILE;
					State = STATE_FAIL;
				}

				break;
			case STATE_WRITE_TO_PROGRAM_AREA:
#ifdef DEBUG_PRINT
			    	System_printf("STATE_WRITE_TO_PROGRAM_AREA\n", ((Address)/1024));
			    	System_flush();
#endif
				PRINTSCI(MSG_STATE_WRITE_TO_PROGRAM_AREA);

				// 섹터 0~3을 삭제 후 4~6데이터를 복사.
				AddressLast = Address-0x40000;

				if( FW_WriteToProgramArea(&AddressLast) )
				{
					SCI_write(SCI_RS232,ACK_FINISH);
					PRINTSCI(MSG_UPDATE_COMPLETE);
#ifdef DEBUG_PRINT
			    	System_printf("STATE_WRITE_TO_PROGRAM_AREA Succ\n", ((Address)/1024));
			    	System_flush();
#endif
				}
				else
				{
#ifdef DEBUG_PRINT
			    	System_printf("STATE_WRITE_TO_PROGRAM_AREA Failed\n", ((Address)/1024));
			    	System_flush();
#endif
					uErrorState = MSG_STATE_WRITE_TO_PROGRAM_AREA;
					State = STATE_FAIL;
					break;
				}
				State = STATE_END_PROGRAM;

				break;
			case STATE_FAIL:
				SCI_write(SCI_RS232,ACK_FAIL);
				PRINTSCI(MSG_UP_FAIL);
				PRINTSCI(uErrorState);
				State = STATE_END_PROGRAM;
				break;
			case STATE_END_PROGRAM:

#ifdef DEBUG_PRINT
				System_printf("STATE_END_PROGRAM \n",cBuffer[uIndex-1]);
				System_flush();
#endif
				/*---------------------------------------------------------------------------
					End Program
				---------------------------------------------------------------------------*/
				RebootSystem();
				return;
		}
	}
}

private Bool FW_WriteToProgramArea(Uint32* const AddressLast)
{
	Uint32 AddressSrc = 0x40000;
	Uint32 AddressDest = 0;
	Uint16 i=0;
	Uint16	ReadData = 0;
	Uint16	ReadData2 = 0;

	for(i=0;i<3;i++)
	{
		WriteEnableExFlash();
		SectorEraseExFlash(ExFlashSector[i]);
	}

	/* READ & WRITE DATA */
	while(AddressDest <= *AddressLast)
	{
		if((AddressDest % 100) == 0)
		{
			SCI_write(SCI_RS232,ACK_PROGRESS);
			SendFlush();
		}
		FW_ReadData(AddressSrc++,&ReadData );
		FW_WriteData(AddressDest++, &ReadData);
	}

	/* VERIFY */
	AddressSrc = 0;
	AddressDest = 0;
	ReadData = ReadData2 = 0;


	while(AddressDest <= *AddressLast)
	{
		if((AddressDest % 200) == 0)
		{
			SCI_write(SCI_RS232,ACK_PROGRESS);
			SendFlush();
		}

		FW_ReadData(AddressSrc++,&ReadData);
		FW_ReadData(AddressDest++,&ReadData2);

		if( ReadData != ReadData2)
		{
			return FALSE;
		}
	}

	return TRUE;
}
private void FW_WriteData(Uint32 Address, Uint16* pData)
{
	WriteEnableExFlash();
	PageProgramExFlash(Address, *pData);
}

private Bool FW_ReadData(Uint32 Address, Uint16* pData)
{
	if( (ReadStatusRegisterExFlash() & 0x0001) == 0x0000)
	{
		*pData = ReadExFlash(Address);
		*pData = *pData & 0x00FF;
	}
	else
	{
		return FALSE;
	}

	return TRUE;
}
















