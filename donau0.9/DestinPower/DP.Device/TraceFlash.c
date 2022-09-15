/*
 * TraceFlash.c
 *
 *  Created on: 2014. 12. 3.
 *      Author: destinPower
 *
 *  Hardware Layer : SPIa.h
 *
 *  Trace 데이터 최대 8 개
 *  TYPE : INT16
 *  SAMPLING : CC PERIOD * 2 ~ 3
 *  1 SEC ( 200ms=고장전, 800ms=고장후 -> 버퍼링 )
 *
 *  1 / ( (CC_PERIOD us * SAMPLING ) =
 *  SAMPLING 2, CC_PERIOD 200 일 경우 : 2500 회 Sample
 *  -> 총 Flash 저장 Bytes = 2500 * 2byte * 8channel =  39.0625 KB / Sec  -> 156.25회 저장( 156.25 * 621us = 약 98ms소요. )
 *
 *  버퍼 개수 : 2 ( 실시간 저장 버퍼 A, 실시간 저장 버퍼 B)
 *  시퀀스 : 장비 RUN -> 실시간 저장 BUFFER A(200ms) -> 고장 발생 -> 최초 고장 발생 시각 및 그 때의 고장정보 저장, 트레이스번호 -> 버퍼A를 FLASH에 저장(256byte단위, CRC추가), 그 동안 실시간 저장은 버퍼 B, 반복 - 종료
 *  *Erase Count
 */

#include "TraceFlash.h"
#include "SPIa.h"
#include <string.h>
#include "TraceFlash.h"
#include "Fault.h"
/*
 * FLASH Sector
 * 0~63개 섹터, 섹터당 64KB
 * 0x0000(0xFFFF) ~  0x3F0000(0x3F FFFF)
 */

#if EXTRACE_USE_SPI_FLASH == 1
#define SECTOR_OFFSET 0x10000
#define SECTOR_CHANNEL_OFFSET 0x1000
#define TRACE_STORAGE_STX 0x10000


/* BUFFER */
#pragma DATA_SECTION(EX_TRACE_BUF_PING, "H45")
#pragma DATA_SECTION(EX_TRACE_BUF_PONG, "H45")
int EX_TRACE_BUF_PING[TF_TRACE_CHANNEL_MAX][TF_TF_BUFSIZE]; // 200ms buf
int EX_TRACE_BUF_PONG[TF_TRACE_CHANNEL_MAX][TF_TF_BUFSIZE]; // 200ms buf


/* 최대 30개의 고장에 대한 저장 정보 - NvSRAM */
#pragma DATA_SECTION(FaultInfoList, "H45")
TF_FAULT_INFO FaultInfoList;
TF_MANAGER TM;


void TF_Create()
{
	// NvSRAM으로 부터 노드 정보를 읽어 온다.
	// TM

	memset(EX_TRACE_BUF_PING, 0, sizeof(EX_TRACE_BUF_PING));
	memset(EX_TRACE_BUF_PONG, 0, sizeof(EX_TRACE_BUF_PONG));
	memset(&FaultInfoList, 0, sizeof(FaultInfoList));
	memset(&TM, 0, sizeof(TM));
}

void TF_DataHook(UInt16 idx, int channel, void* pValue)
{
	if( idx >= TF_TRACE_CHANNEL_MAX )
		return;

	TM.traceList[idx] = channel;
	TM.pTraceValue[idx] = pValue;
}

int pingI=0;
int pongI=0;
int bStop = 0;

void TF_UpdateData()
{
	int i = 0;
	float fTemp = 0;

	if( !FLT_GetHeavyStatus() )
	{
		/* 고장 전 200ms */
		for( i=0; i< TF_TRACE_CHANNEL_MAX; i++)
		{
			fTemp =  *TM.pTraceValue[i];

			/* 1Byte는 최대 +-32765 인데 scale을 10으로 맞추므로. fabs X*/
			// 전류가 너무 클 수 있을 경우(1MW. Gain을 0.5로 주도록 한다. )
			if( fTemp > 3270.0 ) 				fTemp = 3270;
			else if( fTemp < -3270.0 )			fTemp = -3270;

			EX_TRACE_BUF_PING[i][pingI] = ( fTemp * 10 + 0.5f );
		}

		if( ++pingI >= TF_TF_BUFSIZE )
		{
			pingI = 0;
		}
	}
	else
	{
		if( bStop )
			return;
		/* 고장 후 200ms */
		for( i=0; i< TF_TRACE_CHANNEL_MAX; i++)
		{
			EX_TRACE_BUF_PONG[i][pongI] = ( *TM.pTraceValue[i] * 10 + 0.5f );
		}

		if( ++pongI >= TF_TF_BUFSIZE )
		{
			pongI = 0;
			bStop = 1;
		}
	}
}

void TF_PushData(int channel, int value)
{

}

void TF_WriteFlash(int mode)
{
	int i = 0;
	int j = 0;

	if( mode == 0 ) /* ping */
	{
		//2바이트 단위 변수이며, 데이터 저장 구조를 채널별로 어떻게 진행할 것인지 잘 결정해야 함.
		//1채널 - 500 * 2byte 저장. ( 4회 )
		// sector 옵셋은?
		for(i = 0; i < 8; i++)
		{
			for(j =0; j < 4; j++)
			{
//				if( j != 4 )
//					SPIA_dataFlash_writeByte(TRACE_STORAGE_STX, EX_TRACE_BUF_PING[i][j+128/*변수당 2byte므로*/],256);
//				else
//					SPIA_dataFlash_writeByte(TRACE_STORAGE_STX, EX_TRACE_BUF_PING[i][j+128],232);

			}
		}
	}
	else /* pong */
	{

	}
}

#endif
