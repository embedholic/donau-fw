/*
 * TraceFlash.c
 *
 *  Created on: 2014. 12. 3.
 *      Author: destinPower
 *
 *  Hardware Layer : SPIa.h
 *
 *  Trace ������ �ִ� 8 ��
 *  TYPE : INT16
 *  SAMPLING : CC PERIOD * 2 ~ 3
 *  1 SEC ( 200ms=������, 800ms=������ -> ���۸� )
 *
 *  1 / ( (CC_PERIOD us * SAMPLING ) =
 *  SAMPLING 2, CC_PERIOD 200 �� ��� : 2500 ȸ Sample
 *  -> �� Flash ���� Bytes = 2500 * 2byte * 8channel =  39.0625 KB / Sec  -> 156.25ȸ ����( 156.25 * 621us = �� 98ms�ҿ�. )
 *
 *  ���� ���� : 2 ( �ǽð� ���� ���� A, �ǽð� ���� ���� B)
 *  ������ : ��� RUN -> �ǽð� ���� BUFFER A(200ms) -> ���� �߻� -> ���� ���� �߻� �ð� �� �� ���� �������� ����, Ʈ���̽���ȣ -> ����A�� FLASH�� ����(256byte����, CRC�߰�), �� ���� �ǽð� ������ ���� B, �ݺ� - ����
 *  *Erase Count
 */

#include "TraceFlash.h"
#include "SPIa.h"
#include <string.h>
#include "TraceFlash.h"
#include "Fault.h"
/*
 * FLASH Sector
 * 0~63�� ����, ���ʹ� 64KB
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


/* �ִ� 30���� ���忡 ���� ���� ���� - NvSRAM */
#pragma DATA_SECTION(FaultInfoList, "H45")
TF_FAULT_INFO FaultInfoList;
TF_MANAGER TM;


void TF_Create()
{
	// NvSRAM���� ���� ��� ������ �о� �´�.
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
		/* ���� �� 200ms */
		for( i=0; i< TF_TRACE_CHANNEL_MAX; i++)
		{
			fTemp =  *TM.pTraceValue[i];

			/* 1Byte�� �ִ� +-32765 �ε� scale�� 10���� ���߹Ƿ�. fabs X*/
			// ������ �ʹ� Ŭ �� ���� ���(1MW. Gain�� 0.5�� �ֵ��� �Ѵ�. )
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
		/* ���� �� 200ms */
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
		//2����Ʈ ���� �����̸�, ������ ���� ������ ä�κ��� ��� ������ ������ �� �����ؾ� ��.
		//1ä�� - 500 * 2byte ����. ( 4ȸ )
		// sector �ɼ���?
		for(i = 0; i < 8; i++)
		{
			for(j =0; j < 4; j++)
			{
//				if( j != 4 )
//					SPIA_dataFlash_writeByte(TRACE_STORAGE_STX, EX_TRACE_BUF_PING[i][j+128/*������ 2byte�Ƿ�*/],256);
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
