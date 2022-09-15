/*
 * TraceFlash.h
 *
 *  Created on: 2014. 12. 3.
 *      Author: destinPower
 *
 */

#ifndef TRACEFLASH_H_
#define TRACEFLASH_H_

#if EXTRACE_USE_SPI_FLASH == 1
#include "LGS_Common.h"
#include "RtcTime.h"

#define TF_NODE_BUF_MAX 30
#define TF_TRACE_CHANNEL_MAX 8
#define TF_TF_BUFSIZE 500

typedef struct
{
	RtcTime faultTime;   /* 6byte */
	Uint32  faultInfo;   /* 10byte */
	UInt16 OperationLoad; /* 전류 부하량 */

	Uint32 flashAddress;
	UInt32 flashSize;
}TF_FAULT_INFO;

typedef struct
{
	UInt16 traceList[TF_TRACE_CHANNEL_MAX];
	float*  pTraceValue[TF_TRACE_CHANNEL_MAX];

	UInt16 usedIdx;
	UInt16 nextIdx;

}TF_MANAGER;


typedef struct
{
	/* Page(256byte) 단위로 Write할 것이므로 이 구조체의 byte합은 256 이하가 되어야 함 */
	UInt16 seqId;
	Int16 Data[TF_NODE_BUF_MAX][TF_TRACE_CHANNEL_MAX]; /* 30 Sample * 8Channel */
	Int16 CheckSum;

	union
	{
		UInt16 info;
		struct
		{
			UInt16 isBrokenNode : 1;
			UInt16 rev: 15;
		}BIT;
	}NODE_DIAG;
}TF_NodeCell;

void TF_Create();
void TF_DataHook(UInt16 idx, int channel, void* pValue);
void TF_UpdateData();
#endif
#endif /* TRACEFLASH_H_ */
