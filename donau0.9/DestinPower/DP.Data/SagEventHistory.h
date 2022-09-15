/*
 * SagEventHistory.h
 *
 *  Created on: 2016. 7. 20.
 *      Author: Yang_PC
 */

#ifndef SAGEVENTHISTORY_H_
#define SAGEVENTHISTORY_H_

#include "LGS_Common.h"
#include "CTRL_INV_SEAMLESS.h"
#include "CTRL_BYP_EVT_OPERATION.h"
#include "Odt.h"
#include "SYSTEM.h"
#include "time.h"
#include <string.h>

#define SAG_STATE_CALL_PERIOD 5 /*ms*/

typedef struct _SagEvt{
	Uint16	YearMonth;	// 연월
	Uint16	DateHour;	// 일시
	Uint16	MinSec;		// 분초

	Uint16	CompT;		// Sag 발생 후 부터 보상시간
	Bool	bRestore;	// 보상 후 복구 유무

	float 	VoltLV;		// Sag 발생 시 전압 레벨
	float 	VoltR;		// Sag 발생 시 전압 레벨 R
	float 	VoltS;		// Sag 발생 시 전압 레벨 S
	float 	VoltT;		// Sag 발생 시 전압 레벨 T

	float	CompVoltR;	// Sag 발생 시 보상 전압 R
	float	CompVoltS;	// Sag 발생 시 보상 전압 S
	float	CompVoltT;	// Sag 발생 시 보상 전압 T

	// june comment: 추후 trace 연결 시 trace buffer ring으로 변경 또는 채널을 늘리고 trace stx pointer 연결 할 것.
	// bool bTraceCaptured
	// int* pTraceHook
	// int iTraceSize
}SagEvt;

typedef enum _SAG_STATE{
	SAG_DATA_SEND				,
	SAG_DATA_UPDATE
}SAG_STATE;

typedef struct _SagOperation
{
	SAG_STATE Sag_uStatus;
	Bool Sag_data_ready_flag; /* For HMI */
	Bool Sag_time_odt_flag;   /* For grid sag point */
	Bool bFilterDelayLatch;

	SagEvt Sag_data_Instant; /* Sag Capture Data */
	SagEvt Sag_data_Shadow;  /* For Modbus Data */
}SagOperation;


extern void Sag_Evt_History_Create(void);
extern void Sag_Evt_History_Proceed(void);
extern SagOperation SAG_OP;

#endif /* DESTINPOWER_DP_DATA_SAGEVENTHISTORY_H_ */
