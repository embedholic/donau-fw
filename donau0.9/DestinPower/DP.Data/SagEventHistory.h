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
	Uint16	YearMonth;	// ����
	Uint16	DateHour;	// �Ͻ�
	Uint16	MinSec;		// ����

	Uint16	CompT;		// Sag �߻� �� ���� ����ð�
	Bool	bRestore;	// ���� �� ���� ����

	float 	VoltLV;		// Sag �߻� �� ���� ����
	float 	VoltR;		// Sag �߻� �� ���� ���� R
	float 	VoltS;		// Sag �߻� �� ���� ���� S
	float 	VoltT;		// Sag �߻� �� ���� ���� T

	float	CompVoltR;	// Sag �߻� �� ���� ���� R
	float	CompVoltS;	// Sag �߻� �� ���� ���� S
	float	CompVoltT;	// Sag �߻� �� ���� ���� T

	// june comment: ���� trace ���� �� trace buffer ring���� ���� �Ǵ� ä���� �ø��� trace stx pointer ���� �� ��.
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
