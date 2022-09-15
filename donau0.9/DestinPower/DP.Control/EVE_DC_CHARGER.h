/*
 * EVE_DC_CHARGER.h
 *
 *  Created on: 2018. 1. 22.
 *      Author: Seokhyun
 */

#ifndef DESTINPOWER_DP_CONTROL_EVE_DC_CHARGER_H_
#define DESTINPOWER_DP_CONTROL_EVE_DC_CHARGER_H_

#include <xdc/runtime/Assert.h>
#include "LGS_Common.h"
#include "SYSTEM.h"

typedef struct _EVE
{
	//(MCU1 -> EVE MCU4)
	Bool bCMD_On;			// EVE On/Off ���
	Bool bCMD_FaultReset;	// EVE Fault Reset ���
	Bool bCMDCharge;			// 1: EVE ���� ���� ��, 0: Stand-by
	Bool bMC8_OnOff;
	Bool bMC8_Status;
	Bool bMC9_OnOff;
	Bool bMC9_Status;

	//(EVE MCU4 -> MCU1)
	Bool bStatus;			// EVE ���� ���� 1: EVE ���� ��, 0: EVE Off
	Bool bFaultStatus;		// EVE�� Shutdown �� SET
	Bool bInitCharge;		// EVE ���� ����

	Bool bCAN_Fail;         // ��� ���� -> EVE Off STBLN���� ����
	Bool bCAN_Fail_TxChekbit;
	Bool bCAN_Fail_RxChekbit;
	Bool bCAN_Prev_RxChekbit;
	Uint16 nCAN_Fail_Timer;

	float MAX_Charge_Volt;	// EVE Max ���� ����
	float Start_Charge_Volt;// EVE ���� ���� ����
	float Charge_Power; // EVE ���� ����


#if DBUG_MODE == 2 || DBUG_MODE == 1

#endif

} Eve;

void EVE_DC_Charger_Create(void);
void EVE_Check_EVE_Stautus(void);
void EVE_MCB_Chekcing(void);
extern void EVE_CAN_Communication_TxChecking(void);
extern void EVE_CAN_Communication_RxChecking(void);

extern Eve EVE;
extern void EVE_Proceed(void);
extern void EVE_UpdateParameter(void);

#endif /* DESTINPOWER_DP_CONTROL_EVE_DC_CHARGER_H_ */
