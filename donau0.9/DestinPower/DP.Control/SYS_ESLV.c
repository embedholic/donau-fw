/*
 * SYS_ESLV.c
 *
 *  Created on: 2014. 11. 6.
 *      Author: destinPower
 */

#include "SYS_ESLV.h"
#include "SYSTEM.h"
#include "ODT.h"
#include "FAULT.h"
#include "Event.h"
#include "CC.h"
#include "MCB.h"
#include "MCCB.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "NvSRAM.h"
#include "GPIO.h"
#include "CTRL_MODE.h"

ESLV eslvCtrl;

void SYS_ESLV_Create()
{
	SYS_ESLV_UpdateParameter();
}

void SYS_ESLV_UpdateParameter()
{
	if( PRM_PCS[GRID_GATE_WAY_ENB].iValue == 0 )
		eslvCtrl.g_bEnb = (PARAM_VAL(BYP_MODE) == 2);
	else
		eslvCtrl.g_bEnb = 0;
}

#if 0 //[CO] by JCNET
/*
 * Fault또는 Stop 상태에서 Byp V에 따른 CB2와 제어 모드 설정
 */
ESLV_RETURN SYS_ESLV_BYP_Control()
{
	static Bool bStatusPrev = 0;
	static Bool bStatusNow = 0;

	if( CTRL_BYP_NormalOK() )
	{
		bStatusNow = 0;
		if( bStatusNow != bStatusPrev )
		{
			// TODO TEST
			ODT_Initialize(INVERTER.odtCb2Off);
		}

		if( ODT_Update(INVERTER.odtBypassDelay, TRUE) == ODT_FINISH )
		{
			// wait
		}
		else
		{
			// wait
			return ESLV_CTRL_ING;
		}

		// GC Mode
		CTRL_MODE_Set(PARAM_OPERATION_MODE_GC);

#if DBUG_MODE == 0
		if( !MC_GetStatus(STATUS_CB4) )
#else
		//if( MCB_GetStatus( M_CB4_SSW ) == CB_OFF )
#endif
			MCB_UpdateCmd(M_CB4_SSW, CB_CMD_ON);


#if DBUG_MODE == 0
		if( MC_GetStatus(STATUS_CB4) == OPEN )
#else
		if( MCB_GetStatus( M_CB4_SSW ) == CB_OFF )
#endif
		{
			// 5초 후 CB2 CLOSE가 되지 않으면 고장 발생
			if( ODT_Update(INVERTER.odtCb2Off, TRUE) == ODT_FINISH ) // TODO TEST
			{
				FLT_Raise(FLTH_SSW_CB4);
			}

			bStatusPrev = 0;
			return ESLV_CTRL_ING; /* STOP 상태 시퀀스 정지 */
		}
		else
		{
			// CB2-GRID CLOSED
			bStatusPrev = 0;
			return ESLV_CTRL_SUCC;
		}
	}
	else /* Bypass V 가 없다. */
	{
		bStatusNow = 1;
		if( bStatusNow != bStatusPrev )
		{
			ODT_Initialize(INVERTER.odtCb2Off);
		}

		ODT_Initialize(INVERTER.odtBypassDelay);

		// GI Mode - 5분간 재투입 방지 해지.
		CTRL_MODE_Set(PARAM_OPERATION_MODE_IS);

#if DBUG_MODE == 0
		if( MC_GetStatus(STATUS_CB4) )
#else
			//if( MCB_GetStatus( M_CB4_SSW ) == CB_ON )
#endif
			MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);

#if DBUG_MODE == 0
		if( MC_GetStatus(STATUS_CB4) == CLOSED )
#else
		if( MCB_GetStatus( M_CB4_SSW ) == CB_ON )
#endif
		{
			// odt 후 CB2 고장 발생
			if( ODT_Update(INVERTER.odtCb2Off, TRUE) == ODT_FINISH )
			{
				FLT_Raise(FLTH_SSW_CB4);
			}

			bStatusPrev = 1;
			return ESLV_CTRL_ING; /* STOP 상태 시퀀스 정지 */
		}
		else
		{
			// CB2-GRID OPEN
			bStatusPrev = 1;
			return ESLV_CTRL_SUCC;
		}
	}
}
#endif

/*
 * RUN 상태에서 그리드 복귀시 처리
 */
Bool SYS_ESLV_BYP_Resync()
{
	int flgOdt;

	if ( eslvCtrl.g_bEnb )
	{
		if( CTRL.INV.operation_mode == PARAM_OPERATION_MODE_IS )
		{
			if( CTRL_BYP_NormalOK() )
				flgOdt = ODT_Update(INVERTER.odtBypNormal, TRUE);
			else
				ODT_Initialize(INVERTER.odtBypNormal);

			if (flgOdt == ODT_FINISH)
			{
				CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;

				// trans resync
				return TRUE;
			}
		}
	}

	return FALSE;
}

