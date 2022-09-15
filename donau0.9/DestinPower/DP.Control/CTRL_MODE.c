/*
 * CTRL_MODE.c
 *
 *  Created on: 2014. 6. 26.
 *      Author: destinPower
 */

#include "CTRL_MODE.h"
#include "PRM_PCS.h"
#include "SYSTEM.h"
#include "Fault.h"
#include "CC.h"
#include <string.h>

//typedef struct _CTRL_MODE_BOX
//{
//	CMODE setMode;
//	CMODE operationMode;
//}CTRL_MODE_OBJ;
//static CTRL_MODE_OBJ cmObj;

extern void CTRL_INV_PRC_UpdateParameter();
extern void CTRL_INV_SYNC_UpdateParameter();
extern void CTRL_INV_PQC_UpdateParameter();
extern void CTRL_INV_DROOP_UpdateParameter();

void CTRL_MODE_Create()
{
	//memset(&cmObj, 0, sizeof(cmObj));

	CTRL_MODE_Set((CMODE)PRM_PCS[GC_IS_MODE].iValue);
}

Bool CTRL_MODE_Set(int mode)
{
	if( mode > PARAM_OPERATION_MODE_AUTO_IS || mode < PARAM_OPERATION_MODE_GC)
	{
		return FALSE;
	}
	else
	{
		//cmObj.setMode = (CMODE)mode;

		// 하단 코드 삭제시 계속 필터 초기화하여 rms계산 안됨.
		if( PRM_PCS[GC_IS_MODE].iValue == (int)mode )
			return TRUE;


		PRM_PCS[GC_IS_MODE].iValue = (int)mode;
	}

	if ( INVERTER.uStatus == SYS_INV_FAULT || INVERTER.uStatus == SYS_INV_STOP || INVERTER.uStatus == SYS_INV_DC_CHARGE
		|| INVERTER.uStatus == SYS_INV_TEST_MODE )
	{
		//Todo someting..
	}
	else
	{
		return TRUE; // just skip - for PLC
	}


	switch( PRM_PCS[GC_IS_MODE].iValue /*cmObj.setMode*/ )
	{
	case PARAM_OPERATION_MODE_GC:
		//XXX GC 모드일 경우 자동으로 P+R 모드로 들어가므로, DC SRC 모드에서는 변경하지 않는다.
		// 하단 코드가 없으면, GC 선택 시 이전에 VSI_PR 상태일경우 CSI로 변경되지 않게 된다.
		// 현재는 IS 모드일 경우 CONTROLLER_SEL을 변경하지 않도록 함.-> PQC_Update에서 제어모드를 VSI로 가져감.
		// 주의! Auto-GI를 사용할 경우 CONTROLLER_SEL을 CSI_PR로 설정해 놓아야 함.
		PRM_PCS[CONTROLLER_SEL].iValue = PARAM_CSI_PR;
		break;

	case PARAM_OPERATION_MODE_IS:
		PRM_PCS[CONTROLLER_SEL].iValue = PARAM_VSI_PR;
		INVERTER.gridReconnect.bEnb = FALSE; // GC 모드에서 5분간 재접속 방지 상태 일 경우. GI로 변경하면 상태 해지.(포스코 요청)
		FLT_SetGridFailureEvent(FALSE);
		break;
	case PARAM_OPERATION_MODE_AUTO_IS:
		PRM_PCS[CONTROLLER_SEL].iValue = PARAM_CSI_PR;
		break;
	default:
		break;
	}

	CTRL_INV_PRC_UpdateParameter();
	CTRL_INV_SYNC_UpdateParameter();
	CTRL_INV_PQC_UpdateParameter();
	CTRL_INV_DROOP_UpdateParameter();

	/// XXX 없앰.
	/// CTRL_MODE_CheckParam();

	return TRUE;
}

Bool CTRL_MODE_DPWM_ON_POSSIBLE()
{
	if( CTRL.INV.ctrl_mode == PARAM_CSI_DQ || CTRL.INV.ctrl_mode == PARAM_CSI_PR)
		return TRUE;

	return FALSE;
}

int CTRL_MODE_GetCurrentSetMode()
{
	//return cmObj.setMode;
	return PRM_PCS[GC_IS_MODE].iValue;
}

/*
 * for Modbus
 * 0: Grid Connect
 * 1: Grid Independent
 */
int CTRL_MODE_GetGC_GI_Mode()
{
/*
 * 140930 독립 운전 모드인지 확인 하는 방법을 System State를 이용하도록 수정함.

	if( CTRL.INV.ctrl_mode == PARAM_VSI_PR )
		return 1;
	else
		return 0;
*/

	if( INVERTER.uStatus == SYS_INV_ISLANDING )
	{
		return 1;
	}
	// AutoGI모드에서 GC->GI 로 전환되었을 때만 Islanding 상태이고 GI모드에서는 RUN 상태임.
	else if ( CTRL.INV.operation_mode == PARAM_OPERATION_MODE_IS )
	{
		return 1;
	}
	else if ( CTRL.INV.ctrl_mode == PARAM_VSI_PR )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


Bool CTRL_MODE_IsGIMode_Master()
{
	return ( PRM_PCS[DGT_DI1].iValue == 1 );
}

Bool CTRL_MODE_IsGIMode_Slave()
{
	return ( PRM_PCS[DGT_DI1].iValue == 0 );
}

inline Bool CTRL_MODE_NOT_EQ(PRM_PCS_LIST l, int value)
{
	if( PRM_PCS[l].iValue != value )
		return TRUE;

	return FALSE;
}
/*
 * 기본적인 파라미터 값을 확인 후 값이 변경되어 있으면 경고를 발생 시킨다.
 */
Void CTRL_MODE_CheckParam()
{
	Bool bWrong = FALSE;

	bWrong |= CTRL_MODE_NOT_EQ(ACTIVE_DAMPING, 0);
	bWrong |= CTRL_MODE_NOT_EQ(VCON_P_GAIN, 15000);

	bWrong |= CTRL_MODE_NOT_EQ(ICON_I_GAIN, 50);

	bWrong |= CTRL_MODE_NOT_EQ(BATT_CUTOFF_VOLTAGE_DCHG, 1);
	bWrong |= CTRL_MODE_NOT_EQ(BATT_VDCREF_CHG, 1);
	bWrong |= CTRL_MODE_NOT_EQ(BATT_VDCREF_DCHG, 1);
	bWrong |= CTRL_MODE_NOT_EQ(BATT_CYCLING_CNT, 10);
	bWrong |= CTRL_MODE_NOT_EQ(BATT_INIT_STATE, 10);
	bWrong |= CTRL_MODE_NOT_EQ(BATT_CHG_TIME, 10);

	bWrong |= CTRL_MODE_NOT_EQ(BATT_START_IDLE_TIME, 15000);

	bWrong |= CTRL_MODE_NOT_EQ(BATT_17, 2000);

	bWrong |= CTRL_MODE_NOT_EQ(BATT_DCC_P_GAIN, 500);
	bWrong |= CTRL_MODE_NOT_EQ(V_DROOP_RAMP, 20);
	bWrong |= CTRL_MODE_NOT_EQ(CC_CUT_OFF_FREQ, 10);

	bWrong |= CTRL_MODE_NOT_EQ(VDS_FF_GAIN, 100);
	bWrong |= CTRL_MODE_NOT_EQ(VQS_FF_GAIN, 100);

	bWrong |= CTRL_MODE_NOT_EQ(VC_PR_CUTOFF_IS, 150);

	if( bWrong )
	{
		FLT_Raise(FLTL_INVALID_PARAM);
	}

}

