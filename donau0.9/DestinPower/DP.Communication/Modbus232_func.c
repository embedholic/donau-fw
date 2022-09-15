/*
 * Modbus232_func.c
 *
 *  Created on: 2013. 11. 8.
 *      Author: oci-ems
 */


#include <xdc/std.h>
#include <string.h>
#include "Modbus232_func.h"
#include "Modbus232.h"
#include "DSP2834x_Device.h"
#include "DSP2834x_Examples.h"

#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include "SCI.h"
#include "Circ.h"
#include "CC.h"
#include "Fault.h"
#include "System.h"
#include "Event.h"
#include "Trace.h"
#include "MCCB.h"
#include "PRM_PCS.h"
#include "MCB.h"
#include "CAN_GBI.h"
#include "Version.h"
#include "MO.h"
#include "CTRL_MODE.h"
#include "GPIO.h"
#include "CTRL_Gen.h"
#include "MVP.h"
#include "SagEventHistory.h"


extern int bPcsCommandLocked;
extern UInt16 iCabTemp;
UInt32 i32Status = 0;
unsigned int counter = 0;

extern unsigned int debug_rx_cnt;
extern unsigned int debug_tx_cnt;

//extern int bTraceSaved;

Int16 MT_getPcs_A(int offset)
{
	Int16 temp;
	int i;
	temp = 0;
//return offset * 10;
	switch(offset)
	{
	case 0: // manufacture rev
		return MAJOR_MODEL+SP_MODEL;
	case 1: // manufacture - model type
		return PRM_PCS[BYP_MODE].iValue;

	case 2: // sw ver
		return VER_VERSION_MODBUS;
	case 3: // param ver
		return PARA_VERSION_MODBUS;
	case 4:
		return counter++;
	case 5:
		return PRM_PCS[WH_CHG_HI].iValue;
	case 6:
		return PRM_PCS[WH_CHG_LO].iValue;
	case 7:
		return PRM_PCS[WH_DCHG_HI].iValue;
	case 8:
		return PRM_PCS[WH_DCHG_LO].iValue;
		// 150122
	case 9: return debug_rx_cnt;
	case 10: return debug_tx_cnt;
	case 11: return BATCTRL.bmsInfo.timeOutCnt_sec;
	case 12: /*For POSCO recloser status*/
		return 0;
	case 13: /*For POSCO recloser ON/OFF request */return RecloserSwitchCmd;
	case 14: return (unsigned int)(( INVCTRL.fltFreq.fOut /*CC_GetFreq(&CONTHETA)*/ ) * 1000.); // Fixed 170410 (필터 값을 모니터링 하기로 함)
	case 15: return (unsigned int)(( EXCTRL.fltFreq.fOut /*CC_GetFreq(&GRIDTHETA)*/ ) * 1000.); // Fixed 170410 (필터 값을 모니터링 하기로 함)
	case 16: /* bypass Va */
		return (int)(GRID_ARG.MEASURE_BYP[0].fPhaseVoltage * 17.32 + 0.5f); // pcc 전압
	case 17: /* bypass Vb */
		return (int)(GRID_ARG.MEASURE_BYP[1].fPhaseVoltage * 17.32 + 0.5f); // pcc 전압
	case 18: /* bypass Vc */
		return (int)(GRID_ARG.MEASURE_BYP[2].fPhaseVoltage * 17.32 + 0.5f); // pcc 전압
	case 19:
			return CTRL_BYP_PCC_SyncOK();
	case 20: /* 나중에 추가 됨 */
			return (int)(BATCTRL.fltDCBattV.fOut * 10 + 0.5);
		//break;
	case 21:
	{
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
			return (int)(BATCTRL.fltI.fOut * 10 + 0.5);
		//else
		//	return 0;
	}
	case 22:
		//if(MCCB_MIRROR.Status.BitVal.bDS1 || MCCB_MIRROR.Status.BitVal.bMC1A_ONOFF)
			return (int)(BATCTRL.fltDCLinkV.fOut * 10 + 0.5);
		//else
		//	return 0;
	case 23:
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
			return (int)(BATCTRL.fPowerFilterd_1st * 0.01 + 0.5);
		//else
		//	return 0;
	case 24 : return 0;

	case 25: return (int)(GRID_ARG.MEASURE[0].fCurrent * 10 + 0.5f); // tr 전류
	case 26: return (int)(GRID_ARG.MEASURE[1].fCurrent * 10 + 0.5f); // tr 전류
	case 27: return (int)(GRID_ARG.MEASURE[2].fCurrent * 10 + 0.5f); // tr 전류


	case 29:
		if( (INVERTER.MEASURE[0].fPhaseVoltage * 17.32 + 0.5) < 20 )
			return 0;
		else
			return (int)(INVERTER.MEASURE[0].fPhaseVoltage * 17.32 + 0.5);
	case 30:
		if( (INVERTER.MEASURE[1].fPhaseVoltage * 17.32 + 0.5) < 20 )
			return 0;
		else
			return (int)(INVERTER.MEASURE[1].fPhaseVoltage * 17.32 + 0.5);
	case 31:
		if( (INVERTER.MEASURE[2].fPhaseVoltage * 17.32 + 0.5) < 20 )
			return 0;
		else
			return (int)(INVERTER.MEASURE[2].fPhaseVoltage * 17.32 + 0.5);

#if 0
	case 32:
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
			return (int)(INVERTER.MEASURE[0].fCurrent * 10 + 0.5);
		//else
		//	return 0;
	case 33:
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
			return (int)(INVERTER.MEASURE[1].fCurrent * 10 + 0.5);
		//else
		//	return 0;
	case 34:
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
			return (int)(INVERTER.MEASURE[2].fCurrent * 10 + 0.5);
		//else
		//	return 0;
#else

	case 32:
				return (int)(INVERTER.MEASURE[0].fCurrent * 10 + 0.5);
	case 33:
				return (int)(INVERTER.MEASURE[1].fCurrent * 10 + 0.5);
	case 34:
				return (int)(INVERTER.MEASURE[2].fCurrent * 10 + 0.5);
#endif
	case 35: return (int)(CC_GetFreq(&CONTHETA) * 10. + 0.5);
	case 36:
		return (BUTTON.bDerating << 8) | INVERTER.uStatus ;
	case 37:
		/* STABLEEN은 CB3가 없기 때문에 */
//		if( /*MCCB_MIRROR.Status.BitVal.bCB2E && */ (MCCB_MIRROR.Status.BitVal.bCB2 || GPIO_GetStaticSwitchOn()) )
			return (int)(GRID_ARG.MEASURE[0].fPhaseVoltage * 17.32 + 0.5f); // pcc 전압
/*		else
			return 0;*/
	case 38:
//		if( /*MCCB_MIRROR.Status.BitVal.bCB2E && */ (MCCB_MIRROR.Status.BitVal.bCB2 || GPIO_GetStaticSwitchOn()) )
			return (int)(GRID_ARG.MEASURE[1].fPhaseVoltage * 17.32 + 0.5f); // pcc 전압
/*		else
			return 0;*/
	case 39:
//		if( /*MCCB_MIRROR.Status.BitVal.bCB2E && */ (MCCB_MIRROR.Status.BitVal.bCB2 || GPIO_GetStaticSwitchOn()) )
			return (int)(GRID_ARG.MEASURE[2].fPhaseVoltage * 17.32 + 0.5f); // pcc 전압
/*		else
			return 0;*/
	case 40:
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
			return EXCTRL.fltFsBypI[0].fOut * 10. + 0.5;
		//else
		//	return 0;
	case 41:
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
			return EXCTRL.fltFsBypI[1].fOut * 10. + 0.5;
		//else
		//	return 0;

	case 42:
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
			return EXCTRL.fltFsBypI[2].fOut * 10. + 0.5;
		//else
		//	return 0;
	case 43: /* reserved */
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
			return (int)(GRID_ARG.fTotalPower * 0.01 + 0.5f);
		//else
		//	return 0;
	case 44:
		//if(MCCB_MIRROR.Status.BitVal.bExMCCB1 )
			return (int)(CC_GetFreq(&GRIDTHETA) * 10. + 0.5f);
		//else
			//return 0;
	case 45:
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
			return (int)(CC_GetActualPowerP_2nd() * 0.01 + 0.5f);
		//else
		//	return 0;
			//return (Int32)(GRID_ARG.fTotalPower * 0.01 + 0.5f); // P
	case 46:
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
        return 1210.0; //by JCNEET
			return (int)(CC_GetActualPowerQ_2nd() * 0.01 + 0.5f);
		//else
		//	return 0;
		//return PRM_PCS[BATT_REMOTE_PCC_P_REF].iValue;
	case 47:
		//if(MCCB_MIRROR.Status.BitVal.bMC4)
        return 1220.0; //by JCNEET
			return (int)(CC_GetPowerFactor() * 1000.0);
		//else
		//	return 0;
	case 48: return PRM_PCS[BATT_ORDER_SOURCE].iValue; // temp2
	case 49: // Status A
		return MF_getStatusInfo_A();
	case 50: // Status B
		return MF_getStatusInfo_B();
	case 51: //OP MODE 	0:NONE 1:Bypass 2:Charge 3:Discharge 4:Ready
		if( INVERTER.uStatus != SYS_INV_RUN )
		{
			//+150122
			if( INVERTER.uStatus == SYS_INV_ISLANDING || INVERTER.uStatus == SYS_INV_RE_SYNC )
			{
				if( BATCTRL.fltI2.fOut > 5)
				{
					// 충전
					return 2;
				}
				else if( BATCTRL.fltI2.fOut < -5)
				{
					// 방전
					return 3;
				}
				else
				{
					return 4;
				}
			}
			else
				return 0;
		}

		if( INVERTER.uStatus == SYS_INV_RUN && CVC_GetStatus() == CVC_DISABLE )
		{
			return 4;
		}

		if( BCState.powerFlow == PFLOW_STANDBY )
			return 4;
		else if( BCState.powerFlow == PFLOW_CHARGE )
			return 2;
		else if( BCState.powerFlow == PFLOW_DISCHARGE )
			return 3;
		else
			return 0;
	case 52:
		temp = 0;
		for(i =0; i<PEBB_COUNT_MAX; i++)
		{
			if( MO_GetStatus(i) == 1 )
				temp |= 1<<i;
		}
		return temp;
	case 53:
		return 0;
	case 54: return (uFault[0] & 0xFFFF );
	case 55: return (uFault[0] >> 16) & 0xFFFF;
	case 56: return (uFault[1] & 0xFFFF );
	case 57: return (uFault[1] >> 16) & 0xFFFF;
	case 58: return (uFault[2] & 0xFFFF );
	case 59: return (uFault[2] >> 16) & 0xFFFF;
	case 60: return (uFault[3] & 0xFFFF );
	case 61: return (uFault[3] >> 16) & 0xFFFF;
	case 62: // PEBB INFO
	case 63:
	case 64:
	case 65:
	case 66:
	case 67:
	case 68:
	case 69:
		temp = offset - 62;
		if( temp >=PEBB_COUNT_MAX )
		{
			return 0;
		}
		return (0x2FE1 & g_gbiRxData[temp].rxM0.STATE.byte16);
	case 70: return INVERTER.uStatus;
	case 71:
		return iCabTemp;
	case 72:case 73:case 74:case 75:case 76:case 77:case 78:case 79: // Heat Sync Temp
		temp = offset - 72;
		if( temp >=PEBB_COUNT_MAX )
		{
			return 0;
		}
		return g_gbiRxData[temp].rxM4.tempHeatSync;
	case 80: /* HMI-PMS간 통신 불량 처리용으로 사용 함. 사용 금지. */ return 0;
	case 81:
		return (int)CTRL_MODE_GetGC_GI_Mode();
	case 82:
		return (int)CTRL_MODE_GetCurrentSetMode();
	case 83: // TODO PEBB Fault Recovery Mode
		return PRM_PCS[BATT_PEBB_FAULT_MODE].iValue;
	case 84: // CP-P
		return CC_GetPowerRefP() / 100.;
	case 85:
		return PRM_PCS[BATT_REMOTE_PCC_Q_REF].iValue; //* 161220 무효보상분(100kW)이 포함되어 레퍼런스 자체를 리턴하도록 수정.(배포 펌웨어 중 100kW에만 먼저 적용) CC_GetPowerRefQ() / 100.;
	case 86:
		return INVERTER.gridReconnect.bEnb;
	case 87:
		return INVERTER.gridReconnect.uiProgressionSecond;
	case 88:return g_gbiRxData[0].rxM3.tempS;
	case 89:return g_gbiRxData[0].rxM3.tempT; /* 150114 Fixed 88->96 */
	case 90:return g_gbiRxData[0].rxM3.tempR;
	case 91:
		if( TRACE.iStop == 1 )
		{
			return 2;
		}
		else
		{
			return 1;
		}

	case 92:
	case 93:
		break;

	//	EVENT1
	case 94:	return SAG_OP.Sag_data_Shadow.YearMonth;
	case 95: 	return SAG_OP.Sag_data_Shadow.DateHour;
	case 96: 	return SAG_OP.Sag_data_Shadow.MinSec;
	case 97: 	return ((int)(SAG_OP.Sag_data_Shadow.VoltLV * 10 + 0.5f));
	case 98: 	return SAG_OP.Sag_data_Shadow.CompT;
	case 99: 	return SAG_OP.Sag_data_Shadow.bRestore;
	case 100: 	return 0; //SAG_OP.Sag_data_Shadow.Trace;
	case 101:	return SAG_OP.Sag_data_ready_flag; //SAG 데이터 존재 유무 확인 용
	case 102:   return TRACE.fStopTimeRatio;
	case 103:   return ((int)(SAG_OP.Sag_data_Shadow.VoltR * 10 + 0.5f));
	// 104번지 이상함
	case 105:   return ((int)(SAG_OP.Sag_data_Shadow.VoltS * 10 + 0.5f));
	case 106:   return ((int)(SAG_OP.Sag_data_Shadow.VoltT * 10 + 0.5f));
	case 107:	return ((int)((SAG_OP.Sag_data_Shadow.CompVoltR + SAG_OP.Sag_data_Shadow.CompVoltS + SAG_OP.Sag_data_Shadow.CompVoltT) * INV_3 * 10 + 0.5f));
	case 108:	return ((int)(SAG_OP.Sag_data_Shadow.CompVoltR * 10 + 0.5f));
	case 109:	return ((int)(SAG_OP.Sag_data_Shadow.CompVoltS * 10 + 0.5f));
	case 110:	return ((int)(SAG_OP.Sag_data_Shadow.CompVoltT * 10 + 0.5f));

	//	EVENT2
//	case 139: 	return SAG_EVT_TABLE[SECOND].YearMonth;
//	case 140: 	return SAG_EVT_TABLE[SECOND].DateHour;
//	case 141: 	return SAG_EVT_TABLE[SECOND].MinSec;
//	case 142: 	return ((int)(SAG_EVT_TABLE[SECOND].VoltLV * 10 + 0.5f));
//	case 144: 	return SAG_EVT_TABLE[SECOND].CompT;
//	case 145: 	return SAG_EVT_TABLE[SECOND].bRestore;
//	case 146: 	return SAG_EVT_TABLE[SECOND].Trace;
//	//	EVENT3
//	case 147: 	return SAG_EVT_TABLE[THIRD].YearMonth;
//	case 148: 	return SAG_EVT_TABLE[THIRD].DateHour;
//	case 149: 	return SAG_EVT_TABLE[THIRD].MinSec;
///*	case 150: 	return ((int)(SAG_EVT_TABLE[THIRD].VoltLV * 10 + 0.5f) & 0xFFFF);
//	case 151: 	return ((int)(SAG_EVT_TABLE[THIRD].VoltLV * 10 + 0.5f) >> 16) & 0xFFFF;*/
//	case 150: 	return ((int)(SAG_EVT_TABLE[THIRD].VoltLV * 10 + 0.5f));
//	//case 151: 	return ((SAG_EVT_TABLE[THIRD].VoltLV) >> 16) & 0xFFFF;
//	case 152: 	return SAG_EVT_TABLE[THIRD].CompT;
//	case 153: 	return SAG_EVT_TABLE[THIRD].bRestore;
//	case 154: 	return SAG_EVT_TABLE[THIRD].Trace;

	//DDI 1~8
	case 150: return MCCB_MIRROR.Status.BitVal.bMC1; //bDS1;
	case 151: return MCCB_MIRROR.Status.BitVal.bMC1A; //bCB2;
	case 152: return MCCB_MIRROR.Status.BitVal.bInductor; //bSMPS;
	case 153: return MCCB_MIRROR.Status.BitVal.bFuseRec; //bSPD1;
	case 154: return MCCB_MIRROR.Status.BitVal.bPowerFault; //bEPO;
	case 155: return MCCB_MIRROR.Status.BitVal.bFuseInv; //bSSW_OT;
	case 156: return MCCB_MIRROR.Status.BitVal.bDoor; // bMC1A;
	case 157: return MCCB_MIRROR.Status.BitVal.bMC2; // bGFD;
	//ADI 1~16
	case 158: return MCCB_MIRROR.Status.BitVal.bSSWOH; // bSPD4;
	case 159: return MCCB_MIRROR.Status.BitVal.bREV1; // bCB3;
	case 160: return MCCB_MIRROR.Status.BitVal.bREV2; // bExMCCB1;
	case 161: return MCCB_MIRROR.Status.BitVal.bCB1; // bExMCCB2;
	case 162: return MCCB_MIRROR.Status.BitVal.bCB2; // bExMCCB3;
	case 163: return MCCB_MIRROR.Status.BitVal.bCB3; // bSSWFUSE;
	case 164: return MCCB_MIRROR.Status.BitVal.bREV3; // bMCB7;
	case 165: return MCCB_MIRROR.Status.BitVal.bREV4; //bRETEMP;
#if 0 //by JCNET
	case 166: return MCCB_MIRROR.Status.BitVal.bTRTEMP;
	case 167: return MCCB_MIRROR.Status.BitVal.bCB4;
	case 168: return MCCB_MIRROR.Status.BitVal.bMC8;
	case 169: return MCCB_MIRROR.Status.BitVal.bMC9;
	case 170: return MCCB_MIRROR.Status.BitVal.bDoor;
	case 171: return MCCB_MIRROR.Status.BitVal.bADI_REV14;
	case 172: return MCCB_MIRROR.Status.BitVal.bADI_REV15;
	case 173: return MCCB_MIRROR.Status.BitVal.bADI_REV16;
#endif
	case 197: return SOURCE_CODE_BASE;
	case 198: return BUILD_DATE_YYYY;
	case 199: return BUILD_DATE_MMDD;
	default :
		break;
	}
	return 0;
}

// for Real Time Data
float lValue;
Uint16* pValue;
int MT_getRealTimeData(int ID)
{
	int iInfoID;
	iInfoID = ID / 2;

	if( (ID % 2) != 0 ) // 두번째 요청일 경우 기존에 가져간 부분을 제외한 부분을 리턴. 두번째 값 가져가기전 갱신된 값이 있을 경우 float오류가 발생하므로.
	{
		return (pValue[0]) & 0xFFFF;
	}

	if ( iInfoID >= TRC_NUM ) return 0;
	if(INFO[iInfoID].bInUsed==0)
		return 0;



	switch( INFO[iInfoID].iItemDataType )
	{
		case TRC_FLOAT:
			lValue = *(float *)(INFO[iInfoID].pItemAddr);
			break;
		case TRC_INT:
			lValue = (float)(*(int *)(INFO[iInfoID].pItemAddr));
			break;
		case TRC_UNS:
			lValue = (float)(*(Uns *)(INFO[iInfoID].pItemAddr));
			break;
		default:
			lValue = 0.;
	}

	pValue = ((Uint16*)&lValue);
	if( (ID % 2) == 0 )
		return (pValue[1] ) & 0xFFFF;
	else
		return (pValue[0]) & 0xFFFF;


}

Uint16 MF_getStatusInfo_A()
{
	Uns count;
	Uint16 mask;
	Uint16 retVal;
	retVal = 0;

	for(count=0;count<16;count++)
	{
		mask= ((Uint16)1<<count);
		switch(count)
		{
		case 0:	//	DC Charger ON/OFF
			if(bDcChargerOnOff)
				retVal|=mask;
			break;
		case 1: // Heavy Fault 유무
			if(FLT_GetHeavyStatus())
				retVal|=mask;
			break;

		case 2: // CB1(BATT CB) ON/OFF
#if 0 //by JCNET
			if(MCCB_MIRROR.Status.BitVal.bDS1)
#else
	        if(MCCB_MIRROR.Status.BitVal.bMC1)
#endif
				retVal|=mask;
			break;
		case 3: // CB1A(초기 충전) ON/OFF
			if(MCCB_MIRROR.Status.BitVal.bMC1A)
				retVal|=mask;
			break;
		case 4: // MCB2 = DDI2
//by JCNET			if(MCCB_MIRROR.Status.BitVal.bCB2)
		    if(MCCB_MIRROR.Status.BitVal.bMC2)
				retVal|=mask;
			break;
#if 0 //by JCNET
		case 5: // CB3 ( Point of common coupling )
			if(MCCB_MIRROR.Status.BitVal.bCB3)
				retVal|=mask;
			break;
		case 6: // CB2 ( GRID or LOAD CB - 수동 스위치 )
			if(MCCB_MIRROR.Status.BitVal.bExMCCB1)
				retVal|=mask;
			break;
		case 7: // CB1 ( GRID ES-LV Type)
			if(MCCB_MIRROR.Status.BitVal.bCB4)
				retVal|=mask;
#else
        case 5: // CB1 ( Point of common coupling )
            if(MCCB_MIRROR.Status.BitVal.bCB1)
                retVal|=mask;
            break;
        case 6: // CB2 ( GRID or LOAD CB - 수동 스위치 )
            if(MCCB_MIRROR.Status.BitVal.bCB2)
                retVal|=mask;
            break;
        case 7: // CB3 ( GRID ES-LV Type)
            if(MCCB_MIRROR.Status.BitVal.bCB3)
                retVal|=mask;
#endif
			break;
		case 8:
			// REMOTE-LOCAL
			break;
		case 9:
			if(FLT_BattVoltageFailure())
				retVal|=mask;
			break;
		case 10:
//+ 150316 June --> Posco에서 Fault history를 남기므로, 처리하지 않도록 함.
			if( FLT_ConVoltageFailure() )
				retVal|=mask;
			break;
#if 0 //- 150316
			if ( CVC_GetStatus() != CVC_DISABLE )
				retVal += mask;
			break;
#endif
		case 11:
			if(FLT_GridVoltageFailure())
				retVal|=mask;
			break;
		case 12:
			if(GPIO_GetStaticSwitch())
				retVal |= mask;
			break;
		case 13:
			if(FLT_GetEachStatus(FLTH_EPO))
				retVal|=mask;
			break;
		case 14:
			if( INVERTER.bDcChargeOK )
				retVal|=mask;
			break;
		case 15:
			break;
		}
	}
	return retVal;
}
Uint16 MF_getStatusInfo_B()
{
	Uns count;
	Uint16 mask;
	Uint16 retVal;
	retVal = 0;

#if 0 //by JCNET
	for(count=0;count<16;count++)
	{
		mask= ((Uint16)1<<count);
		switch(count)
		{
		case 0:
			if( BUTTON.bPwmOnOff )
				retVal|=mask;
			break;
		case 1: /* MCU-PMS Communication Error. MCU는 사용 X */
			break;
		case 2:
			if(MCCB_MIRROR.Status.BitVal.bExMCCB2)
							retVal|=mask;
			break;
		case 3:
			if(MCCB_MIRROR.Status.BitVal.bExMCCB3)
							retVal|=mask;
			break;
		case 4:
			if(MCCB_MIRROR.Status.BitVal.bMCB7)
							retVal|=mask;
			break;
		case 5:
			if(MCCB_MIRROR.Status.BitVal.bMC8)
							retVal|=mask;
			break;
		case 6:
			if(MCCB_MIRROR.Status.BitVal.bMC9)
							retVal|=mask;
			break;
		case 7: if( EVE.bStatus ) retVal|=mask;
			break;
		case 8: if( EVE.bFaultStatus ) retVal|=mask;
			break;
		case 9: if( EVE.bInitCharge ) retVal|=mask;
			break;
		case 10:
			break;
		case 11:
			break;
		case 12:/* 12~13 FR PMS 사용 */
		case 13:
			break;
		case 14: // GATE(PWM) ON/OFF
			if ( CVC_GetStatus() != CVC_DISABLE )
				retVal|=mask;
			break;
		case 15:// SYSTEM ON/OFF
			if( SYS_CheckInvCmdOn() )
				retVal|=mask;
			break;
		}
	}
#else

#endif
	return retVal;
}

int MF_TestCmd(UInt16 val)
{
	int iTemp;

	if( (PRM_PCS[SYS_MO_ENABLE].iValue == 0) || (PRM_PCS[CTRL_TEST_MODE].iValue != 0) ) // Test Mode
	{
		switch( val )
		{
		case 21:	case 22:	case 23:	case 24:	case 25:
		case 26:	case 27:	case 28:	case 29:	case 30:
		case 31:	case 32:
			//MC1~12(Logos는 8이 MAX) MC ON
			iTemp = val - 21;
			MO_PebbCtrl(iTemp, 1);
			break;
		case 35:	case 36:	case 37:	case 38:	case 39:
		case 40:	case 41:	case 42:	case 43:	case 44:
		case 45:	case 46:
			//MC1~12(Logos는 8이 MAX) MC OFF
			iTemp = val - 35;
			MO_PebbCtrl(iTemp, 0);
			break;
		}
	}

	if(PRM_PCS[CTRL_TEST_MODE].iValue != 1 && PRM_PCS[CTRL_TEST_MODE].iValue != 2 ) // Test Mode
	{
		return 0;
	}

	//FIXED
	switch( val )
	{
	case 1: /* LOGOS CB1A */
		MC_UpdateStatusForTest(CTRL_MC1A_ONOFF, TRUE);
		break;
	case 2:
		MC_UpdateStatusForTest(CTRL_MC1A_ONOFF, FALSE);
		break;
	case 3:
#if 0 //by JCNET
		if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			MCB_UpdateCmd(M_CB1_BATT, CB_CMD_ON);
		else
		{
			/*
			 * 배터리 전압 보이는 상태에서 DC 메인 차단기 오조작 할 수 있기 때문에 사고 방지용
			 */
			if( (BATCTRL.fltDCBattV_2nd.fOut <= 50.)
					|| ((BATCTRL.fltDCBattV_2nd.fOut > 50.) && MC_GetStatus(STATUS_MC1A2A)) )
				MC_UpdateStatusForTest(CTRL_DS1_ON, TRUE);
		}
#endif
		break;
	case 4:
		if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			MCB_UpdateCmd(M_CB1_BATT, CB_CMD_OFF);
		else
#if 0 //by JCNET
			MC_UpdateStatusForTest(CTRL_DS1_ON, FALSE);
#else
            MC_UpdateStatusForTest(CTRL_MC1_ON, FALSE);
#endif
		break;
	case 5:
		MCB_UpdateCmd(M_CB1_BATT, CB_CMD_TRIP);
		break;
	case 6:
		MCB_UpdateCmd(M_CB1_BATT, CB_CMD_RESET);
		break;
	case 7: /* LOGOS MC5 -- LOGOS로 안되었음 확인필요. 205 Command로 함.*/
#if 0 //by JCNET
		MC_UpdateStatusForTest(CTRL_SCFAN_CTRL, TRUE);
#endif
		break;
	case 8:
//by JCNET 		MC_UpdateStatusForTest(CTRL_SCFAN_CTRL, FALSE);
		break;
#if 0 // by JCNET
	case 9: /* LOGOS MC4 */
		if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			MCB_UpdateCmd(M_CB4_SSW, CB_CMD_ON);
		else
			MC_UpdateStatusForTest(CTRL_CB4_ON, TRUE);
		break;

	case 10:
		if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);
		else
			MC_UpdateStatusForTest(CTRL_CB4_ON, FALSE);
		break;
#endif
	case 11: /* LOGOS CB2 */
		//TODO CHECK PCC.v_pk?
		// 그리드 전압이 살아 있으면 TRIP 발생
#if 0 //by JCNET
		if( ACP.PCC.v_pk <=50 )
		{
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
				MCB_UpdateCmd(M_CB3, CB_CMD_ON);
			else
				MC_UpdateStatusForTest(CTRL_CB3_ON, TRUE);
		}
#endif
		break;
	case 12:
#if 0
		if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			MCB_UpdateCmd(M_CB3, CB_CMD_OFF);
		else
			MC_UpdateStatusForTest(CTRL_CB3_ON, FALSE);
#endif //by JCNET
		break;
	case 13:
		MCB_UpdateCmd(M_CB3, CB_CMD_TRIP);
		break;
	case 14:
		MCB_UpdateCmd(M_CB3, CB_CMD_RESET);
		break;
	case 15:
		SYS_SetTestRunCommand(TRUE);
		break;
	case 16:
		SYS_SetTestRunCommand(FALSE);
		break;
#if 0 //by JCNET
	case 17:
		MC_UpdateStatusForTest(CTRL_MC8_ONOFF, TRUE);
		break;
	case 18:
		MC_UpdateStatusForTest(CTRL_MC8_ONOFF, FALSE);
		break;
	case 19:
		MC_UpdateStatusForTest(CTRL_MC9_ONOFF, TRUE);
		break;
	case 20:
		MC_UpdateStatusForTest(CTRL_MC9_ONOFF, FALSE);
		break;
#endif
	case 21:	case 22:	case 23:	case 24:	case 25:
	case 26:	case 27:	case 28:	case 29:	case 30:
	case 31:	case 32:
		//MC1~12(Logos는 8이 MAX) MC ON
		iTemp = val - 21;
		MO_PebbCtrl(iTemp, 1);
		break;
	case 35:	case 36:	case 37:	case 38:	case 39:
	case 40:	case 41:	case 42:	case 43:	case 44:
	case 45:	case 46:
		//MC1~12(Logos는 8이 MAX) MC OFF
		iTemp = val - 35;
		MO_PebbCtrl(iTemp, 0);
		break;
	case 50:
		CAN_GBI_ManualOperation(TRUE);
		break;
	case 51:
		CAN_GBI_ManualOperation(FALSE);
		break;
	case 60:
		//MC_UpdateStatusForTest(CTRL_SSEFAN_CTRL, TRUE);
		break;
	case 61:
		//MC_UpdateStatusForTest(CTRL_SSEFAN_CTRL, FALSE);
		break;
	case 62:
		GPIO_StaticSwitch(GPIO_STATIC_SW_ON);
		break;
	case 63:
		GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
		break;
	case 101:
	case 102:
	case 103:
	case 104:
	case 105:
	case 106:
	case 107:
	case 108:
	case 109:
	case 110:
	case 111:
	case 112:
	case 114:
	case 115:
        DIO_setStatus(val-101+1,0);
//by JCNET		MC_UpdateStatusForTest((CBMC_CTRL)(val-101),FALSE);
		break;
	case 201:
	case 202:
	case 203:
	case 204:
	case 205:
	case 206:
	case 207:
	case 208:
	case 209:
	case 210:
	case 211:
	case 212:
	case 213:
	case 214:
	    DIO_setStatus(val-201+1,1);
//		MC_UpdateStatusForTest((CBMC_CTRL)(val-201),TRUE);
		break;

	default:
		return 0;
	}

	return 0;
}
//by JCNET
Bool DONAU_is_active_all();
void DONAU_SetAllCommand(Bool onOff);
//
void MF_PCS_Command(int wReg)
{
	// 포스코의 요청으로 lock 기능 사용 안함.
	//-if( bPcsCommandLocked )
	//-	return -1;

	// ess operation 1: on 2: off
 //by JCNET
    if(wReg & 0x80)
    {
        extern void DONAU_SetEachState(Uns wReg);
        DONAU_SetEachState(wReg&0x7);
        return;
    }
	switch( wReg )
	{
		case 1:
		{
	#if 0
			if( BUTTON.bInverter && INVERTER.bDcChargeOK )
			{
				// 이미 시스템 시작중이고, Charge 완료 상태이면 Start.
				BUTTON.bStart = TRUE;
			}
			else
	#endif
//by JCNET for TEST
//
			if(!DONAU_is_active_all())
			{
                TRC_TraceReset();
			    DONAU_SetAllCommand(ON);
			    break;
			}

			if( !SYS_CheckInvCmdOn() && !FLT_GetHeavyStatus() && !INVERTER.gridReconnect.bEnb )
			{
				EVT_Store(EVT_ON);
				FLT_UpdateAutoResetInit();
				SYS_SetInverterCommand(ON);
				TRC_TraceReset();
			}
			//TODO 하단 코드 삭제 가능 여부 확인. GI 모드에서는 gridReconnect.bEnb가 해지 됨.
			else if( !SYS_CheckInvCmdOn() && !FLT_GetHeavyStatus() && PRM_PCS[GC_IS_MODE].iValue == 1 )
			{
				EVT_Store(EVT_ON);
				FLT_UpdateAutoResetInit();
				SYS_SetInverterCommand(ON);
				TRC_TraceReset();
			}
			else
			{
	#if DBUG_MODE != 0
	// HILL 모드에서는 ON 컨맨드 내리면 기동 시킨다.
				if( INVERTER.gridReconnect.bEnb )
				{
					INVERTER.gridReconnect.bEnb = FALSE;
					FLT_SetGridFailureEvent(FALSE);
				}

	#endif
			}
		}
		break;
		case 2:
		{
			if( (PRM_PCS[SYS_OPTION].iValue & 0x8) /* START-STOP 사용*/ )
			{
				if( BUTTON.bInverter && BUTTON.bStart == FALSE)
				{
					EVT_Store(EVT_OFF);
					SYS_SetInverterCommand(OFF);
					FLT_UpdateAutoResetInit();
				}
			}
			else
			{
				if( BUTTON.bInverter )
				{
					BUTTON.bStart = FALSE;
					EVT_Store(EVT_OFF);
					SYS_SetInverterCommand(OFF);
					FLT_UpdateAutoResetInit();
				}
			}

		}
		break;
		case 3:
		{
			if( FLT_GetStatus() != 0 )
			{
				EVT_Store(EVT_FLT_RESET);
				FLT_Initialize();
				EVE.bCMD_FaultReset = TRUE;
			}
		}
		break;
		case 4:
		{
			if (INVERTER.uStatus == SYS_INV_STOP || INVERTER.uStatus == SYS_INV_FAULT || INVERTER.uStatus == SYS_INV_TEST_MODE)
			{
				if( PRM_PCS[CTRL_TEST_MODE].iValue == 5 )
				{
					SYS_SetInverterCommand(OFF);
					EVT_Store(EVT_PARA_INIT);
					PARA_Initialize();
					//SYSTEM OFF!!
					Task_sleep(1000);//sleep some
					RebootSystem();
				}
			}
		}
		break;
		case 5:
		{
			if( PRM_PCS[CTRL_TEST_MODE].iValue == 5 )
			{
				EVT_Store(EVT_HISTORY_INIT);
				EVT_ClearCache();
			}
		}
		break;
		case 6:
		{
			TRC_StopTrace();
		}
		break;
		case 7:
		{
			TRC_TraceReset();
		}
		break;
		case 8:
		{
			//SYSTEM OFF!!
			SYS_SetInverterCommand(OFF);
			Task_sleep(10000);//Some Sleep

			//Watchdog reset
			RebootSystem();
		}
		break;
		case 9:
		{
			if (INVERTER.uStatus == SYS_INV_STOP || INVERTER.uStatus == SYS_INV_FAULT)
				PRM_PCS[CTRL_TEST_MODE].iValue = 1;
		}
		break;
		case 10:
		{
			PRM_PCS[CTRL_TEST_MODE].iValue = 0;
		}
		break;
		case 11:
		{
			if( INVERTER.bDcChargeOK )
				BUTTON.bStart = TRUE;
		}
		break;
		case 12:
		{
			//if( BUTTON.bInverter )
				BUTTON.bStart = FALSE;
		}
		break;
		case 13:
		{
			BUTTON.bPwmOnOff = ON;
		}
		break;
		case 14:
		{
			//-160406 if( BCState.powerFlow == PFLOW_STANDBY )
				BUTTON.bPwmOnOff = OFF;
		}
		break;
		case 15:
		{
			SAG_OP.Sag_data_ready_flag = FALSE; // Sag_data_ready_flag clear by HMI
			//Dequeue
		}
		break;
		case 16:
		{
		}
		break;
		case 17:
		{
		}
		break;
		case 18:
		{
		}
		break;
		case 90:
		{
			BUTTON.bDerating = TRUE;
		}
		break;
		case 91:
		{
			BUTTON.bDerating = FALSE;
		}
		break;

		default:

		{
			bPcsCommandLocked = 1;
		}
		break;
	}
	bPcsCommandLocked = 1;
}
