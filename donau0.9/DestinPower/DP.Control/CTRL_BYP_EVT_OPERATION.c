#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>
#include "CTRL_BYP_EVT_OPERATION.h"
#include "SYSTEM.h"
#include "CC.h"
#include "CTRL_GEN.h"
//#include "SagEventHistory.h"
//#include "CTRL_INV_SEAMLESS.h"

#include "GPIO.h"
#include "Fault.h"
#include "Odt.h"
#include "EVENT.h"
#include "FastRMS.h"
#include "CTRL_BYP_SYNC.h"
#include "LGS_Common.h"
#include "MathConst.h"
#include "PI.h"
#include "FILTER.h"
#include "PWM.h"
#include "RAMP.h"
#include "FastRms.h"
#include "EADC.h"
#include "CTRL_INV_PRC.h"
#include "CTRL_INV_DROOP.h"
#include "CTRL_INV_PQC.h"
#include "CTRL_INV_SYNC.h"
#include "CTRL_INV_VUC.h"
#include "CTRL_INV_VI.h"
#include "CTRL_INV_SEAMLESS.h"
#include "Prm_pcs.h"
#include "CTRL_GEN.h"
#include "SagEventHistory.h"

/*
#define PM_OVERLOAD_LV1_KVA	250e3	//	추후 파라미터로 변경 - 경고장
#define PM_OVERLOAD_LV2_KVA	500e3	//	추후 파라미터로 변경 - 중고장
#else if
#define PM_OVERLOAD_LV1_KVA	300e3	//	추후 파라미터로 변경 - 경고장
#define PM_OVERLOAD_LV2_KVA	600e3	//	추후 파라미터로 변경 - 중고장
*/


volatile float x/*, y, z*/;

Bool bByp_Evt_Operating = FALSE;
Bool bSSW_CB2_Disable = FALSE;
Bool bSSW_CB2_Disable_Init = FALSE;

Byp_Evt_State BYP_EVT_STATE;

#pragma CODE_SECTION (CTRL_BYP_EVT_OPERATION_Create, "dp_ctrl")
void CTRL_BYP_EVT_OPERATION_Create(void)
{
	Uns	uPeriod = 5;

	INVERTER.odtPMOverLoad = Odt_(&INVERTER.ODT_PM_OVERLOAD, PRM_PCS[PM_OVERLOAD_TIME].iValue, uPeriod);	//	Power Meter 과부하 LV1 경고장 시간 -> SSW Off, CB3E ON, PWM Off
	INVERTER.odtPMOverLoadClear = Odt_(&INVERTER.ODT_PM_OVERLOAD_CLEAR, PRM_PCS[PM_OVERLOAD_CLEAR_TIME].iValue, uPeriod);	//	Power Meter 과부하 LV1 경고장 해제 -> SSW On, CB3E Off, PWM On
	INVERTER.odtBypOverLoad = Odt_(&INVERTER.ODT_BYP_OVERLOAD, 5000, uPeriod);	//	105% Over Load 5sec
	INVERTER.odtBypOverLoadClear = Odt_(&INVERTER.ODT_BYP_OVERLOAD_CLEAR, 7000, uPeriod);	//	105% Over Load Clear 7sec
	INVERTER.odtSCROverLoad = Odt_(&INVERTER.ODT_SCR_OVERLOAD, 5000, uPeriod);	//
	INVERTER.odtSCROnDelay = Odt_(&INVERTER.ODT_SCR_ON_DELAY, 500, uPeriod);	//	CB3E가 ON 된 후 Delay 후 SCR OFF 500ms
	INVERTER.odtSCROffDelay = Odt_(&INVERTER.ODT_SCR_OFF_DELAY, 500, uPeriod);	//	CB3E가 ON 된 후 Delay 후 SCR OFF 500ms
	INVERTER.odtCB2OnDelay = Odt_(&INVERTER.ODT_CB2_ON_DELAY, 500, uPeriod);	//	SCR이 ON 된 후 Delay 후 CB3E ON 500ms
	INVERTER.odtCB2OffDelay = Odt_(&INVERTER.ODT_CB2_OFF_DELAY, 500, uPeriod);	//	SCR이 ON 된 후 Delay 후 CB3E OFF 500ms

	INVERTER.odtSSWCB2Disable = Odt_(&INVERTER.ODT_SSW_CB2_DISABLE, 500, uPeriod);	//	SCR OC 발생 시 SSW와 CB3E 동작 비활성화

	CTRL.INV.SEAMLESS.enb_for_BEO = TRUE;
}

#if 0 //[CO] by JCNET
#pragma CODE_SECTION (CTRL_BYP_EVT_OPERATION_Proceed, "dp_ctrl")
void CTRL_BYP_EVT_OPERATION_Proceed(void)
{
	x = 950.0 * 0.66;	// RMS VALUE /*임시*/	//	SCR 정격 전류 파라미터 추가, 정격 전류의 몇 % 과부하 파라미터 추가 ex) SCR_RATE_CURRENT * SCR_OVERLOAD_RATIO * 0.01

	if( INVERTER.uStatus == SYS_INV_RUN && GPIO_GetStaticSwitchOn() == OFF )
		CTRL.INV.SEAMLESS.enb_for_BEO = FALSE;	//  GI 전환 방지
	else if( INVERTER.uStatus == SYS_INV_RUN && GPIO_GetStaticSwitchOn() == ON)
		CTRL.INV.SEAMLESS.enb_for_BEO = TRUE;	//  GI 전환 활성화

	/* MG & STABLEEN 공용	*/
	// CUSTOMER LOAD Power Meter 피상전력 계산 S = root(P^2 + Q^2)
	GenBlock.SET_loadPowerMeterS =
		sqrt(( GenBlock.SET_loadPowerMeterP * GenBlock.SET_loadPowerMeterP ) + ( GenBlock.SET_loadPowerMeterQ * GenBlock.SET_loadPowerMeterQ ));

	if( (GenBlock.SET_loadPowerMeterS > PRM_PCS[PM_OVERLOAD_LV2_KVA].iValue * PRM_PCS[PM_OVERLOAD_LV2_KVA].fIncDec * 1000) )  // 부하 과부하(파워미터)
	{
		FLT_Raise(FLTH_PM_OVERLOAD);
		GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
		MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);
	}

	if( (GenBlock.SET_loadPowerMeterS > PRM_PCS[PM_OVERLOAD_LV1_KVA].iValue * PRM_PCS[PM_OVERLOAD_LV1_KVA].fIncDec * 1000
			&& GenBlock.SET_loadPowerMeterS < PRM_PCS[PM_OVERLOAD_LV2_KVA].iValue * PRM_PCS[PM_OVERLOAD_LV2_KVA].fIncDec * 1000) && INVERTER.uStatus != SYS_INV_FAULT)  // 부하 과부하(파워미터)
	{
		bByp_Evt_Operating = TRUE;	//	-> 과전류 이므로 충전 중지

		ODT_Initialize(INVERTER.odtPMOverLoadClear);
		ODT_Initialize(INVERTER.odtSCROnDelay);

		if( ODT_Update(INVERTER.odtPMOverLoad, TRUE) == ODT_FINISH )
		{
			FLT_Raise(FLTL_PM_OVERLOAD);

			if( !MC_GetStatus(STATUS_CB4) && CTRL_BYP_NormalOK() )
			{
				MCB_UpdateCmd(M_CB4_SSW, CB_CMD_ON);	// CB 반복 걸리는 부분
			}

			if( MC_GetStatus(STATUS_CB4) )	// CB3E ON 확인 될 때 SCR OFF
			{
				if(ODT_Update(INVERTER.odtSCROffDelay, TRUE) == ODT_FINISH)
					GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
			}
		}

		if(!CTRL_BYP_NormalOK())
		{
			MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);
			GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
		}
	}
	else if( (GenBlock.SET_loadPowerMeterS < PRM_PCS[PM_OVERLOAD_LV1_KVA].iValue * PRM_PCS[PM_OVERLOAD_LV1_KVA].fIncDec * 1000)
			&& !FLT_GetEachStatus(FLTL_SSW_OC) /* && bByp_Evt_Operating == TRUE */)
	{
		FLT_Clear(FLTL_PM_OVERLOAD);

		if(bByp_Evt_Operating == TRUE)
		{
			ODT_Initialize(INVERTER.odtPMOverLoad);
			ODT_Initialize(INVERTER.odtSCROffDelay);

			if( MC_GetStatus(STATUS_CB4) && ODT_Update(INVERTER.odtPMOverLoadClear, TRUE) == ODT_FINISH && CTRL_BYP_NormalOK())
			{
				if(ODT_Update(INVERTER.odtSCROnDelay, TRUE) == ODT_FINISH)
				{
					GPIO_StaticSwitch(GPIO_STATIC_SW_ON);
				}
			}

			if( GPIO_GetStaticSwitchOn() == ON && bByp_Evt_Operating == TRUE )
			{
				if( MC_GetStatus(STATUS_CB4) )
				{
					MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);

					FLT_Clear(FLTL_PM_OVERLOAD);
				}
			}

			//	Normal State
			if(!MC_GetStatus(STATUS_CB4) && GPIO_GetStaticSwitchOn() == ON)
			{
					bByp_Evt_Operating = FALSE;
			}
		}
	}

	if(bSSW_CB2_Disable == TRUE)
	{
		if(bSSW_CB2_Disable_Init == TRUE)
			ODT_Initialize(INVERTER.odtSSWCB2Disable);

		ODT_Update(INVERTER.odtSSWCB2Disable, TRUE);

		if(ODT_Update(INVERTER.odtSSWCB2Disable, TRUE) == ODT_FINISH)
		{
			bSSW_CB2_Disable = FALSE;
			FLT_Clear(FLTL_SSW_OC);

			ODT_Initialize(INVERTER.odtSSWCB2Disable);

			if(GPIO_GetStaticSwitchOn() == OFF)
				GPIO_StaticSwitch(GPIO_STATIC_SW_ON);
		}
	}
	/* MG & STABLEEN 공용	*/

#if 0
	/* 	SYS_INV_RUN
	 * 	SCR 측에 105%_OVERLOAD 발생 시
	 * 	SCR 측에 105% 이상 과전류 발생 & Xsec 초과 시 CB3E ON & SCR OFF
	 */
	if( /* GPIO_GetStaticSwitchOn() == ON && */
			INVERTER.uStatus == SYS_INV_RUN && /* PCC.uState == GPC_NOGATING && */ /* CTRL.INV.SEAMLESS.enb_for_BEO == TRUE && */
			( EXCTRL.fltFsBypI[0].fOut > x || EXCTRL.fltFsBypI[1].fOut > x || EXCTRL.fltFsBypI[2].fOut > x ) )
	{
		ODT_Initialize(INVERTER.odtBypOverLoadClear);
		//ODT_Initialize(INVERTER.odtCB2OffDelay);

		bByp_Evt_Operating = TRUE;	//	-> 과전류 이므로 충전 중지

		if( GPIO_GetStaticSwitchOn() == ON )
			ODT_Update(INVERTER.odtBypOverLoad, TRUE);

		if( ODT_Update(INVERTER.odtBypOverLoad, TRUE) == ODT_FINISH )
		{
			PCC.uState = GPC_NOGATING;

			EVT_Store_NoDup(BYP_OVERLOAD);

			if( !MC_GetStatus(STATUS_CB4) )
			{
				MCB_UpdateCmd(M_CB4_SSW, CB_CMD_ON);
			}

			ODT_Update(INVERTER.odtSCROffDelay, TRUE);

			if(ODT_Update(INVERTER.odtSCROffDelay, TRUE) == ODT_FINISH)
			{
				GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);

				ODT_Initialize(INVERTER.odtSCROffDelay);
			}
		}
	}	//	if( ( INVERTER.uStatus == SYS_INV_RUN ) &&	(ACP.BYP.ia > a || ACP.BYP.ia < b || ACP.BYP.ib > a || ACP.BYP.ib < b || ACP.BYP.ic > a || ACP.BYP.ic < b))
	else if( /* MC_GetStatus(STATUS_CB4) == ON && */
			INVERTER.uStatus == SYS_INV_RUN && /* PCC.uState == GPC_NOGATING && */ /* CTRL.INV.SEAMLESS.enb_for_BEO == FALSE && */
			( EXCTRL.fltFsBypI[0].fOut < x || EXCTRL.fltFsBypI[1].fOut < x || EXCTRL.fltFsBypI[2].fOut < x ) )
	{
		/* 정상부하 시 CNT 초기화	*/
		ODT_Initialize(INVERTER.odtBypOverLoad);
		//ODT_Initialize(INVERTER.odtSCROffDelay);

		if( MC_GetStatus(STATUS_CB4) )
			ODT_Update(INVERTER.odtBypOverLoadClear, TRUE);

		if( ODT_Update(INVERTER.odtBypOverLoadClear, TRUE) == ODT_FINISH )
		{
			GPIO_StaticSwitch(GPIO_STATIC_SW_ON);

			EVT_Store_NoDup(BYP_OVERLOAD_CLEAR);
		}

		if(MC_GetStatus(STATUS_CB4) == ON && GPIO_GetStaticSwitchOn() == ON)
			ODT_Update(INVERTER.odtCB2OffDelay, TRUE);

		if(ODT_Update(INVERTER.odtCB2OffDelay, TRUE) == ODT_FINISH)
		{
			if( MC_GetStatus(STATUS_CB4) )
			{
				MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);

				ODT_Initialize(INVERTER.odtCB2OffDelay);
			}
		}

		/*	Normal State	*/
		if(!MC_GetStatus(STATUS_CB4) && GPIO_GetStaticSwitchOn() == ON)
		{
			bByp_Evt_Operating = FALSE;

			PCC.uState == GPC_GATING;

			CTRL.INV.SEAMLESS.enb_for_BEO = TRUE;	//  GI 전환 활성화
		}
	}
#endif	//	#if 0



}
#endif

/*void CTRL_BYP_EVT_OPERATION_SCR_OC(void)
{
	//SCR 과전류 레벨
	float yy = 950.0 * sqrt(2) * 0.66;	//	PEAK VALUE
	float zz = -yy;	//	PEAK VALUE

	if( ( GPIO_GetStaticSwitchOn() == ON ) && CTRL.INV.SEAMLESS.enb_for_BEO == TRUE &&
			(ACP.BYP.Ia > yy || ACP.BYP.Ia < zz || ACP.BYP.Ib > yy || ACP.BYP.Ib < zz || ACP.BYP.Ic > yy || ACP.BYP.Ic < zz) ) //	PEAK 값으로 판단
	{
		CkeckOC(ACP.BYP.Ia, yy, zz);
		CkeckOC(ACP.BYP.Ib, yy, zz);
		CkeckOC(ACP.BYP.Ic, yy, zz);

		if (ACP.BYP.Ia_OC_Count >= BYP_OC_COUNT_MAX
				|| ACP.BYP.Ib_OC_Count >= BYP_OC_COUNT_MAX
				|| ACP.BYP.Ic_OC_Count >= BYP_OC_COUNT_MAX)	//	BYPASS 측 OC 판단
		{
			bByp_Evt_Operating = TRUE;

			CTRL.INV.SEAMLESS.enb_for_BEO = FALSE;	//  GI 전환 비활성화

			GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);

			if( !MC_GetStatus(STATUS_CB4) )
			{
				MCB_UpdateCmd(M_CB4_SSW, CB_CMD_ON);
			}
		}
	}
	else
	{
		INVCTRL.Ia_OC_Count = 0;
		INVCTRL.Ib_OC_Count = 0;
		INVCTRL.Ic_OC_Count = 0;
	}
}*/



// end
