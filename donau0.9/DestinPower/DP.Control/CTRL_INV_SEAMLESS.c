/*
 * CTRL_INV_SEAMLESS.c
 *
 *  Created on: 2014. 5. 26.
 *      Author: Seth Oh
 */

#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "CTRL_INV_SEAMLESS.h"
#include "CC.h"
#include "PI.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "filter.h"
#include "trace.h"
#include "FAULT.h"
#include "MCCB.h"
//#include "SYSTEM.h"
#include "Event.h"
//#include "SagEventHistory.h"
//#include "CTRL_BYP_EVT_OPERATION.h"
#include "EPWM.h"

#define CTRL_SEAMLESS_UV_OV_COUNT_MIN 8
#define CTRL_FAULT_OV_COUNT 166 /* 16.6ms */

extern	AC_Panel	ACP;
extern	Controller	CTRL;

Uns ctrl_fault_ov_cnt = 0;
Uns ctrl_seamless_uv_cnt = 0;
Uns ctrl_seamless_ov_cnt = 0;
Uns ctrl_seamless_uv_lv2_cnt = 0;
Uns ctrl_seamless_ov_lv2_cnt = 0;
Uns ctrl_seamless_uv_cnt_max = 0;
Uns ctrl_seamless_uv_lv2_cnt_max = 0;
Uns ctrl_seamless_uv_ov_cnt_min = CTRL_SEAMLESS_UV_OV_COUNT_MIN;
Uns ctrl_seamless_uv_ov_cnt_min_lv2 = CTRL_SEAMLESS_UV_OV_COUNT_MIN;
Uns ctrl_seamless_enb_instant_fault = FALSE;
Uns ctrl_seamless_enb_instant_fault_lv2 = FALSE;

unsigned int us_cnt_flag = 0;
float MIN_V_RMS = 1000.;
float MAX_V_RMS = 0.;

float Sag_Volt_LV = 0.0;
float Sag_Volt_R = 0.0;
float Sag_Volt_S = 0.0;
float Sag_Volt_T = 0.0;
float Sag_Volt_PK = 0.0;

extern Bool bBlockSeamless;
extern int bTraceSaved;

void CTRL_INV_SEAMLESS_Proceed(void)
{
	if (!CTRL.INV.SEAMLESS.g_enb) return;
#ifdef BYP_EVT_OPERATION
	if (!CTRL.INV.SEAMLESS.enb_for_BEO) return;
#endif

	if(INVERTER.uStatus == SYS_INV_ISLANDING)
	{
		if(Sag_Volt_R > (EXCTRL.fltFsBypV[0].fPrevIn / ACP.PCC.RATE.Vph) * 100.0)
			Sag_Volt_R = (EXCTRL.fltFsBypV[0].fPrevIn / ACP.PCC.RATE.Vph) * 100.0;
		if(Sag_Volt_S > (EXCTRL.fltFsBypV[1].fPrevIn / ACP.PCC.RATE.Vph) * 100.0)
			Sag_Volt_S = (EXCTRL.fltFsBypV[1].fPrevIn / ACP.PCC.RATE.Vph) * 100.0;
		if(Sag_Volt_T > (EXCTRL.fltFsBypV[2].fPrevIn / ACP.PCC.RATE.Vph) * 100.0)
			Sag_Volt_T = (EXCTRL.fltFsBypV[2].fPrevIn / ACP.PCC.RATE.Vph) * 100.0;

		if(Sag_Volt_PK > (ACP.BYP.v_pk / ACP.PCC.RATE.Vph_pk) * 100.)
			Sag_Volt_PK = (ACP.BYP.v_pk / ACP.PCC.RATE.Vph_pk) * 100.;

		if(Sag_Volt_R > Sag_Volt_S)
		{
			if(Sag_Volt_S > Sag_Volt_T)
				Sag_Volt_LV = Sag_Volt_T;
			else
				Sag_Volt_LV = Sag_Volt_S;
		}
		else
		{
			if(Sag_Volt_R > Sag_Volt_T)
				Sag_Volt_LV = Sag_Volt_T;
			else
				Sag_Volt_LV = Sag_Volt_R;
		}

		//if(Sag_data_update_flag)
		{
			if(Sag_Volt_LV < PARAM_VAL(GRID_UV_LEVEL_INSTANT))
			{
				SAG_OP.Sag_data_Instant.VoltR = Sag_Volt_R;
				SAG_OP.Sag_data_Instant.VoltS = Sag_Volt_S;
				SAG_OP.Sag_data_Instant.VoltT = Sag_Volt_T;
				SAG_OP.Sag_data_Instant.VoltLV = Sag_Volt_LV;
			}
			else
			{
				if(Sag_Volt_R > Sag_Volt_S)
				{
					if(Sag_Volt_S > Sag_Volt_T)
					{
						SAG_OP.Sag_data_Instant.VoltR = Sag_Volt_R;
						SAG_OP.Sag_data_Instant.VoltS = Sag_Volt_S;
						SAG_OP.Sag_data_Instant.VoltT = Sag_Volt_PK;
					}
					else
					{
						SAG_OP.Sag_data_Instant.VoltR = Sag_Volt_R;
						SAG_OP.Sag_data_Instant.VoltS = Sag_Volt_PK;
						SAG_OP.Sag_data_Instant.VoltT = Sag_Volt_T;
					}
				}
				else
				{
					if(Sag_Volt_R > Sag_Volt_T)
					{
						SAG_OP.Sag_data_Instant.VoltR = Sag_Volt_R;
						SAG_OP.Sag_data_Instant.VoltS = Sag_Volt_S;
						SAG_OP.Sag_data_Instant.VoltT = Sag_Volt_PK;
					}
					else
					{
						SAG_OP.Sag_data_Instant.VoltR = Sag_Volt_PK;
						SAG_OP.Sag_data_Instant.VoltS = Sag_Volt_S;
						SAG_OP.Sag_data_Instant.VoltT = Sag_Volt_T;
					}
				}
				SAG_OP.Sag_data_Instant.VoltLV = Sag_Volt_PK;
			}
		}
	}
	else
	{
		Sag_Volt_LV = 110.;
		Sag_Volt_R = 110.;
		Sag_Volt_S = 110.;
		Sag_Volt_T = 110.;
		Sag_Volt_PK = 110.;
	}

	if(INVERTER.uStatus == SYS_INV_RUN)
	{
		if(MIN_V_RMS > EXCTRL.fltFsBypV[0].fPrevIn)
			MIN_V_RMS = EXCTRL.fltFsBypV[0].fPrevIn;
		if(MIN_V_RMS > EXCTRL.fltFsBypV[1].fPrevIn)
			MIN_V_RMS = EXCTRL.fltFsBypV[1].fPrevIn;
		if(MIN_V_RMS > EXCTRL.fltFsBypV[2].fPrevIn)
			MIN_V_RMS = EXCTRL.fltFsBypV[2].fPrevIn;

		if(MAX_V_RMS < EXCTRL.fltFsBypV[0].fPrevIn)
			MAX_V_RMS = EXCTRL.fltFsBypV[0].fPrevIn;
		if(MAX_V_RMS < EXCTRL.fltFsBypV[1].fPrevIn)
			MAX_V_RMS = EXCTRL.fltFsBypV[1].fPrevIn;
		if(MAX_V_RMS < EXCTRL.fltFsBypV[2].fPrevIn)
			MAX_V_RMS = EXCTRL.fltFsBypV[2].fPrevIn;
	}
	else
	{
		MIN_V_RMS = 1000.;
		MAX_V_RMS = 0.;
	}

	if(bBlockSeamless)
		return;

	if ( CTRL.INV.operation_mode == PARAM_OPERATION_MODE_AUTO_IS && INVERTER.uStatus == SYS_INV_RUN )
	{
		if(EXCTRL.bypEqe < CTRL.INV.SEAMLESS.pcc_uv_level2_instant) //Level2(pk)
		{
			ctrl_seamless_uv_lv2_cnt++;
			if((ctrl_seamless_uv_lv2_cnt == PRM_PCS[SEAMLESS_FAST_SCR_OFF].iValue) && (PRM_PCS[SEAMLESS_FAST_SCR_OFF].iValue != 0) && !FLT_GetHeavyStatus())
				GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
		}
		if(MIN_V_RMS < CTRL.INV.SEAMLESS.pcc_uv_level_instant )	//Level1(rms)
		{
			ctrl_seamless_uv_cnt++;
			if((ctrl_seamless_uv_cnt == PRM_PCS[SEAMLESS_FAST_SCR_OFF].iValue) && (PRM_PCS[SEAMLESS_FAST_SCR_OFF].iValue != 0) && !FLT_GetHeavyStatus())
				GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
		}
		if((EXCTRL.bypEqe > CTRL.INV.SEAMLESS.pcc_uv_level2_instant) && (MIN_V_RMS > CTRL.INV.SEAMLESS.pcc_uv_level_instant))
		{
			ctrl_seamless_uv_cnt = 0;
			ctrl_seamless_uv_lv2_cnt = 0;
			if(PRM_PCS[SEAMLESS_FAST_SCR_OFF].iValue != 0 && CTRL.INV.SEAMLESS.pcc_blackout_enb == FALSE)
			{
				if(INVERTER.uStatus == SYS_INV_RUN && ctrl_seamless_ov_lv2_cnt == 0 && !FLT_GetHeavyStatus())
					GPIO_StaticSwitch(GPIO_STATIC_SW_ON);
			}
		}

#if 0 /* 180626 Wesco 협의 사항. OV에 대해서는 Seamless 전환하지 않고 고장을 발생시킴. */
		if (ACP.PCC.v_pk > CTRL.INV.SEAMLESS.pcc_ov_level2_instant)
		{
			ctrl_seamless_ov_lv2_cnt++;
			if((ctrl_seamless_ov_lv2_cnt == PRM_PCS[SEAMLESS_FAST_SCR_OFF].iValue) && (PRM_PCS[SEAMLESS_FAST_SCR_OFF].iValue != 0) && !FLT_GetHeavyStatus())
				GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
		}
		else
		{
			ctrl_seamless_ov_lv2_cnt = 0;
			if(PRM_PCS[SEAMLESS_FAST_SCR_OFF].iValue != 0 && CTRL.INV.SEAMLESS.pcc_blackout_enb == FALSE)
			{
				if(INVERTER.uStatus == SYS_INV_RUN && ctrl_seamless_uv_lv2_cnt == 0 && !FLT_GetHeavyStatus())
					GPIO_StaticSwitch(GPIO_STATIC_SW_ON);
			}
		}
#else
		if((EXCTRL.bypEqe > CTRL.INV.SEAMLESS.pcc_ov_level2_instant) || (MAX_V_RMS > CTRL.INV.SEAMLESS.pcc_ov_level_instant))
		{
			if( ctrl_fault_ov_cnt++ >= CTRL_FAULT_OV_COUNT)
				FLT_Raise(FLTH_GRID_OV_LEVEL2);
		}
		else
			ctrl_fault_ov_cnt = 0;
#endif

		if (ctrl_seamless_uv_cnt > ctrl_seamless_uv_cnt_max)
			ctrl_seamless_uv_cnt_max = ctrl_seamless_uv_cnt;
		if(ctrl_seamless_uv_lv2_cnt > ctrl_seamless_uv_lv2_cnt_max)
			ctrl_seamless_uv_lv2_cnt_max = ctrl_seamless_uv_lv2_cnt;
	}
	else
	{
		// Init Count
		ctrl_fault_ov_cnt = 0;
		ctrl_seamless_uv_cnt = 0;
		ctrl_seamless_ov_cnt = 0;
		ctrl_seamless_uv_lv2_cnt = 0;
		ctrl_seamless_ov_lv2_cnt = 0;
		ctrl_seamless_enb_instant_fault = FALSE;
		ctrl_seamless_enb_instant_fault_lv2 = FALSE;
	}

	if((EXCTRL.bypEqe <= CTRL.INV.SEAMLESS.pcc_ov_level2_instant) && (EXCTRL.fltFsBypV[0].fPrevIn <= CTRL.INV.SEAMLESS.pcc_ov_level_instant) &&
	   (EXCTRL.fltFsBypV[1].fPrevIn <= CTRL.INV.SEAMLESS.pcc_ov_level_instant) && (EXCTRL.fltFsBypV[2].fPrevIn <= CTRL.INV.SEAMLESS.pcc_ov_level_instant)
	   && FLT_GetEachStatus(FLTH_GRID_OV_LEVEL2))
	{
		ctrl_fault_ov_cnt = 0;
		FLT_Clear(FLTH_GRID_OV_LEVEL2);
	}

	PRM_PCS[CTRL_SEAMLESS_OV_CNT_STATUS].iValue = ctrl_seamless_uv_cnt_max;
	PRM_PCS[CTRL_SEAMLESS_UV_CNT_STATUS].iValue = ctrl_seamless_uv_lv2_cnt_max;

	//Level1(rms) Check
	if(ctrl_seamless_uv_cnt >= ctrl_seamless_uv_ov_cnt_min || ctrl_seamless_ov_cnt >= ctrl_seamless_uv_ov_cnt_min)
	{
		ctrl_seamless_enb_instant_fault = TRUE;
		ctrl_seamless_uv_cnt = 0;
		ctrl_seamless_ov_cnt = 0;
	}

	//Level2(pk) Check
	if(ctrl_seamless_uv_lv2_cnt >= ctrl_seamless_uv_ov_cnt_min_lv2  || ctrl_seamless_ov_lv2_cnt >= ctrl_seamless_uv_ov_cnt_min_lv2 )
	{
		ctrl_seamless_enb_instant_fault_lv2 = TRUE;
		ctrl_seamless_uv_lv2_cnt = 0;
		ctrl_seamless_ov_lv2_cnt = 0;
	}

	/*
	 * 180626 Wesco 협의사항 GRID OF,UF -> Fault
	 * FLT_GetEachStatus(FLTL_GRID_OF2_SEAMLESS) || FLT_GetEachStatus(FLTL_GRID_UF2_SEAMLESS)
	 */
	if( CTRL.INV.operation_mode == PARAM_OPERATION_MODE_AUTO_IS && INVERTER.uStatus == SYS_INV_RUN && (ctrl_seamless_enb_instant_fault || ctrl_seamless_enb_instant_fault_lv2))
	{
		CTRL.INV.SEAMLESS.pcc_blackout_enb = ON;
		if (CTRL.INV.ctrl_mode == PARAM_CSI_PR)
		{
			CTRL.INV.ctrl_mode = PARAM_VSI_PR;
		}
		CTRL.INV.ctrl_mode_change_enb = ON;

		//CTRL.IRT.csi_to_vsi_oneShot = ON;
		CTRL_INV_VI_OneShotEnable();


		if (PRM_PCS[TRC_TRACE_MODE].iValue == TRC_BYP_OFF_STOP)
		{
//			if( TRACE.iStop == OFF )
//			{
//				bTraceSaved = FALSE;
//			}
			TRC_StopTrace();
		}

		if(ctrl_seamless_enb_instant_fault)
			EVT_Store_NoDup(EVT_PCC_LEVEL1);
		else if(ctrl_seamless_enb_instant_fault_lv2)
			EVT_Store_NoDup(EVT_PCC_LEVEL2);

		ctrl_seamless_enb_instant_fault = FALSE;
		ctrl_seamless_enb_instant_fault_lv2 = FALSE;

		return;
	}
}

Void CTRL_INV_SEAMLESS_UpdateParameter_running()
{
	if( PARAM_VAL(CTRL_SEAMLESS_CNT) < 2 )
		PRM_PCS[CTRL_SEAMLESS_CNT].iValue = 2 / PRM_PCS[CTRL_SEAMLESS_CNT].fIncDec;

	if( PARAM_VAL(CTRL_SEAMLESS_CNT) > 100 )
		PRM_PCS[CTRL_SEAMLESS_CNT].iValue = 100 / PRM_PCS[CTRL_SEAMLESS_CNT].fIncDec;

	ctrl_seamless_uv_ov_cnt_min = PARAM_VAL(CTRL_SEAMLESS_CNT);
	ctrl_seamless_uv_ov_cnt_min_lv2 = PARAM_VAL(CTRL_SEAMLESS_CNT);
}

void CTRL_INV_SEAMLESS_UpdateParameter(void)
{
	CTRL.INV.SEAMLESS.g_enb = PARAM_VAL(SEAMLESS_ENB);
	CTRL.INV.SEAMLESS.pcc_ov_level_instant = ACP.PCC.RATE.Vph * PARAM_VAL(GRID_OV_LEVEL_INSTANT) * 0.01; // 110%
	CTRL.INV.SEAMLESS.pcc_uv_level_instant = ACP.PCC.RATE.Vph * PARAM_VAL(GRID_UV_LEVEL_INSTANT) * 0.01; // RMS

	CTRL.INV.SEAMLESS.pcc_ov_level2_instant = ACP.PCC.RATE.Vph * PARAM_VAL(GRID_OV_LEVEL2_INSTANT) * SQRT2 * 0.01; //130%
	CTRL.INV.SEAMLESS.pcc_uv_level2_instant = ACP.PCC.RATE.Vph * PARAM_VAL(GRID_UV_LEVEL2_INSTANT) * SQRT2 * 0.01; // Peak

	CTRL.INV.SEAMLESS.pcc_uv_level1_instant_restore = ACP.PCC.RATE.Vph * (PARAM_VAL(GRID_UV_LEVEL_INSTANT) + 3.) * 0.01;
	CTRL.INV.SEAMLESS.pcc_uv_level2_instant_restore = ACP.PCC.RATE.Vph * (PARAM_VAL(GRID_UV_LEVEL2_INSTANT) + 3.) * SQRT2 * 0.01;

//	CTRL.INV.SEAMLESS.pcc_uv_level_instant = ACP.PCC.RATE.Vph * SQRT2 * 0.75;
}

void CTRL_INV_SEAMLESS_Reset()
{
	ctrl_fault_ov_cnt = 0;

	ctrl_seamless_uv_cnt = 0;
	ctrl_seamless_ov_cnt = 0;
	ctrl_seamless_uv_lv2_cnt = 0;
	ctrl_seamless_ov_lv2_cnt = 0;

	ctrl_seamless_uv_cnt_max = 0;
	ctrl_seamless_uv_lv2_cnt_max = 0;

	ctrl_seamless_enb_instant_fault = FALSE;
	ctrl_seamless_enb_instant_fault_lv2 = FALSE;
}

void CTRL_INV_SEAMLESS_Create(void)
{
	CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;
	ctrl_seamless_enb_instant_fault = FALSE;
	ctrl_seamless_enb_instant_fault_lv2 = FALSE;
	CTRL_INV_SEAMLESS_UpdateParameter();
	CTRL.INV.SEAMLESS.odt = Odt_(&CTRL.INV.SEAMLESS.ODT, 5 * 1000 /* CC_tsSampleInv / us_cnt_flag */ /* sec */, 1);
	CTRL_INV_SEAMLESS_UpdateParameter_running();
}

Bool CTRL_INV_SEAMLESS_PccNormalOK(void)
{
	if (ACP.PCC.v_pk > CTRL.INV.SEAMLESS.pcc_uv_level_instant && ACP.PCC.v_pk < CTRL.INV.SEAMLESS.pcc_ov_level_instant)
		return TRUE;
	else
		return FALSE;
}

Bool CTRL_INV_SEAMLESS_PccBlackout(void)
{
	if (ACP.PCC.v_pk < CTRL.INV.SEAMLESS.pcc_uv_level_instant * 0.5)
		return TRUE;
	else
		return FALSE;
}

Bool CTRL_Vpcc_NormalOK(void)
{
	if (ACP.PCC.v_pk > (ACP.PCC.RATE.Vph_pk * (float)PRM_PCS[GRID_UV_LEVEL1].iValue * 0.01) )
		return TRUE;
	else
		return FALSE;
}

void CTRL_SEAMLESS_PWM_OFF(void)
{
#if 0 //by JCNET
	if(CTRL.INV.SEAMLESS.pcc_blackout_enb == TRUE)
	{
		CTRL.INV.SEAMLESS.PwmOffCnt++;
		EPWM_Disable();

		if(CTRL.INV.SEAMLESS.PwmOffCnt >= PARAM_VAL(SEAMLESS_PWM_OFF_CNT))
		{
			CTRL.INV.SEAMLESS.PwmOffCnt = PARAM_VAL(SEAMLESS_PWM_OFF_CNT);
			EPWM_Enable();
		}
	}
	else
	{
		CTRL.INV.SEAMLESS.PwmOffCnt = 0;

		if(IVC.uStatus != CVC_DISABLE)
			EPWM_Enable();
	}
#endif
}
