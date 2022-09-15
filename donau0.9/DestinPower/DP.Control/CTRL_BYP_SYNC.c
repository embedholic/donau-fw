/*
 * CTRL_BYP_SYNC.c
 *
 *  Created on: 2014. 4. 21.
 *      Author: Seth Oh
 */
#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "CTRL_BYP_SYNC.h"
#include "CC.h"
#include "PI.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "filter.h"
#include "CTRL_FILTER.h"
#include "GPIO.h"
#include "trace.h"
#include "FAULT.h"
#include "SYS_ESLV.h"
#include "MCCB.h"
#include "CTRL_GEN.h"

extern	AC_Panel	ACP;
extern	Controller	CTRL;


#if DBUG_MODE == 3
extern float fDebug;
int fEventCnd = 0;
#endif
void CTRL_BYP_SYNC_Proceed(void)
{
	volatile float a;
	volatile iir1st *pIIR1;
	volatile PICon *pPICon;
	volatile PIArg *pPIArg;

	// [150806] POSCO GI Test
	//-if (!CTRL.BYP.SYNC.g_enb && !eslvCtrl.g_bEnb && !(PRM_PCS[BYP_MODE].iValue == 3) && !(PRM_PCS[BYP_MODE].iValue == 10)) return;
	if (!CTRL.BYP.SYNC.g_enb && !eslvCtrl.g_bEnb && (PRM_PCS[BYP_MODE].iValue == 0)) return;

	a = - ACP.PCC.vds * ACP.BYP.vqs + ACP.PCC.vqs * ACP.BYP.vds;

	//150810 for POSCO
/*	if( PRM_PCS[BYP_MODE].iValue == 20 )
	{
		if(INVERTER.uStatus == SYS_INV_RE_SYNC)
			a = - ACP.INV.vds_ll * ACP.BYP.vqs + ACP.INV.vqs_ll * ACP.BYP.vds;
	}*/

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2(&CTRL.BYP.SYNC.FILTER.iir_w, a);
	}
	else
	{
		IIR1_Filter2_IS(&CTRL.BYP.SYNC.FILTER.iir_w, a);
	}

	/*
	 * 160405
	 */
	if( INVERTER.uStatus == SYS_INV_RE_SYNC )
	{
		CTRL.BYP.SYNC.PI_W.ARG.fMax = (2 * PI * PARAM_VAL(GRID_RATED_FREQ)) * PARAM_VAL(RESYNC_W_PI_MAX) * 0.01;
		CTRL.BYP.SYNC.PI_W.ARG.fMin = - CTRL.BYP.SYNC.PI_W.ARG.fMax;
	}
	else
	{
		CTRL.BYP.SYNC.PI_W.ARG.fMax = (2 * PI * PARAM_VAL(GRID_RATED_FREQ)) * PARAM_VAL(SYNC_W_PI_MAX) * 0.01;
		CTRL.BYP.SYNC.PI_W.ARG.fMin = - CTRL.BYP.SYNC.PI_W.ARG.fMax;
	}

	CTRL.BYP.SYNC.PI_W.ARG.fErr = CTRL.BYP.SYNC.FILTER.iir_w.fOut;
	PICON_PI_ONLY_RunM((&CTRL.BYP.SYNC.PI_W.CORE), (&CTRL.BYP.SYNC.PI_W.ARG));
	CTRL.BYP.SYNC.w = CTRL.BYP.SYNC.PI_W.CORE.fOut;

	ACP.BYP.v_pk = sqrt(ACP.BYP.vds * ACP.BYP.vds + ACP.BYP.vqs * ACP.BYP.vqs);
	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2(&CTRL.BYP.SYNC.FILTER.iir_v, ACP.BYP.v_pk);
	}
	else
	{
		IIR1_Filter2_IS(&CTRL.BYP.SYNC.FILTER.iir_v, ACP.BYP.v_pk);
	}
	CTRL.BYP.SYNC.v = ACP.TR.RATE.RATIO * CTRL.BYP.SYNC.FILTER.iir_v.fOut;
}

void CTRL_BYP_SYNC_UpdateParameter(void)
{
	float inv_freq;

	if( PRM_PCS[GRID_GATE_WAY_ENB].iValue == 0 )
		CTRL.BYP.SYNC.g_enb = PARAM_VAL(BYP_MODE) == 1;
	else
		CTRL.BYP.SYNC.g_enb = 0;

	if( PRM_PCS[SYNC_W_PI_KP_DIVISION].iValue < 1 )
	{
		PRM_PCS[SYNC_W_PI_KP_DIVISION].iValue = 1;
	}

	CTRL.BYP.SYNC.PI_W.K.fP = (float)PARAM_VAL(SYNC_W_PI_KP) / (float)PRM_PCS[SYNC_W_PI_KP_DIVISION].iValue ;
	CTRL.BYP.SYNC.PI_W.K.fIT = (float)PARAM_VAL(SYNC_W_PI_KI);
	CTRL.BYP.SYNC.PI_W.K.fA = 1. / CTRL.BYP.SYNC.PI_W.K.fP;

//	CTRL.BYP.SYNC.ov_level_instant = ACP.INV.RATE.Vph * PARAM_VAL(BYP_OV_LEVEL_INSTANT) * SQRT2 * 0.01;
//	CTRL.BYP.SYNC.uv_level_instant = ACP.INV.RATE.Vph * PARAM_VAL(BYP_UV_LEVEL_INSTANT) * SQRT2 * 0.01;

	inv_freq = 	PARAM_VAL(GRID_RATED_FREQ);
	CTRL.BYP.SYNC.PI_W.ARG.fMax = (2 * PI * inv_freq) * PARAM_VAL(SYNC_W_PI_MAX) * 0.01;
	CTRL.BYP.SYNC.PI_W.ARG.fMin = - CTRL.BYP.SYNC.PI_W.ARG.fMax;
}

void CTRL_BYP_SYNC_Create(void)
{
	float t_sample_main, t_sample_is;

	t_sample_main = CTRL_FILTER_GetSampleTime(CTRL_CC_PERIOD);
	t_sample_is = CTRL_FILTER_GetSampleTime(IS_CCP);

	IIR1_Creation_IS(&CTRL.BYP.SYNC.FILTER.iir_w, t_sample_main, 10. , t_sample_is);
	IIR1_Creation_IS(&CTRL.BYP.SYNC.FILTER.iir_v, t_sample_main, 5. , t_sample_is);

	PICon_(&CTRL.BYP.SYNC.PI_W.CORE);
	PIArg_(&CTRL.BYP.SYNC.PI_W.ARG, &CTRL.BYP.SYNC.PI_W.K);
	CTRL.BYP.SYNC.PI_W.ARG.fAlpha = 1; // 1 ==> PI 제어, 0 ==> IP 제어
#ifndef RE_SYNC_DEBUG
	CTRL.BYP.SYNC.sync_accept_rad = (float)PARAM_VAL(CTRL_IINV_SYNC_TOLERANCE_THETA) * ACP.INV.RATE.Omega / 360.; // 1 deg
#else
	CTRL.BYP.SYNC.sync_accept_rad = (float)PARAM_VAL(CTRL_IINV_SYNC_TOLERANCE_THETA) * 2 * PI / 360.; // 1 deg
#endif
	// TODO CHECK 3/180 * 3.14

	CTRL.BYP.SYNC.odtBypNormal = Odt_(&CTRL.BYP.SYNC.ODT_BYP_NORMAL, 5000, 5 /*ms SystemState*/);


	CTRL_BYP_SYNC_UpdateParameter();
}

Bool CTRL_BYP_NormalOK(void)
{
	//1. if (ACP.BYP.v_pk > 296. && ACP.BYP.v_pk < 327.  )
	//2. if( ACP.BYP.v_pk > (ACP.PCC.RATE.Vph_pk ) * 0.9 && ACP.BYP.v_pk < (ACP.PCC.RATE.Vph_pk  )* 1.1 ) // BYPASS 쪽은 RATE가 계산이 안되어 있음.
	if ( ACP.BYP.v_pk > (ACP.PCC.RATE.Vph_pk * 0.9) )
		return TRUE;
	else
		return FALSE;
}

Bool CTRL_BYP_PCC_SyncOK( void )
{
#if DBUG_MODE == 2
	return TRUE;
#else
	//  w = theta omega. radian * 60 Hz
	//- 150810 if ( (fabs(CTRL.BYP.SYNC.w) < (CTRL.BYP.SYNC.sync_accept_rad * 2 * PI)) )
	if( CTRL_BYP_NormalOK() )
	{
		//160418 delete. Resync 상태에서 싱크 하도록 수정 함.
//		if( PRM_PCS[BYP_MODE].iValue == 20  )
//		{
//			if ( (fabs(CTRL.BYP.SYNC.w) < (CTRL.BYP.SYNC.sync_accept_rad)) )
//				return TRUE;
//			else
//				return FALSE;
//		}
//		else
		{

			// BYP 20 에서는 Islanding 상태에서

			if(INVERTER.uStatus == SYS_INV_RE_SYNC)
			{
#ifndef RE_SYNC_DEBUG
				if ( (fabs(CTRL.BYP.SYNC.w) < (CTRL.BYP.SYNC.sync_accept_rad)) )
					return TRUE;
				else
					return FALSE;
#else
				if ( (fabs(ACP.BYP.THETA.diff_rad) < (CTRL.BYP.SYNC.sync_accept_rad)) )
					return TRUE;
				else
					return FALSE;
#endif
			}
			else
				return FALSE;
		}
	}
	else
	{
		return FALSE;
	}
#endif
}

