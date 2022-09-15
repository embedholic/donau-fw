/*
 * CTRL_INV_SYNC.c
 *
 *  Created on: 2014. 4. 3.
 *      Author: Seth Oh
 */
#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "CTRL_INV_SYNC.h"
#include "CC.h"
#include "PI.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "filter.h"
#include "CTRL_FILTER.h"
#include "CTRL_GEN.h"
#include "MCCB.h"

extern	AC_Panel	ACP;
extern	Controller	CTRL;

void CTRL_INV_SYNC_Proceed(void)
{
	volatile float a, b, c;
	volatile iir1st *pIIR1;
	volatile PICon *pPICon;
	volatile PIArg *pPIArg;

	if( PRM_PCS[INV_TRANSFORMER].iValue == 1)
		a = - ACP.INV.vds_ll * ACP.PCC.vqs + ACP.INV.vqs_ll * ACP.PCC.vds;
	else
		a = - ACP.INV.vds * ACP.PCC.vqs + ACP.INV.vqs * ACP.PCC.vds;


	b = sqrt(ACP.PCC.vds * ACP.PCC.vds + ACP.PCC.vqs * ACP.PCC.vqs);
	c = sqrt(ACP.INV.vds * ACP.INV.vds + ACP.INV.vqs * ACP.INV.vqs);

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2(&CTRL.INV.SYNC.FILTER.iir_w, a);
		IIR1_Filter2(&CTRL.INV.SYNC.FILTER.iir_v, b);
		IIR1_Filter2(&CTRL.INV.SYNC.FILTER.iir_inv_v, c);

	}
	else
	{
		IIR1_Filter2_IS(&CTRL.INV.SYNC.FILTER.iir_w, a);
		IIR1_Filter2_IS(&CTRL.INV.SYNC.FILTER.iir_v, b);
		IIR1_Filter2_IS(&CTRL.INV.SYNC.FILTER.iir_inv_v, c);
	}


	/*[ PI Controller Run - Sync Controller]**************************************/
	CTRL.INV.SYNC.PI_W.ARG.fErr = CTRL.INV.SYNC.FILTER.iir_w.fOut;
	PICON_PI_ONLY_RunM((&CTRL.INV.SYNC.PI_W.CORE), (&CTRL.INV.SYNC.PI_W.ARG));
	CTRL.INV.SYNC.w = CTRL.INV.SYNC.PI_W.CORE.fOut;
	CTRL.INV.SYNC.v = ACP.TR.RATE.RATIO * CTRL.INV.SYNC.FILTER.iir_v.fOut;


	/*[ PI Controller Run - Mag Controller-Generator]**************************************/
	CTRL.INV.SYNC.pi_mag_input = CTRL.GEN.SYNC.v - CTRL.INV.SYNC.FILTER.iir_inv_v.fOut;
	CTRL.INV.SYNC.PI_MAG.ARG.fErr = CTRL.INV.SYNC.pi_mag_input; // CTRL.INV.SYNC.FILTER.iir_pi_mag.fOut;
	PICON_PI_ONLY_RunM((&CTRL.INV.SYNC.PI_MAG.CORE), (&CTRL.INV.SYNC.PI_MAG.ARG));
	// 출력 -> v_sec(Resync)

	/*[ PI Controller Run - Mag Controller-Bypass]**************************************/
	CTRL.INV.SYNC.pi_mag_input = CTRL.BYP.SYNC.v - CTRL.INV.SYNC.FILTER.iir_inv_v.fOut;
	CTRL.INV.SYNC.PI_MAG_BYP.ARG.fErr = CTRL.INV.SYNC.pi_mag_input;
	PICON_PI_ONLY_RunM((&CTRL.INV.SYNC.PI_MAG_BYP.CORE), (&CTRL.INV.SYNC.PI_MAG_BYP.ARG));
	// 출력 -> v_sec(Resync)

	if ( CTRL.INV.operation_mode == PARAM_OPERATION_MODE_IS ) // operation_mode: IS(1)
	{
		switch (INVERTER.uStatus)
		{
		case SYS_INV_AC_GENERATE: /*PARAM_OPERATION_MODE_IS*/
		default:
			CTRL.INV.DROOP.v_sec = 0.;
			CTRL.INV.DROOP.w_sec = 0.;
			PICON_Initialize(&CTRL.INV.PQC.PI_Q.CORE);
			PICON_Initialize(&CTRL.INV.PQC.PI_P.CORE);
			RAMP_Initialize(&CTRL.INV.PQC.RAMP.v_ref);
			RAMP_Initialize(&CTRL.INV.PQC.RAMP.w_ref);
			break;
		case SYS_INV_START_SYNC:/*PARAM_OPERATION_MODE_IS*/
			CTRL.INV.DROOP.v_sec = CTRL.INV.SYNC.v - CTRL.INV.DROOP.v_ref_tr;
			CTRL.INV.DROOP.w_sec = CTRL.INV.SYNC.w;
			break;
		case SYS_INV_RUN:/*PARAM_OPERATION_MODE_IS*/
		case SYS_INV_ISLANDING:/*PARAM_OPERATION_MODE_IS*/
			CTRL.INV.DROOP.v_sec = 0.;
			CTRL.INV.DROOP.w_sec = 0.; //CTRL.INV.SYNC.w;
			break;
		case SYS_INV_RE_SYNC: /*PARAM_OPERATION_MODE_IS*/
			if( CTRL_BYP_NormalOK() )
			{
				CTRL.INV.DROOP.v_sec = CTRL.INV.SYNC.PI_MAG_BYP.CORE.fOut;
				CTRL.INV.DROOP.w_sec = CTRL.BYP.SYNC.w;
			}
			else
			{	/* Generator Sync */
				CTRL.INV.DROOP.v_sec = CTRL.INV.SYNC.PI_MAG.CORE.fOut; //CTRL.GEN.SYNC.v - CTRL.INV.DROOP.v_ref_tr; (두대 병렬 상황에서 Droop깨짐)
				CTRL.INV.DROOP.w_sec = CTRL.GEN.SYNC.w;
			}
			break;
		}
	}
	else
	{
		switch (INVERTER.uStatus)
		{
		case SYS_INV_AC_GENERATE:
		case SYS_INV_ISLANDING:
		default:
			CTRL.INV.DROOP.v_sec = 0.;
			CTRL.INV.DROOP.w_sec = 0.;
			PICON_Initialize(&CTRL.INV.PQC.PI_Q.CORE);
			PICON_Initialize(&CTRL.INV.PQC.PI_P.CORE);
			RAMP_Initialize(&CTRL.INV.PQC.RAMP.v_ref);
			RAMP_Initialize(&CTRL.INV.PQC.RAMP.w_ref);

			break;
		case SYS_INV_START_SYNC:
			CTRL.INV.DROOP.v_sec = CTRL.INV.SYNC.v - CTRL.INV.DROOP.v_ref_tr;
			CTRL.INV.DROOP.w_sec = CTRL.INV.SYNC.w;
			break;
		case SYS_INV_RUN:
			if ( CTRL.INV.ctrl_mode == PARAM_VSI_PR ) // VSI_p+r
			{
// 				100kW 일경우 커패시터 무효 전력 보상분으로 9*omega*Cf*Inv V peak^2 - (9. * 2.*PI*PARAM_VAL(GRID_RATED_FREQ) * 80e-6 * (INVCTRL.fltEqe.fOut*INVCTRL.fltEqe.fOut)
//				a = 4.5 * ACP.INV.RATE.Omega * 80e-6 * (INVCTRL.fltEqe.fOut*INVCTRL.fltEqe.fOut);
//				CTRL.INV.PQC.q_ref = (PRM_PCS[BATT_VF_CHECK_TIME].iValue * 1000.) - a;
				CTRL.INV.PQC.q_ref = (PRM_PCS[BATT_VF_CHECK_TIME].iValue * 1000.);
				CTRL.INV.PQC.PI_Q.ARG.fErr = (CTRL.INV.PQC.q_ref - ACP.INV.q_f) ;
				a = PICON_NO_COMPEN_Run((&CTRL.INV.PQC.PI_Q.CORE), (&CTRL.INV.PQC.PI_Q.ARG));
				CTRL.INV.PQC.v_ref_ramp = RAMP_Change(&CTRL.INV.PQC.RAMP.v_ref , CTRL.INV.PQC.PI_Q.CORE.fOut);

				CTRL.INV.PQC.v_ref = CTRL.INV.PQC.v_ref_ramp + (CTRL.INV.PQC.PI_Q.ARG.fErr * CTRL.INV.DROOP.GAIN.N*PARAM_VAL(BATT_VDC_RAMP));
				CTRL.INV.PQC.q_compen_coeff = ACP.INV.RATE.Omega * INVCTRL.Li * INVCTRL.EPeakInverse;
				CTRL.INV.PQC.v_ref += CTRL.INV.PQC.q_compen_coeff * CTRL.INV.PQC.q_ref;

				// 150923 0으로 사용.
				//-151106 CTRL.INV.DROOP.v_sec = 0; //
				// 151106 Office. TR LESS 버전에서 인너시가 생길 수 있음. 다시 추가.
				CTRL.INV.DROOP.v_sec =  CTRL.INV.PQC.v_ref + CTRL.INV.SYNC.v - CTRL.INV.DROOP.v_ref;

//deleted		CTRL.INV.DROOP.v_sec = CTRL.INV.PQC.q_compen_coeff * CTRL.INV.PQC.q_ref * 1.; // Open Loop
//deleted		CTRL.INV.DROOP.v_sec = PRM_PCS[BATT_VF_CHECK_TIME].iValue + 5.3;

				CTRL.INV.PQC.PI_P.ARG.fErr = (CTRL.INV.PQC.p_ref - ACP.INV.p_f) ;
				b = PICON_Run((&CTRL.INV.PQC.PI_P.CORE), (&CTRL.INV.PQC.PI_P.ARG));
				CTRL.INV.PQC.w_ref_ramp = RAMP_Change(&CTRL.INV.PQC.RAMP.w_ref , CTRL.INV.PQC.PI_P.CORE.fOut);
				CTRL.INV.PQC.w_ref = CTRL.INV.PQC.w_ref_ramp + (CTRL.INV.PQC.PI_P.ARG.fErr * CTRL.INV.DROOP.GAIN.M*PARAM_VAL(BATT_CURRENT_REF_OFFSET));
				CTRL.INV.PQC.w_ref += CTRL.INV.PQC.p_compen_coeff * CTRL.INV.PQC.p_ref;

				CTRL.INV.DROOP.w_sec = CTRL.INV.SYNC.w + CTRL.INV.PQC.w_ref;

		//		CTRL.INV.DROOP.w_sec = CTRL.INV.SYNC.w + CTRL.INV.PQC.p_compen_coeff * CTRL.INV.PQC.p_ref; // Open Loop
		//		CTRL.INV.DROOP.w_sec = CTRL.INV.SYNC.w;

				// BypassMode 30에서 DROOP 기능과 PQC 제어기 기능은 상관 없으므로, v_sec,w_sec를 0으로 놓아도 됨.
				// PQC제어기는 계통 연계 전압 제어 하다가 독립 운전 전압 제어 모드를 넘어가기 위한 제어기 임.
			}
			else if ( CTRL.INV.ctrl_mode == PARAM_CSI_PR )
			{
				CTRL.INV.DROOP.v_sec = 0;
				CTRL.INV.DROOP.w_sec = CTRL.INV.SYNC.w;
			}
			break;

		case SYS_INV_RE_SYNC:
			/* 151014 */
			if( PRM_PCS[BYP_MODE].iValue == 20 )
			{
#ifndef STABLEEN
				if( CTRL_BYP_NormalOK() )
				{
#endif
					CTRL.INV.DROOP.v_sec = CTRL.INV.SYNC.PI_MAG_BYP.CORE.fOut;
					CTRL.INV.DROOP.w_sec = CTRL.BYP.SYNC.w;
#ifndef STABLEEN
				}
				else
				{	/* Generator Sync */
					CTRL.INV.DROOP.v_sec = CTRL.INV.SYNC.PI_MAG.CORE.fOut; //CTRL.GEN.SYNC.v - CTRL.INV.DROOP.v_ref_tr; (두대 병렬 상황에서 Droop깨짐)
					CTRL.INV.DROOP.w_sec = CTRL.GEN.SYNC.w;
				}
#endif
			}
			else if( PRM_PCS[BYP_MODE].iValue == 30  )
			{
				CTRL.INV.DROOP.v_sec = CTRL.INV.SYNC.PI_MAG.CORE.fOut; //CTRL.GEN.SYNC.v - CTRL.INV.DROOP.v_ref_tr; (두대 병렬 상황에서 Droop깨짐)
				CTRL.INV.DROOP.w_sec = CTRL.GEN.SYNC.w;
			}
			else
			{
				CTRL.INV.DROOP.v_sec = CTRL.BYP.SYNC.v - CTRL.INV.DROOP.v_ref_tr;
				CTRL.INV.DROOP.w_sec = CTRL.BYP.SYNC.w;
			}
		}
	}
	CTRL.INV.DROOP.w_ref_sec = CTRL.INV.DROOP.w_ref + CTRL.INV.DROOP.w_sec;
	CTRL.INV.DROOP.v_ref_sec = CTRL.INV.DROOP.v_ref_tr + CTRL.INV.DROOP.v_sec;

}

void CTRL_INV_SYNC_UpdateParameter(void)
{
	float inv_freq;

	if( PRM_PCS[SYNC_W_PI_KP_DIVISION].iValue < 1 )
	{
		PRM_PCS[SYNC_W_PI_KP_DIVISION].iValue = 1;
	}

	CTRL.INV.SYNC.PI_W.K.fP = (float)PARAM_VAL(SYNC_W_PI_KP) / (float)PRM_PCS[SYNC_W_PI_KP_DIVISION].iValue;
	CTRL.INV.SYNC.PI_W.K.fIT = (float)PARAM_VAL(SYNC_W_PI_KI);
//	CTRL.INV.SYNC.PI_W.K.fP = 0.002;
//	CTRL.INV.SYNC.PI_W.K.fIT = 0.;
	CTRL.INV.SYNC.PI_W.K.fA = 1. / CTRL.INV.SYNC.PI_W.K.fP;

	inv_freq = 	PARAM_VAL(GRID_RATED_FREQ);
	CTRL.INV.SYNC.PI_W.ARG.fMax = (2 * PI * inv_freq) * PARAM_VAL(SYNC_W_PI_MAX) * 0.01;
//	CTRL.INV.SYNC.PI_W.ARG.fMax = (2 * PI * inv_freq) * 25. * 0.01;
	CTRL.INV.SYNC.PI_W.ARG.fMin = - CTRL.INV.SYNC.PI_W.ARG.fMax;

	if( PRM_PCS[SYNC_PI_MAG_LIMIT].iValue < 0 )
		PRM_PCS[SYNC_PI_MAG_LIMIT].iValue = 0;

	//151215
	CTRL.INV.SYNC.PI_MAG.K.fP = (float)PARAM_VAL(CTRL_SYNC_PI_MAG_P)  / 100.; // 160113 "/ 1000" -> "/ 100"
	CTRL.INV.SYNC.PI_MAG.K.fIT = (float)PARAM_VAL(CTRL_SYNC_PI_MAG_I) / 100.; // 160113 "/ 1000" -> "/ 100"
	CTRL.INV.SYNC.PI_MAG.K.fA = 1. / CTRL.INV.SYNC.PI_MAG.K.fP;
	CTRL.INV.SYNC.PI_MAG.ARG.fMax = ACP.INV.RATE.Vph_pk * ( PARAM_VAL(SYNC_PI_MAG_LIMIT) / 100 );//160106 - ACP.INV.RATE.Vph_pk * 0.2;
	CTRL.INV.SYNC.PI_MAG.ARG.fMin = -CTRL.INV.SYNC.PI_MAG.ARG.fMax;

	CTRL.INV.SYNC.PI_MAG_BYP.K.fP = (float)PARAM_VAL(CTRL_SYNC_PI_MAG_P)  / 100.; // 160113 "/ 1000" -> "/ 100"
	CTRL.INV.SYNC.PI_MAG_BYP.K.fIT = (float)PARAM_VAL(CTRL_SYNC_PI_MAG_I) / 100.; // 160113 "/ 1000" -> "/ 100"
	CTRL.INV.SYNC.PI_MAG_BYP.K.fA = 1. / CTRL.INV.SYNC.PI_MAG_BYP.K.fP;
	CTRL.INV.SYNC.PI_MAG_BYP.ARG.fMax = ACP.INV.RATE.Vph_pk * ( PARAM_VAL(SYNC_PI_MAG_LIMIT) / 100 );//160106 - ACP.INV.RATE.Vph_pk * 0.2;
	CTRL.INV.SYNC.PI_MAG_BYP.ARG.fMin = -CTRL.INV.SYNC.PI_MAG_BYP.ARG.fMax;

	IIR1_UpdateCoeff(&CTRL.INV.SYNC.FILTER.iir_w, CC_tsSample, PARAM_VAL(SYNC_W_IIR_CUTOFF));
	IIR1_UpdateCoeff(&CTRL.INV.SYNC.FILTER.iir_v, CC_tsSample, PARAM_VAL(SYNC_V_IIR_CUTOFF));
	IIR1_UpdateCoeff(&CTRL.INV.SYNC.FILTER.iir_inv_v, CC_tsSample, PARAM_VAL(SYNC_V_IIR_CUTOFF));
	//IIR1_UpdateCoeff(&CTRL.INV.SYNC.FILTER.iir_pi_mag, CC_tsSample, PARAM_VAL(SYNC_PI_MAG_FILTER));

}

void CTRL_INV_SYNC_Create(void)
{
	float t_sample_main, t_sample_is;

	t_sample_main = CTRL_FILTER_GetSampleTime(CTRL_CC_PERIOD);
	t_sample_is = CTRL_FILTER_GetSampleTime(IS_CCP);

	IIR1_Creation_IS(&CTRL.INV.SYNC.FILTER.iir_w, t_sample_main, 10. , t_sample_is);
	IIR1_Creation_IS(&CTRL.INV.SYNC.FILTER.iir_v, t_sample_main, 5. , t_sample_is);
	IIR1_Creation_IS(&CTRL.INV.SYNC.FILTER.iir_inv_v, t_sample_main, 5. , t_sample_is);
	//IIR1_Creation_IS(&CTRL.INV.SYNC.FILTER.iir_pi_mag, t_sample_main, 5. , t_sample_is);

	PICon_(&CTRL.INV.SYNC.PI_W.CORE);
	PIArg_(&CTRL.INV.SYNC.PI_W.ARG, &CTRL.INV.SYNC.PI_W.K);
	CTRL.INV.SYNC.PI_W.ARG.fAlpha = 1; // 1 ==> PI 제어, 0 ==> IP 제어

	//151215
	PICon_(&CTRL.INV.SYNC.PI_MAG.CORE);
	PIArg_(&CTRL.INV.SYNC.PI_MAG.ARG, &CTRL.INV.SYNC.PI_MAG.K);
	CTRL.INV.SYNC.PI_MAG.ARG.fAlpha = 1; // 1 ==> PI 제어, 0 ==> IP 제어

	PICon_(&CTRL.INV.SYNC.PI_MAG_BYP.CORE);
	PIArg_(&CTRL.INV.SYNC.PI_MAG_BYP.ARG, &CTRL.INV.SYNC.PI_MAG_BYP.K);
	CTRL.INV.SYNC.PI_MAG_BYP.ARG.fAlpha = 1; // 1 ==> PI 제어, 0 ==> IP 제어

	/*
	 * SYNC를 천천히 가져가기 위한 파라미터 임
	 */
	if( PRM_PCS[RESYNC_W_PI_MAX].iValue <= 0 )
		PRM_PCS[RESYNC_W_PI_MAX].iValue = 1;

	CTRL_INV_SYNC_UpdateParameter();
}


