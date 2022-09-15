/*
 * CTRL_INV_DROOP.c
 *
 *  Created on: 2014. 4. 3.
 *      Author: Seth Oh
 */

#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "CTRL_INV_DROOP.h"
#include "CC.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "CTRL_FILTER.h"
#include "CTRL_GEN.h"

extern	AC_Panel	ACP;
extern	Controller	CTRL;


void CTRL_INV_DROOP_Control(void)
{
	volatile float a, b;
	volatile iir1st *pIIR1;
//	volatile PICon *pPICon;
//	volatile PIArg *pPIArg;

//	ACP.INV.ids = INVCTRL.Ids;
//	ACP.INV.iqs = INVCTRL.Iqs;
	ACP.INV.ids = (2*ACP.INV.ia - ACP.INV.ib - ACP.INV.ic)*INV_3;
	ACP.INV.iqs = (- ACP.INV.ib + ACP.INV.ic) * INV_SQRT3;

//	ACP.INV.p = - 1.5 * (ACP.INV.vds * ACP.INV.ids + ACP.INV.vqs * ACP.INV.iqs);
//	ACP.INV.q = - 1.5 * (ACP.INV.vqs * ACP.INV.ids - ACP.INV.vds * ACP.INV.iqs);

	if (PARAM_RAW_VAL(CTRL_POS_SEQ_PLL_ENB))
	{
		ACP.INV.p = 1.5 * (ACP.INV.vds_p * ACP.INV.ids + ACP.INV.vqs_p * ACP.INV.iqs);
		ACP.INV.q = 1.5 * (- ACP.INV.vqs_p * ACP.INV.ids + ACP.INV.vds_p * ACP.INV.iqs);
	}
	else
	{
		ACP.INV.p = 1.5 * (ACP.INV.vds * ACP.INV.ids + ACP.INV.vqs * ACP.INV.iqs);
		ACP.INV.q = 1.5 * (- ACP.INV.vqs * ACP.INV.ids + ACP.INV.vds * ACP.INV.iqs);
	}

	IIR1_Filter(&ACP.INV.FILTER.iir1st_p, ACP.INV.p, ACP.INV.p_f);
	IIR1_Filter(&ACP.INV.FILTER.iir1st_q, ACP.INV.q, ACP.INV.q_f);

	if (ACP.INV.p_f > ACP.INV.RATE.P3)
		ACP.INV.p_f_lim = ACP.INV.RATE.P3;
	else if (ACP.INV.p_f < - ACP.INV.RATE.P3)
		ACP.INV.p_f_lim = - ACP.INV.RATE.P3;
	else
		ACP.INV.p_f_lim = ACP.INV.p_f;

	if (ACP.INV.q_f > ACP.INV.RATE.Q3)
		ACP.INV.q_f_lim = ACP.INV.RATE.Q3;
	else if (ACP.INV.q_f < - ACP.INV.RATE.Q3)
		ACP.INV.q_f_lim = - ACP.INV.RATE.Q3;
	else
		ACP.INV.q_f_lim = ACP.INV.q_f;

	CTRL.INV.PQC.p_ref = PRM_PCS[BATT_DCHG_PWR].iValue * 1000.; // 100000.;
//	CTRL.INV.PQC.p_ref = 0.;

	if (PRM_PCS[DROOP_ENB].iValue)
	{
/*
		if (CTRL.INV.operation_mode == 0 || CTRL.INV.operation_mode == 2)
		{
			if ( INVERTER.uStatus != SYS_INV_RUN )
				CTRL.INV.DROOP.enb = ON;
			else
				CTRL.INV.DROOP.enb = OFF;
		}
		else
		{
			CTRL.INV.DROOP.enb = ON;
		}
*/
		if ( CTRL.INV.grid_connected_enb )
		{
			CTRL.INV.DROOP.enb = OFF;

		}
		else
		{
			CTRL.INV.DROOP.enb = ON;
		}
	}
	else
	{
		CTRL.INV.DROOP.enb = OFF;
	}




	if (CTRL.INV.DROOP.enb)
	{
#if 0 /* T2 */
		// ramp
		if( GenBlock.bApply_w_ramp == TRUE )
		{
#if 0
		if(  GenBlock.bSetInit_w_ramp )
		{
			 GenBlock.bSetInit_w_ramp = FALSE;
			 RAMP_SetInitOut(&CTRL.INV.DROOP.RAMP.p_ref , CTRL.INV.DROOP.Omega_ref - ( CTRL.INV.DROOP.GAIN.M * (ACP.INV.p_f_lim- ACP.INV.RATE.P3))); /*+ 151215 */
			 RAMP_SetInitOut(&CTRL.INV.DROOP.RAMP.q_ref , CTRL.INV.DROOP.Voltage_ref - ( CTRL.INV.DROOP.GAIN.N * (ACP.INV.q_f_lim- ACP.INV.RATE.Q3))); /*+ 151215 */
		}

		CTRL.INV.DROOP.w_ref = RAMP_Change(&CTRL.INV.DROOP.RAMP.p_ref , CTRL.INV.DROOP.Omega_ref - ( CTRL.INV.DROOP.GAIN.M * (ACP.INV.p_f_lim- ACP.INV.RATE.P3)));
		CTRL.INV.DROOP.v_ref = RAMP_Change(&CTRL.INV.DROOP.RAMP.q_ref , CTRL.INV.DROOP.Voltage_ref - ( CTRL.INV.DROOP.GAIN.N * (ACP.INV.q_f_lim- ACP.INV.RATE.Q3)));
#endif

		}
		else
		{
			CTRL.INV.DROOP.w_ref = CTRL.INV.DROOP.Omega_ref - ( CTRL.INV.DROOP.GAIN.M * (ACP.INV.p_f_lim- ACP.INV.RATE.P3));
			CTRL.INV.DROOP.v_ref = CTRL.INV.DROOP.Voltage_ref - ( CTRL.INV.DROOP.GAIN.N * (ACP.INV.q_f_lim- ACP.INV.RATE.Q3));
		}
#else
		CTRL.INV.DROOP.w_ref = CTRL.INV.DROOP.Omega_ref - ( CTRL.INV.DROOP.GAIN.M * (ACP.INV.p_f_lim- ACP.INV.RATE.P3));
		CTRL.INV.DROOP.v_ref = CTRL.INV.DROOP.Voltage_ref - ( CTRL.INV.DROOP.GAIN.N * (ACP.INV.q_f_lim- ACP.INV.RATE.Q3));
#endif
	}
	else
	{
		CTRL.INV.DROOP.w_ref = ACP.INV.RATE.Omega;

		if( (BYP_MODE_CANADA() &&  CTRL.INV.operation_mode == PARAM_OPERATION_MODE_IS) || PRM_PCS[BYP_MODE].iValue == 20)
		{
			CTRL.INV.DROOP.v_ref = ACP.INV.RATE.Vph_pk_gi;
		}
		else
			CTRL.INV.DROOP.v_ref = ACP.INV.RATE.Vph_pk;
	}

	if ( CTRL.INV.operation_mode == PARAM_OPERATION_MODE_IS ) // operation_mode: IS(1)
		CTRL.INV.DROOP.power_fbk_enb = ON;
	else
	{
		if ( INVERTER.uStatus != SYS_INV_RUN )
			CTRL.INV.DROOP.power_fbk_enb = ON;
		else
			CTRL.INV.DROOP.power_fbk_enb = OFF;
	}

	if ( CTRL.INV.DROOP.power_fbk_enb )
		a = ACP.INV.p_f_lim * INV_3 / CTRL.INV.DROOP.v_ref * 2.;
	else
		a = CTRL.INV.PQC.p_ref * INV_3 * INVCTRL.EPeakInverse * 2.;

	/*
	 * 151105 CTRL.INV_VI.T_SEC(VI oneshot)이 진행되는 동안에는 TR Z 가 보상되지 않도록 요청
	 */
	if(INVERTER.bRemove_Z_TR)
		b = 0;
	else
		b = a * ACP.TR.RATE.Z;

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2(&CTRL.INV.DROOP.FILTER.iir_Z_tr, b);
	}
	else
	{
		IIR1_Filter2_IS(&CTRL.INV.DROOP.FILTER.iir_Z_tr, b);
	}

	CTRL.INV.DROOP.v_ref_tr = CTRL.INV.DROOP.v_ref + CTRL.INV.DROOP.FILTER.iir_Z_tr.fOut;
}

void CTRL_INV_DROOP_UpdateRamp(void)
{
	float a;
	a = PARAM_VAL(V_DROOP_RAMP) * ACP.INV.RATE.Vph_pk * CC_tsSample; // 10 p.u.
	RAMP_SetDelta(&CTRL.INV.DROOP.RAMP.v_ref, a);

}

void CTRL_INV_DROOP_GI_V_Update()
{
	// Apply Limit
	if( PRM_PCS[GI_V_REF].iValue < ACP.INV.RATE.Vll * 0.89 )
		PRM_PCS[GI_V_REF].iValue = ACP.INV.RATE.Vll;

	if( PRM_PCS[GI_V_REF].iValue > ACP.INV.RATE.Vll * 1.11 )
		PRM_PCS[GI_V_REF].iValue = ACP.INV.RATE.Vll;

	ACP.INV.RATE.Vph_pk_gi = PRM_PCS[GI_V_REF].iValue * INV_SQRT3 * SQRT2;
}

void CTRL_INV_DROOP_UpdateParameter(void)
{
	float a;
	float dr_freq, dr_volt;

	CTRL.INV.DROOP.enb = PRM_PCS[DROOP_ENB].iValue; // DROOP DISABLE(0), ENABLE(1)

	//-151108-순천- dr_freq = PARAM_VAL(DROOP_FREQ) * 0.01;
	dr_freq = PARAM_VAL(DROOP_FREQ) * 0.001;
	CTRL.INV.DROOP.Omega_ref = (1.- (dr_freq)) * ACP.INV.RATE.Omega;
	CTRL.INV.DROOP.GAIN.M = ( ACP.INV.RATE.Omega - CTRL.INV.DROOP.Omega_ref ) / ACP.INV.RATE.P3;

	dr_volt = PARAM_VAL(DROOP_VOLT) * 0.01;
	CTRL.INV.DROOP.Voltage_ref = (1. - (dr_volt)) * ACP.INV.RATE.Vph_pk;
	CTRL.INV.DROOP.GAIN.N = ( ACP.INV.RATE.Vph_pk - CTRL.INV.DROOP.Voltage_ref ) / ACP.INV.RATE.Q3;

	ACP.INV.RATE.Z = ACP.INV.RATE.Vll * ACP.INV.RATE.Vll / ACP.INV.RATE.P3;

	a = PARAM_VAL(DROOP_TR_Z) * 0.01;
	ACP.TR.RATE.Z = ACP.INV.RATE.Z * a;

	if( PRM_PCS[CTRL_INV_IIR1ST_P].iValue <= 0 )
		PRM_PCS[CTRL_INV_IIR1ST_P].iValue = 30;
	if( PRM_PCS[CTRL_INV_IIR1ST_Q].iValue <= 0 )
		PRM_PCS[CTRL_INV_IIR1ST_Q].iValue = 30;

	// 151215
	IIR1_UpdateCoeff(&ACP.INV.FILTER.iir1st_p, CC_tsSample, PARAM_VAL(CTRL_INV_IIR1ST_P));
	IIR1_UpdateCoeff(&ACP.INV.FILTER.iir1st_q, CC_tsSample, PARAM_VAL(CTRL_INV_IIR1ST_Q));
}

void CTRL_INV_DROOP_1Sec_func()
{
	//CTRL_INV_DROOP_Decrease_w();
}
void CTRL_INV_DROOP_100ms_func()
{
	CTRL_INV_DROOP_Decrease_w();
}
void CTRL_INV_DROOP_Decrease_w()
{
	if( GenBlock.bApply_w_ramp )
	{
#if 0 /* T1 */
		CTRL.INV.DROOP.Omega_ref = CTRL.INV.DROOP.Omega_ref-0.01;
		CTRL.INV.DROOP.Voltage_ref = CTRL.INV.DROOP.Voltage_ref-0.0005;
#endif
	}
}

void CTRL_INV_DROOP_Create(void)
{
	float a;
	float dr_freq, dr_volt;
	float t_sample_main, t_sample_is;

	t_sample_main = CTRL_FILTER_GetSampleTime(CTRL_CC_PERIOD);
	t_sample_is = CTRL_FILTER_GetSampleTime(IS_CCP);

	IIR1_Creation_IS(&CTRL.INV.DROOP.FILTER.iir_Z_tr, t_sample_main, 10. , t_sample_is);

	a = PARAM_VAL(V_DROOP_RAMP) * ACP.INV.RATE.Vph_pk * CC_tsSample; // 10 p.u.
	RAMP_SetDelta(&CTRL.INV.DROOP.RAMP.v_ref, a);
	RAMP_Initialize(&CTRL.INV.DROOP.RAMP.v_ref);

	CTRL.INV.DROOP.enb = PRM_PCS[DROOP_ENB].iValue; // DROOP DISABLE(0), ENABLE(1)

	ACP.INV.RATE.P3 = (float) PRM_PCS[INV_CAPACITY].iValue * 1000.;
	ACP.INV.RATE.Q3 = (float) PRM_PCS[INV_CAPACITY].iValue * 1000.;

	//-151108 dr_freq = PARAM_VAL(DROOP_FREQ) * 0.01;
	dr_freq = PARAM_VAL(DROOP_FREQ) * 0.001;
	CTRL.INV.DROOP.Omega_ref = (1-dr_freq) * ACP.INV.RATE.Omega;
	CTRL.INV.DROOP.GAIN.M = ( ACP.INV.RATE.Omega - CTRL.INV.DROOP.Omega_ref ) / ACP.INV.RATE.P3;

	dr_volt = PARAM_VAL(DROOP_VOLT) * 0.01;
//	dr_volt = 0.05;
	CTRL.INV.DROOP.Voltage_ref = (1-dr_volt) * ACP.INV.RATE.Vph_pk;
	CTRL.INV.DROOP.GAIN.N = ( ACP.INV.RATE.Vph_pk - CTRL.INV.DROOP.Voltage_ref ) / ACP.INV.RATE.Q3;


	// 151215 30->Parameter->30
	//-IIR1_Creation_IS(&ACP.INV.FILTER.iir1st_p, t_sample_main, 30 , t_sample_is);
	//-IIR1_Creation_IS(&ACP.INV.FILTER.iir1st_q, t_sample_main, 30 , t_sample_is);

	a = PARAM_VAL(DROOP_TR_Z) * 0.01;
	ACP.TR.RATE.Z = ACP.INV.RATE.Z * a;

	CTRL_INV_DROOP_UpdateParameter();
	CTRL_INV_DROOP_UpdateRamp();
	CTRL_INV_DROOP_GI_V_Update();

#if 1 /* T2 */
	//151215
	RAMP_SetDelta(&CTRL.INV.DROOP.RAMP.p_ref, 0.000001);
	RAMP_SetDelta(&CTRL.INV.DROOP.RAMP.q_ref, 0.000001);
#endif
}

