/*
 * CTRL_INV_VUC.c
 *
 *  Created on: 2014. 5. 16.
 *      Author: Seth
 *      Voltage unbalance compensation
 */
#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "CTRL_INV_VUC.h"
#include "CC.h"
#include "PI.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "filter.h"
#include "CTRL_FILTER.h"

void CTRL_INV_VUC_Proceed(void)
{
	volatile float a, b;
	volatile iir1st *pIIR1;
	volatile PICon *pPICon;
	volatile PIArg *pPIArg;

	if ( CTRL.INV.VUC.g_enb && !CTRL.INV.grid_connected_enb )
	{
		CTRL.INV.VUC.enb = ON;
	}
	else
	{
		CTRL.INV.VUC.enb = OFF;
	}
/*
	if (CTRL.INV.VUC.filter_coeff_change_enb)
	{
		IIR1_ChangeCoefficient(&CTRL.INV.VUC.FILTER.iir_pos);
		IIR1_ChangeCoefficient(&CTRL.INV.VUC.FILTER.iir_neg);
		CTRL.INV.VUC.filter_coeff_change_enb = OFF;
	}
*/
	if ( CTRL.INV.VUC.enb )
	{
		a = sqrt(ACP.INV.vds_p * ACP.INV.vds_p + ACP.INV.vqs_p * ACP.INV.vqs_p);

		b = sqrt(ACP.INV.vds_n * ACP.INV.vds_n + ACP.INV.vqs_n * ACP.INV.vqs_n);
		if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
		{
			IIR1_Filter2(&CTRL.INV.VUC.FILTER.iir_pos, a);
			IIR1_Filter2(&CTRL.INV.VUC.FILTER.iir_neg, b);
		}
		else
		{
			IIR1_Filter2_IS(&CTRL.INV.VUC.FILTER.iir_pos, a);
			IIR1_Filter2_IS(&CTRL.INV.VUC.FILTER.iir_neg, b);
		}

		CTRL.INV.VUC.vuf = b / a;

		CTRL.INV.VUC.PI_UF.ARG.fErr = - CTRL.INV.VUC.vuf;
		PICON_PI_ONLY_RunM((&CTRL.INV.VUC.PI_UF.CORE), (&CTRL.INV.VUC.PI_UF.ARG));

		CTRL.INV.VUC.vds_compen = ACP.INV.vds_n * CTRL.INV.VUC.PI_UF.CORE.fOut;
		CTRL.INV.VUC.vqs_compen = ACP.INV.vqs_n * CTRL.INV.VUC.PI_UF.CORE.fOut;
	}
	else
	{
		CTRL.INV.VUC.vds_compen = 0.;
		CTRL.INV.VUC.vqs_compen = 0.;
		CTRL.INV.VUC.vuf = 0.;
		PICON_Initialize(&CTRL.INV.VUC.PI_UF.CORE);
	}
}

void CTRL_INV_VUC_UpdateParameterPIGain(void)
{
	CTRL.INV.VUC.PI_UF.K.fP = PARAM_VAL(VUC_PI_KP);
	CTRL.INV.VUC.PI_UF.K.fIT = PARAM_VAL(VUC_PI_KI) * CC_tsSample;
	CTRL.INV.VUC.PI_UF.K.fA = 1. / CTRL.INV.SYNC.PI_W.K.fP;
}

void CTRL_INV_VUC_UpdateParameter(void)
{
	CTRL.INV.VUC.g_enb = PARAM_VAL(VUC_ENB);
	CTRL_INV_VUC_UpdateParameterPIGain();
//	CTRL.INV.VUC.filter_coeff_change_enb = ON;
}

void CTRL_INV_VUC_Create(void)
{
	float t_sample_main, t_sample_is;

	t_sample_main = CTRL_FILTER_GetSampleTime(CTRL_CC_PERIOD);
	t_sample_is = CTRL_FILTER_GetSampleTime(IS_CCP);

	IIR1_Creation_IS(&CTRL.INV.VUC.FILTER.iir_pos, t_sample_main, PARAM_VAL(VUC_PEAK_LPF), t_sample_is);
	IIR1_Creation_IS(&CTRL.INV.VUC.FILTER.iir_neg, t_sample_main, PARAM_VAL(VUC_PEAK_LPF), t_sample_is);

	PICon_(&CTRL.INV.VUC.PI_UF.CORE);
	PIArg_(&CTRL.INV.VUC.PI_UF.ARG, &CTRL.INV.VUC.PI_UF.K);
	CTRL.INV.VUC.PI_UF.ARG.fAlpha = 1; // 1 ==> PI 제어, 0 ==> IP 제어
	CTRL.INV.VUC.PI_UF.ARG.fMax = 2.5;
	CTRL.INV.VUC.PI_UF.ARG.fMin = - 2.5;

	CTRL.INV.VUC.vds_compen = 0.;
	CTRL.INV.VUC.vqs_compen = 0.;
	CTRL.INV.VUC.vuf = 0.;
	CTRL.INV.VUC.enb = OFF;
//	CTRL.INV.VUC.filter_coeff_change_enb = OFF;

	CTRL_INV_VUC_UpdateParameter();
}

#if 0
/*
 * call: cint11()
 * when: CC_bVoltageFilterShadowing == 1
 * call: PARAM_UpdateALL()
 * when: CTRL_VOLT_LPF 파라미터 변경시.
 *
 */
void CTRL_INV_VUC_UpdateFilterCoefficient(void)
{
	if (!CTRL.INV.VUC.filter_coeff_shadow_enb) return;

	IIR1_ChangeCoefficient(&CTRL.INV.VUC.FILTER.iir_pos);
	IIR1_ChangeCoefficient(&CTRL.INV.VUC.FILTER.iir_neg);
	CTRL.INV.VUC.filter_coeff_shadow_enb = OFF;
}

/*
 * call: PARAM_UpdateALL()
 * when: CTRL_CC_PERIOD 파라미터 변경시
 */
void CTRL_INV_VUC_CalcFilterShadow(void)
{
	IIR1_CalcShadow(&CTRL.INV.VUC.FILTER.iir_pos, CC_tsSample, PARAM_VAL(VUC_PEAK_LPF));
	IIR1_CalcShadow(&CTRL.INV.VUC.FILTER.iir_neg, CC_tsSample, PARAM_VAL(VUC_PEAK_LPF));
	CTRL.INV.VUC.filter_coeff_shadow_enb = ON;
}

#endif
