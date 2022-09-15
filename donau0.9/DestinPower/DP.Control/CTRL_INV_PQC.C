/*
 * CTRL_INV_PQC.C
 *
 *  Created on: 2014. 4. 3.
 *      Author: Seth Oh
 */

#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "CTRL_INV_PQC.h"
#include "CC.h"
#include "parameter.h"
#include "prm_pcs.h"

extern	AC_Panel	ACP;
extern	Controller	CTRL;

void CTRL_INV_PQC_UpdateParameterPIGain(void)
{
	CTRL.INV.PQC.PI_Q.K.fP = CTRL.INV.DROOP.GAIN.N * PARAM_VAL(BATT_CHG_DCHG_MODE); // 0.01;
	CTRL.INV.PQC.PI_Q.K.fIT = PARAM_VAL(PQC_Q_I_GAIN) * CC_tsSample * 0.001;
	CTRL.INV.PQC.PI_Q.K.fA = 1. / CTRL.INV.PQC.PI_Q.K.fP;

	CTRL.INV.PQC.PI_P.K.fP = CTRL.INV.DROOP.GAIN.M * PARAM_VAL(BATT_CONST_PWR_CHG); // 0.01;
	CTRL.INV.PQC.PI_P.K.fIT = PARAM_VAL(BATT_END_COND_HOLD_TIME) * CC_tsSample * 0.001;
	CTRL.INV.PQC.PI_P.K.fA = 1. / CTRL.INV.PQC.PI_P.K.fP;
}

void CTRL_INV_PQC_UpdateParameterRamp(void)
{
	float omega, a;

	omega = 2 * PI * PARAM_VAL(GRID_RATED_FREQ);
	a = PARAM_VAL(BATT_CONST_PWR_DCHG) * ACP.INV.RATE.Vph_pk * CC_tsSample; // 0.1 p.u.
	RAMP_SetDelta(&CTRL.INV.PQC.RAMP.v_ref, a);

	a = PARAM_VAL(BATT_DCC_P_GAIN) * omega * CC_tsSample; // 0.1 p.u.
	RAMP_SetDelta(&CTRL.INV.PQC.RAMP.w_ref, a);
}

void CTRL_INV_PQC_UpdateParameter(void)
{
	float Lf, inv_freq, omega, a;

	CTRL.INV.operation_mode = (CMODE)PRM_PCS[GC_IS_MODE].iValue; //  GC(0), IS(1), GC+IS(2)
	if ( CTRL.INV.operation_mode == PARAM_OPERATION_MODE_IS )
	{
		CTRL.INV.ctrl_mode = PARAM_VSI_PR;
		CTRL.INV.ctrl_mode_change_enb = ON;
	}
	else
	{
		CTRL.INV.ctrl_mode = PRM_PCS[CONTROLLER_SEL].iValue;
		CTRL.INV.ctrl_mode_change_enb = ON;
	}

	CTRL.INV.PQC.PI_Q.ARG.fAlpha = 1.; // 1 ==> PI 제어, 0 ==> IP 제어
	CTRL.INV.PQC.PI_Q.ARG.fMax = 3. * ACP.INV.RATE.Q3 * CTRL.INV.DROOP.GAIN.N; // 정격의 30%
	CTRL.INV.PQC.PI_Q.ARG.fMin = - CTRL.INV.PQC.PI_Q.ARG.fMax;

#if OP_MODE == STBLN_300K
	Lf = PARAM_VAL(CTRL_LI) * 1e-6 / 2.; /* TODO 190516 2값 확인. 일단 2로 다 테스트 되어 1로 바꾸지는 않음. */
#elif OP_MODE == STBLN_500K
	Lf = PARAM_VAL(CTRL_LI) * 1e-6 / x.;
#elif OP_MODE == STBLN_1250K
	Lf = PARAM_VAL(CTRL_LI) * 1e-6 / x.;
#else
		Lf = PARAM_VAL(CTRL_LI) * 1e-6;
#endif

	inv_freq = 	PARAM_VAL(GRID_RATED_FREQ);
	omega = 2 * PI * inv_freq;
	CTRL.INV.PQC.q_compen_coeff = omega * Lf / ACP.INV.RATE.Vph;
	INVCTRL.Li = Lf;

	a = PARAM_VAL(BATT_CONST_PWR_DCHG) * ACP.INV.RATE.Vph_pk * CC_tsSample; // 0.1 p.u.
	RAMP_SetDelta(&CTRL.INV.PQC.RAMP.v_ref, a);

	CTRL.INV.PQC.PI_P.ARG.fAlpha = 1.; // 1 ==> PI 제어, 0 ==> IP 제어
	CTRL.INV.PQC.PI_P.ARG.fMax = 3. * ACP.INV.RATE.P3 * CTRL.INV.DROOP.GAIN.M; // 정격의 30%
	CTRL.INV.PQC.PI_P.ARG.fMin = - CTRL.INV.PQC.PI_P.ARG.fMax;
	CTRL.INV.PQC.p_compen_coeff = omega * omega * Lf / (ACP.PCC.RATE.Vph * ACP.INV.RATE.Vph * 360.);

	a = PARAM_VAL(BATT_DCC_P_GAIN) * omega * CC_tsSample; // 0.1 p.u.
	RAMP_SetDelta(&CTRL.INV.PQC.RAMP.w_ref, a);

	CTRL.INV.PQC.PI_Q.K.fP = CTRL.INV.DROOP.GAIN.N * PARAM_VAL(BATT_CHG_DCHG_MODE); // 0.01;
	CTRL.INV.PQC.PI_Q.K.fIT = PARAM_VAL(PQC_Q_I_GAIN) * CC_tsSample * 0.001;
	CTRL.INV.PQC.PI_Q.K.fA = 1. / CTRL.INV.PQC.PI_Q.K.fP;

	CTRL.INV.PQC.PI_P.K.fP = CTRL.INV.DROOP.GAIN.M * PARAM_VAL(BATT_CONST_PWR_CHG); // 0.01;
	CTRL.INV.PQC.PI_P.K.fIT = PARAM_VAL(BATT_END_COND_HOLD_TIME) * CC_tsSample * 0.001;
	CTRL.INV.PQC.PI_P.K.fA = 1. / CTRL.INV.PQC.PI_P.K.fP;

}

void CTRL_INV_PQC_Create(void)
{
	CTRL.INV.PQC.enb = TRUE;

	PICon_(&CTRL.INV.PQC.PI_Q.CORE);
	PIArg_(&CTRL.INV.PQC.PI_Q.ARG, &CTRL.INV.PQC.PI_Q.K);

	RAMP_Initialize(&CTRL.INV.PQC.RAMP.v_ref);

	PICon_(&CTRL.INV.PQC.PI_P.CORE);
	PIArg_(&CTRL.INV.PQC.PI_P.ARG, &CTRL.INV.PQC.PI_P.K);
	RAMP_Initialize(&CTRL.INV.PQC.RAMP.w_ref);

	CTRL_INV_PQC_UpdateParameter();
	CTRL_INV_PQC_UpdateParameterRamp();
}


