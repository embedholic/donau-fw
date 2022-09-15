/*
 * CTRL_FILTER.c
 *
 *  Created on: 2014. 6. 10.
 *      Author: Seth Oh
 */

#include "CTRL_FILTER.h"
#include "CC.h"
#include "PI.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "filter.h"
#include "CTRL_INV_VI.h"

float CTRL_FILTER_GetSampleTime(Uns Period_Para_ID)
{
	volatile Uns uPrd;
	volatile float t_switching;
	volatile float t_sample;

	uPrd = PRM_PCS[Period_Para_ID].iValue;

	if (uPrd < CC_PERIOD_MIN)
		uPrd = CC_PERIOD_MIN;
	if (uPrd > CC_PERIOD_MAX)
		uPrd = CC_PERIOD_MAX;

	t_switching = uPrd * CC_MICRO_UNIT;
	t_sample = t_switching;
#if DOUBLE_CONTROL == 1
	t_sample = t_switching * 0.5;
#endif

	return t_sample;
}

void CTRL_FILTER_Create(void)
{
	float t_sample_main, t_sample_is;
	int i;
	Uns cutoff;

	t_sample_main = CTRL_FILTER_GetSampleTime(CTRL_CC_PERIOD);
	t_sample_is = CTRL_FILTER_GetSampleTime(IS_CCP);

	IIR1_Creation_IS(&INVCTRL.fltEde, t_sample_main, PRM_PCS[CTRL_VOLT_LPF].iValue, t_sample_is);
	IIR1_Creation_IS(&INVCTRL.fltEqe, t_sample_main, PRM_PCS[CTRL_VOLT_LPF].iValue, t_sample_is);
	IIR1_Creation_IS(&INVCTRL.fltEqe2nd, t_sample_main, CC_EQE_2ND_FIL_CUTOFF_HZ, t_sample_is);
	IIR1_Creation_IS(&INVCTRL.fltIqe, t_sample_main, CC_IQE_FLT_CUTOFF_HZ, t_sample_is);

//++JCNET sag controller filter
    IIR1_Creation_IS(&SCCTRL.fltEde, t_sample_main, PRM_PCS[CTRL_VOLT_LPF].iValue, t_sample_is);
    IIR1_Creation_IS(&SCCTRL.fltEqe, t_sample_main, PRM_PCS[CTRL_VOLT_LPF].iValue, t_sample_is);
    IIR1_Creation_IS(&SCCTRL.fltEqe2nd, t_sample_main, CC_EQE_2ND_FIL_CUTOFF_HZ, t_sample_is);
    IIR1_Creation_IS(&SCCTRL.fltEne, t_sample_main, PRM_PCS[CTRL_VOLT_LPF].iValue, t_sample_is);
    cutoff = PARAM_VAL(CTRL_FREQ_CUTOFF_HZ);
    IIR1_Creation_IS(&SCCTRL.fltFreq, t_sample_main, (float)cutoff, t_sample_is);

//--

	IIR1_Creation_IS(&EXCTRL.fltEde, t_sample_main, PRM_PCS[CTRL_VOLT_LPF].iValue, t_sample_is);
	IIR1_Creation_IS(&EXCTRL.fltEqe, t_sample_main, PRM_PCS[CTRL_VOLT_LPF].iValue, t_sample_is);

	IIR1_Creation_IS(&EXCTRL.fltEde_byp, t_sample_main, PRM_PCS[CTRL_VOLT_LPF].iValue, t_sample_is);
	IIR1_Creation_IS(&EXCTRL.fltEqe_byp, t_sample_main, PRM_PCS[CTRL_VOLT_LPF].iValue, t_sample_is);

	IIR1_Creation_IS(&EXCTRL.fltEde_gen, t_sample_main, PRM_PCS[CTRL_VOLT_LPF].iValue, t_sample_is);
	IIR1_Creation_IS(&EXCTRL.fltEqe_gen, t_sample_main, PRM_PCS[CTRL_VOLT_LPF].iValue, t_sample_is);

	IIR1_Creation_IS(&BATCTRL.fltDCLinkV, t_sample_main, CC_DC_V_FLT_CUTOFF_HZ, t_sample_is);
	IIR1_Creation_IS(&BATCTRL.fltDCLinkV2nd, t_sample_main, CC_DC_V_FLT2ND_CUTOFF_HZ, t_sample_is);

	IIR1_Creation_IS(&BATCTRL.fltDCBattVForCC, t_sample_main, CC_DC_V_FOR_CC_FLT_CUTOFF_HZ, t_sample_is);
	IIR1_Creation_IS(&BATCTRL.fltDCBattV, t_sample_main, CC_DC_V_FLT_CUTOFF_HZ, t_sample_is);
	IIR1_Creation_IS(&BATCTRL.fltDCBattV_2nd, t_sample_main, CC_DC_V_FLT2ND_CUTOFF_HZ, t_sample_is);

	IIR1_Creation_IS(&BATCTRL.fltI, t_sample_main, CC_PV_I_FLT_CUTOFF_HZ, t_sample_is);
	IIR1_Creation_IS(&BATCTRL.fltI2, t_sample_main, CC_PV_I_FLT2_CUTOFF_HZ, t_sample_is);

	IIR1_Creation_IS(&INVCTRL.fltPout, t_sample_main, CC_POUT_FLT_CUTOFF_HZ, t_sample_is);
	IIR1_Creation_IS(&INVCTRL.fltPout2nd, t_sample_main, CC_POUT_FLT_CUTOFF_HZ, t_sample_is);

	/*
	 * Filter line item creation
	 */
	cutoff = PARAM_VAL(CTRL_GRID_FS_RMS_CUTOFF_HZ);
	for (i = 0; i < CC_LINE_NUM; i++)
	{
		IIR1_Creation_IS(&EXCTRL.fltFsGridV[i], t_sample_main, cutoff, t_sample_is);
		IIR1_Creation_IS(&EXCTRL.fltFsBypV[i], t_sample_main, cutoff, t_sample_is);
		IIR1_Creation_IS(&EXCTRL.fltFsGenV[i], t_sample_main, cutoff, t_sample_is);
		IIR1_Creation_IS(&EXCTRL.fltFsBypI[i], t_sample_main, PRM_PCS[BESS73].iValue /* CC_BYP_I_CUTOFF_HZ */, t_sample_is);
	}

	cutoff = PARAM_VAL(CTRL_INV_FS_RMS_CUTOFF_HZ);
	for (i = 0; i < CC_LINE_NUM; i++)
	{
		IIR1_Creation_IS(&INVCTRL.fltFsConV[i], t_sample_main, cutoff, t_sample_is);
		IIR1_Creation_IS(&INVCTRL.fltFsConVp[i], t_sample_main, cutoff, t_sample_is);
	}
	cutoff = PARAM_VAL(CTRL_FREQ_CUTOFF_HZ);
	IIR1_Creation_IS(&INVCTRL.fltFreq, t_sample_main, cutoff, t_sample_is);
	IIR1_Creation_IS(&EXCTRL.fltFreq, t_sample_main, cutoff, t_sample_is);
	IIR1_Creation_IS(&EXCTRL.fltFreq_Byp, t_sample_main, cutoff, t_sample_is);
	IIR1_Creation_IS(&EXCTRL.fltFreq_Gen, t_sample_main, cutoff, t_sample_is);

	IIR1_Creation_IS(&GRIDTHETA_AI.fltFreqW, t_sample_main, CC_GRID_FREQ_OMEGA_CUTOFF, t_sample_is);
	IIR1_Creation_IS(&GRID_BYP_THETA.fltFreqW, t_sample_main, CC_GRID_FREQ_OMEGA_CUTOFF, t_sample_is);

	//130622
	IIR1_Creation_IS(&EXCTRL.fltAcPowerP, t_sample_main, PARAM_VAL(BATT_PCC_PQ_CUTOFF_HZ), t_sample_is);
	IIR1_Creation_IS(&EXCTRL.fltAcPowerQ, t_sample_main, PARAM_VAL(BATT_PCC_PQ_CUTOFF_HZ), t_sample_is);

	IIR1_Creation_IS(&EXCTRL.fltAcPowerP_2nd, t_sample_main, CC_ACPOWER_CUTOFF_HZ_2ND, t_sample_is);
	IIR1_Creation_IS(&EXCTRL.fltAcPowerQ_2nd, t_sample_main, CC_ACPOWER_CUTOFF_HZ_2ND, t_sample_is);

}

/*
 * call: CC_Create()
 * call: PARAM_UpdateAll()
 * when: GRID_RATED_FREQ 또는 CC_PERIOD 변경 시
 */
void CTRL_FILTER_UpdateAPS(void)
{
	float t_sample_main, t_sample_is;
	float Freq;

	t_sample_main = CTRL_FILTER_GetSampleTime(CTRL_CC_PERIOD);
	t_sample_is = CTRL_FILTER_GetSampleTime(IS_CCP);
	Freq = PRM_PCS[GRID_RATED_FREQ].iValue;

	APS_Creation_IS(&EXCTRL.fltEaDelay90, t_sample_main, APS_DEGREE_90, Freq, t_sample_is);
	APS_Creation_IS(&EXCTRL.fltEbDelay90, t_sample_main, APS_DEGREE_90, Freq, t_sample_is);
	APS_Creation_IS(&EXCTRL.fltEcDelay90, t_sample_main, APS_DEGREE_90, Freq, t_sample_is);

	APS_Creation_IS(&INVCTRL.fltEaDelay90, t_sample_main, APS_DEGREE_90, Freq, t_sample_is);
	APS_Creation_IS(&INVCTRL.fltEbDelay90, t_sample_main, APS_DEGREE_90, Freq, t_sample_is);
	APS_Creation_IS(&INVCTRL.fltEcDelay90, t_sample_main, APS_DEGREE_90, Freq, t_sample_is);

	APS_Creation_IS(&CTRL.INV.VI.fltIqsDelay90, t_sample_main, APS_DEGREE_90, Freq, t_sample_is);
}

