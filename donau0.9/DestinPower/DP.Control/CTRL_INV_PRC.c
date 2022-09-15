/*
 * CTRL_INV_PRC.c
 *
 *  Created on: 2014. 4. 3.
 *      Author: Seth Oh
 */


#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "CTRL_INV_PRC.h"
#include "CC.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "ramp.h"
#include "CTRL_INV_VUC.h"
#include "CTRL_FILTER.h"
#include "SYSTEM.h"
#include "MCCB.h"
#include "CTRL_GEN.h"

extern	AC_Panel	ACP;
extern	Controller	CTRL;

//PResonatBlock PRC_Ids;
//PResonatBlock PRC_Iqs;
//PResonatBlock PRC_Vds;
//PResonatBlock PRC_Vqs;
void CTRL_INV_PRC_Create(void)
{
	CTRL.INV.ctrl_mode_change_enb = OFF;
	CTRL_INV_PRC_UpdateParameter();
}

FILTER_MODE PRC_APPLAY_GAIN;

#pragma CODE_SECTION (CTRL_INV_PRC_ApplyGC_GI_PI_Gain, "dp_ctrl")
void CTRL_INV_PRC_ApplyGC_GI_PI_Gain(void)
{
	if((CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1) && (INVERTER.uStatus == SYS_INV_RUN))
	{
		CTRL.INV.PRC_Ids.GAIN.Kp    = PARAM_VAL(GC_CC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_P_GAIN);
		CTRL.INV.PRC_Ids.GAIN.Ki	= PARAM_VAL(GC_CC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K1st  = PARAM_VAL(GC_CC_PR_I_GAIN_1ST)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K5th  = PARAM_VAL(GC_CC_PR_I_GAIN_5TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K7th  = PARAM_VAL(GC_CC_PR_I_GAIN_7TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K11th = PARAM_VAL(GC_CC_PR_I_GAIN_11TH)	* PARAM_VAL(CTRL_CC_I_GAIN);

		CTRL.INV.PRC_Iqs.GAIN.Kp    = PARAM_VAL(GC_CC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_P_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.Ki	= PARAM_VAL(GC_CC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K1st  = PARAM_VAL(GC_CC_PR_I_GAIN_1ST)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K5th  = PARAM_VAL(GC_CC_PR_I_GAIN_5TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K7th  = PARAM_VAL(GC_CC_PR_I_GAIN_7TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K11th = PARAM_VAL(GC_CC_PR_I_GAIN_11TH)	* PARAM_VAL(CTRL_CC_I_GAIN);

		PRC_APPLAY_GAIN = FILTER_MODE_GC_SwF_1;
	}
	else if(CTRL.INV.filter_mode == FILTER_MODE_IS_SwF_2
			&& ( INVERTER.uStatus == SYS_INV_ISLANDING || INVERTER.uStatus == SYS_INV_RE_SYNC
					|| CTRL.INV.SEAMLESS.pcc_blackout_enb  ))
	{
		CTRL.INV.PRC_Ids.GAIN.Kp	= PARAM_VAL(GI_CC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_P_GAIN);
		CTRL.INV.PRC_Ids.GAIN.Ki	= PARAM_VAL(GI_CC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K1st	= PARAM_VAL(GI_CC_PR_I_GAIN_1ST)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K5th	= PARAM_VAL(GI_CC_PR_I_GAIN_5TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K7th	= PARAM_VAL(GI_CC_PR_I_GAIN_7TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K11th	= PARAM_VAL(GI_CC_PR_I_GAIN_11TH)	* PARAM_VAL(CTRL_CC_I_GAIN);

		CTRL.INV.PRC_Iqs.GAIN.Kp	= PARAM_VAL(GI_CC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_P_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.Ki	= PARAM_VAL(GI_CC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K1st	= PARAM_VAL(GI_CC_PR_I_GAIN_1ST)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K5th	= PARAM_VAL(GI_CC_PR_I_GAIN_5TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K7th	= PARAM_VAL(GI_CC_PR_I_GAIN_7TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K11th	= PARAM_VAL(GI_CC_PR_I_GAIN_11TH)	* PARAM_VAL(CTRL_CC_I_GAIN);

		CTRL.INV.PRC_Vds.GAIN.Kp	= PARAM_VAL(GI_VC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_P_GAIN);
		CTRL.INV.PRC_Vds.GAIN.Ki    = PARAM_VAL(GI_VC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vds.GAIN.K1st	= PARAM_VAL(GI_VC_PR_I_GAIN_1ST)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vds.GAIN.K5th	= PARAM_VAL(GI_VC_PR_I_GAIN_5TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vds.GAIN.K7th	= PARAM_VAL(GI_VC_PR_I_GAIN_7TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vds.GAIN.K11th	= PARAM_VAL(GI_VC_PR_I_GAIN_11TH)	* PARAM_VAL(CTRL_CC_I_GAIN);

		CTRL.INV.PRC_Vqs.GAIN.Kp	= PARAM_VAL(GI_VC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_P_GAIN);
		CTRL.INV.PRC_Vqs.GAIN.Ki    = PARAM_VAL(GI_VC_PR_P_GAIN)		* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vqs.GAIN.K1st	= PARAM_VAL(GI_VC_PR_I_GAIN_1ST)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vqs.GAIN.K5th	= PARAM_VAL(GI_VC_PR_I_GAIN_5TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vqs.GAIN.K7th	= PARAM_VAL(GI_VC_PR_I_GAIN_7TH)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vqs.GAIN.K11th	= PARAM_VAL(GI_VC_PR_I_GAIN_11TH)	* PARAM_VAL(CTRL_CC_I_GAIN);

		PRC_APPLAY_GAIN = FILTER_MODE_IS_SwF_2;
	}
	else
	{
		CTRL.INV.PRC_Ids.GAIN.Kp	= PARAM_VAL(ICON_I_GAIN)		* PARAM_VAL(CTRL_CC_P_GAIN);
		CTRL.INV.PRC_Ids.GAIN.Ki	= PARAM_VAL(ICON_I_GAIN)		* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K1st	= PARAM_VAL(BATT_DCHG_TIME)		* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K5th	= PARAM_VAL(BATT_CUTOFF_VOLTAGE_DCHG)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K7th	= PARAM_VAL(BATT_VDCREF_CHG)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Ids.GAIN.K11th	= PARAM_VAL(BATT_VDCREF_DCHG)	* PARAM_VAL(CTRL_CC_I_GAIN);

		CTRL.INV.PRC_Iqs.GAIN.Kp	= PARAM_VAL(ICON_I_GAIN)		* PARAM_VAL(CTRL_CC_P_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.Ki	= PARAM_VAL(ICON_I_GAIN)		* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K1st	= PARAM_VAL(BATT_DCHG_TIME)		* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K5th	= PARAM_VAL(BATT_CUTOFF_VOLTAGE_DCHG)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K7th	= PARAM_VAL(BATT_VDCREF_CHG)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Iqs.GAIN.K11th	= PARAM_VAL(BATT_VDCREF_DCHG)	* PARAM_VAL(CTRL_CC_I_GAIN);

		CTRL.INV.PRC_Vds.GAIN.Kp	= PARAM_VAL(VCON_P_GAIN)		* PARAM_VAL(CTRL_CC_P_GAIN);
		CTRL.INV.PRC_Vds.GAIN.Ki    = PARAM_VAL(VCON_P_GAIN)			* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vds.GAIN.K1st	= PARAM_VAL(BATT_START_IDLE_TIME)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vds.GAIN.K5th	= PARAM_VAL(BATT_CYCLING_CNT)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vds.GAIN.K7th	= PARAM_VAL(BATT_INIT_STATE)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vds.GAIN.K11th	= PARAM_VAL(BATT_CHG_TIME)		* PARAM_VAL(CTRL_CC_I_GAIN);

		CTRL.INV.PRC_Vqs.GAIN.Kp	= PARAM_VAL(VCON_P_GAIN)		* PARAM_VAL(CTRL_CC_P_GAIN);
		CTRL.INV.PRC_Vqs.GAIN.Ki    = PARAM_VAL(VCON_P_GAIN)			* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vqs.GAIN.K1st	= PARAM_VAL(BATT_START_IDLE_TIME)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vqs.GAIN.K5th	= PARAM_VAL(BATT_CYCLING_CNT)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vqs.GAIN.K7th	= PARAM_VAL(BATT_INIT_STATE)	* PARAM_VAL(CTRL_CC_I_GAIN);
		CTRL.INV.PRC_Vqs.GAIN.K11th	= PARAM_VAL(BATT_CHG_TIME)		* PARAM_VAL(CTRL_CC_I_GAIN);
	}
}

#pragma CODE_SECTION (CTRL_INV_PRC_UpdateParameter, "dp_ctrl")
void CTRL_INV_PRC_UpdateParameter(void)
{
	float inv_freq;
	float cc_cut_off_freq;
	float is_vc_cut_off_freq, gc_vc_cut_off_freq;
	float har_freq;
	float t_sample_main, t_sample_is;
	int n;

	t_sample_main = CTRL_FILTER_GetSampleTime(CTRL_CC_PERIOD);
	t_sample_is = CTRL_FILTER_GetSampleTime(IS_CCP);

	CTRL.INV.ctrl_mode = PRM_PCS[CONTROLLER_SEL].iValue; // CSI_dq(0), VSI_p+r(1), CSI_p+r(2)
	inv_freq = 	ACP.INV.RATE.Freq;
	gc_vc_cut_off_freq = PARAM_VAL(BATT_17);
	is_vc_cut_off_freq = PARAM_VAL(VC_PR_CUTOFF_IS);
	cc_cut_off_freq = PARAM_VAL(CC_CUT_OFF_FREQ);

	CTRL_INV_PRC_ApplyGC_GI_PI_Gain();

	n = 1;
	har_freq = inv_freq * n;
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Ids.FILTER.har_1st, t_sample_main, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Iqs.FILTER.har_1st, t_sample_main, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Vds.FILTER.har_1st, t_sample_main, gc_vc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Vqs.FILTER.har_1st, t_sample_main, gc_vc_cut_off_freq, har_freq);

	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Ids.FILTER.har_1st, t_sample_is, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Iqs.FILTER.har_1st, t_sample_is, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Vds.FILTER.har_1st, t_sample_is, is_vc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Vqs.FILTER.har_1st, t_sample_is, is_vc_cut_off_freq, har_freq);

	n = 5;
	har_freq = inv_freq * n;
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Ids.FILTER.har_5th, t_sample_main, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Iqs.FILTER.har_5th, t_sample_main, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Vds.FILTER.har_5th, t_sample_main, gc_vc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Vqs.FILTER.har_5th, t_sample_main, gc_vc_cut_off_freq, har_freq);

	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Ids.FILTER.har_5th, t_sample_is, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Iqs.FILTER.har_5th, t_sample_is, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Vds.FILTER.har_5th, t_sample_is, is_vc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Vqs.FILTER.har_5th, t_sample_is, is_vc_cut_off_freq, har_freq);

	n = 7;
	har_freq = inv_freq * n;
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Ids.FILTER.har_7th, t_sample_main, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Iqs.FILTER.har_7th, t_sample_main, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Vds.FILTER.har_7th, t_sample_main, gc_vc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Vqs.FILTER.har_7th, t_sample_main, gc_vc_cut_off_freq, har_freq);

	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Ids.FILTER.har_7th, t_sample_is, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Iqs.FILTER.har_7th, t_sample_is, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Vds.FILTER.har_7th, t_sample_is, is_vc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Vqs.FILTER.har_7th, t_sample_is, is_vc_cut_off_freq, har_freq);

	n = 11;
	har_freq = inv_freq * n;
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Ids.FILTER.har_11th, t_sample_main, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Iqs.FILTER.har_11th, t_sample_main, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Vds.FILTER.har_11th, t_sample_main, gc_vc_cut_off_freq, har_freq);
	PResonatFilter_Creation_GC(&CTRL.INV.PRC_Vqs.FILTER.har_11th, t_sample_main, gc_vc_cut_off_freq, har_freq);

	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Ids.FILTER.har_11th, t_sample_is, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Iqs.FILTER.har_11th, t_sample_is, cc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Vds.FILTER.har_11th, t_sample_is, is_vc_cut_off_freq, har_freq);
	PResonatFilter_Creation_IS(&CTRL.INV.PRC_Vqs.FILTER.har_11th, t_sample_is, is_vc_cut_off_freq, har_freq);

	CTRL.INV.PROTECTION.Idqs_MAX = 1.2 * ACP.INV.RATE.Iph_pk;
	CTRL.INV.PROTECTION.Idqs_MIN = - CTRL.INV.PROTECTION.Idqs_MAX;
}

void CTRL_INV_PRC_InverterPResonat_IS(PResonatBlock *this, float input)
{
	float K1, K5, K7, K11, x;
	PResonatFilter *pPRF;

	K1 = this->GAIN.K1st;
	K5 = this->GAIN.K5th;
	K7 = this->GAIN.K7th;
	K11 = this->GAIN.K11th;

	x = K1 * input;
	PRESONANT_Filter_IS( &this->FILTER.har_1st, x );
	x = K5 * input;
	PRESONANT_Filter_IS( &this->FILTER.har_5th, x );
	x = K7 * input;
	PRESONANT_Filter_IS( &this->FILTER.har_7th, x );
	x = K11 * input;
	PRESONANT_Filter_IS( &this->FILTER.har_11th, x );

	this->output = (this->GAIN.Kp * input) + (this->FILTER.har_1st.y) + (this->FILTER.har_5th.y)
			    + (this->FILTER.har_7th.y) + (this->FILTER.har_11th.y);
}

void CTRL_INV_PRC_InverterPResonat_GC(PResonatBlock *this, float input)
{
	float K1, K5, K7, K11, x;
	PResonatFilter *pPRF;

	K1 = this->GAIN.K1st;
	K5 = this->GAIN.K5th;
	K7 = this->GAIN.K7th;
	K11 = this->GAIN.K11th;

	x = K1 * input;
	PRESONANT_Filter_GC( &this->FILTER.har_1st, x );
	x = K5 * input;
	PRESONANT_Filter_GC( &this->FILTER.har_5th, x );
	x = K7 * input;
	PRESONANT_Filter_GC( &this->FILTER.har_7th, x );
	x = K11 * input;
	PRESONANT_Filter_GC( &this->FILTER.har_11th, x );

	this->output = (this->GAIN.Kp * input) + (this->FILTER.har_1st.y) + (this->FILTER.har_5th.y)
			    + (this->FILTER.har_7th.y) + (this->FILTER.har_11th.y);
}

void CTRL_INV_PRC_Initialize(PResonatBlock *this)
{
	PResonatFilter_Initialize( &this->FILTER.har_1st );
	PResonatFilter_Initialize( &this->FILTER.har_5th );
	PResonatFilter_Initialize( &this->FILTER.har_7th );
	PResonatFilter_Initialize( &this->FILTER.har_11th );

	CTRL.INV.DROOP.v_ref_ramp = 0.;
	CTRL.INV.DROOP.w_ref_sec = 0.;
	RAMP_Initialize(&CTRL.INV.DROOP.RAMP.v_ref);
	RAMP_Initialize(&CTRL.INV.DROOP.RAMP.p_ref);
	RAMP_Initialize(&CTRL.INV.DROOP.RAMP.q_ref);

	//+141105 계통연계 PWM - OFF 후 ON 시 OC 발생할 수 있음.때
	//this->output = 0;
}

void CTRL_INV_PRC_Initialize_filter(PResonatBlock *this)
{
	PResonatFilter_Initialize( &this->FILTER.har_1st );
	PResonatFilter_Initialize( &this->FILTER.har_5th );
	PResonatFilter_Initialize( &this->FILTER.har_7th );
	PResonatFilter_Initialize( &this->FILTER.har_11th );
}

float fThetaDiffSin = 0;
float fThetaDiffCos = 0;

void CTRL_INV_PRC_Run(void)
{
	volatile float a, b;
	volatile float id_ref, iq_ref;

	CTRL.INV.DROOP.v_ref_ramp = RAMP_Change(&CTRL.INV.DROOP.RAMP.v_ref , CTRL.INV.DROOP.v_ref_sec);

	if( INVERTER.uStatus == SYS_INV_RUN )
	{
		CTRL.INV.theta_ref = CONTHETA.fRadian + PI;// 150212
	}
	else
	{
		CTRL.INV.theta_ref += CTRL.INV.DROOP.w_ref_sec * CC_tsSample;
	}

	if (CTRL.INV.theta_ref > PI)
	{
		CTRL.INV.theta_ref += -TWO_PI;
	}
	else if (CTRL.INV.theta_ref < -PI)
	{
		CTRL.INV.theta_ref += TWO_PI;
	}

	CTRL.INV.sin_theta_ref = sin(CTRL.INV.theta_ref);
	CTRL.INV.cos_theta_ref = cos(CTRL.INV.theta_ref);

     /* +150204 June
      * *150212 (INV 전압 불평형 발생으로 인한)CTRL.INV.VI.enb 상황 외에서도 dc 성분을 빼주도록 수정.
	  */
	//- CTRL.INV.vi_dc = ((fabs(INVCTRL.Iqe)) * CTRL.INV.VI.r); //+150209 for TRACE
	 //160121 충전 시 전류 순환
	CTRL.INV.vi_dc = (-INVCTRL.Iqe * CTRL.INV.VI.r); //+150209 for TRACE
	CTRL.INV.vds_ref = (CTRL.INV.DROOP.v_ref_ramp - CTRL.INV.vi_dc ) * CTRL.INV.sin_theta_ref;
	CTRL.INV.vqs_ref = (CTRL.INV.DROOP.v_ref_ramp - CTRL.INV.vi_dc ) * CTRL.INV.cos_theta_ref;

	CTRL_INV_VI_Proceed();
	CTRL.INV.vds_ref_vi = CTRL.INV.vds_ref; //+ 150204 ( - CTRL.INV.VI.vds_compen; )
	CTRL.INV.vqs_ref_vi = CTRL.INV.vqs_ref; //- 150212 ( - CTRL.INV.VI.vqs_compen; )

	CTRL_INV_VUC_Proceed();
	CTRL.INV.vds_ref_vi_vuc = CTRL.INV.vds_ref_vi + CTRL.INV.VUC.vds_compen;
	CTRL.INV.vqs_ref_vi_vuc = CTRL.INV.vqs_ref_vi + CTRL.INV.VUC.vqs_compen;

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		CTRL_INV_PRC_InverterPResonat_GC(&CTRL.INV.PRC_Vds, CTRL.INV.vds_ref_vi_vuc - ACP.INV.vds);
		CTRL_INV_PRC_InverterPResonat_GC(&CTRL.INV.PRC_Vqs, CTRL.INV.vqs_ref_vi_vuc - ACP.INV.vqs);
	}
	else
	{
		if(PRM_PCS[SEAMLESS_PR_STOP].iValue == 1)
		{
			if(CTRL.INV.SEAMLESS.PwmOffCnt < 1 || CTRL.INV.SEAMLESS.PwmOffCnt == PARAM_VAL(SEAMLESS_PWM_OFF_CNT))
			{
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Vds, CTRL.INV.vds_ref_vi_vuc - ACP.INV.vds);
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Vqs, CTRL.INV.vqs_ref_vi_vuc - ACP.INV.vqs);
			}
		}
		else if(PRM_PCS[SEAMLESS_PR_STOP].iValue == 2)
		{
			if(CTRL.INV.SEAMLESS.PwmOffCnt < 1 || CTRL.INV.SEAMLESS.PwmOffCnt == PARAM_VAL(SEAMLESS_PWM_OFF_CNT))
			{
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Vds, CTRL.INV.vds_ref_vi_vuc - ACP.INV.vds);
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Vqs, CTRL.INV.vqs_ref_vi_vuc - ACP.INV.vqs);
			}
			else
			{
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Vds, 0);
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Vqs, 0);
			}
		}
		else
		{
			CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Vds, CTRL.INV.vds_ref_vi_vuc - ACP.INV.vds);
			CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Vqs, CTRL.INV.vqs_ref_vi_vuc - ACP.INV.vqs);
		}
	}

	CTRL.INV.ids_ref_lim = CTRL.INV.PRC_Vds.output;
	if (CTRL.INV.ids_ref_lim > CTRL.INV.PROTECTION.Idqs_MAX) CTRL.INV.ids_ref_lim = CTRL.INV.PROTECTION.Idqs_MAX;
	if (CTRL.INV.ids_ref_lim < CTRL.INV.PROTECTION.Idqs_MIN) CTRL.INV.ids_ref_lim = CTRL.INV.PROTECTION.Idqs_MIN;

	CTRL.INV.iqs_ref_lim = CTRL.INV.PRC_Vqs.output;
	if (CTRL.INV.iqs_ref_lim > CTRL.INV.PROTECTION.Idqs_MAX) CTRL.INV.iqs_ref_lim = CTRL.INV.PROTECTION.Idqs_MAX;
	if (CTRL.INV.iqs_ref_lim < CTRL.INV.PROTECTION.Idqs_MIN) CTRL.INV.iqs_ref_lim = CTRL.INV.PROTECTION.Idqs_MIN;

#if 1

	if ( INVERTER.uStatus == SYS_INV_AC_GENERATE || INVERTER.uStatus == SYS_INV_START_SYNC )
	{
		CTRL.INV.ids_ref = CTRL.INV.ids_ref_lim;
		CTRL.INV.iqs_ref = CTRL.INV.iqs_ref_lim;
		a = PARAM_VAL(ACTIVE_DAMPING) * ACP.INV.RATE.Z;
		CTRL.INV.va_cap_current_compen = a * ACP.CAP.ia;
		CTRL.INV.vb_cap_current_compen = a * ACP.CAP.ib;
		CTRL.INV.vc_cap_current_compen = a * ACP.CAP.ic;
	}
	else
	{
		if (CTRL.INV.ctrl_mode == PARAM_VSI_PR) // VSI_P+R
		{
			CTRL.INV.ids_ref = CTRL.INV.ids_ref_lim;
			CTRL.INV.iqs_ref = CTRL.INV.iqs_ref_lim;
			a = PARAM_VAL(ACTIVE_DAMPING) * ACP.INV.RATE.Z;
			CTRL.INV.va_cap_current_compen = a * ACP.CAP.ia;
			CTRL.INV.vb_cap_current_compen = a * ACP.CAP.ib;
			CTRL.INV.vc_cap_current_compen = a * ACP.CAP.ic;
		}
		else if(CTRL.INV.ctrl_mode == PARAM_CSI_PR)
		{
			id_ref = - INVCTRL.IqeRef;//PRM_PCS[BATT_FSM_STATE].iValue;
//			iq_ref = - (float)PARAM_VAL(CTRL_REACTIVE_POWER) * 0.01 * ACP.INV.RATE.Iph_pk;
			iq_ref = INVCTRL.IdeRef;
			CTRL.INV.ids_ref = -INVCTRL.SinTheta * id_ref - INVCTRL.CosTheta * iq_ref;
			CTRL.INV.iqs_ref = -INVCTRL.CosTheta * id_ref + INVCTRL.SinTheta * iq_ref;
			CTRL.INV.va_cap_current_compen = 0.;
			CTRL.INV.vb_cap_current_compen = 0.;
			CTRL.INV.vc_cap_current_compen = 0.;
		}
	}

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		CTRL_INV_PRC_InverterPResonat_GC(&CTRL.INV.PRC_Ids, CTRL.INV.ids_ref - ACP.INV.ids);
		CTRL_INV_PRC_InverterPResonat_GC(&CTRL.INV.PRC_Iqs, CTRL.INV.iqs_ref - ACP.INV.iqs);
	}
	else
	{
		if(PRM_PCS[SEAMLESS_PR_STOP].iValue == 1)
		{
			if(CTRL.INV.SEAMLESS.PwmOffCnt < 1 || CTRL.INV.SEAMLESS.PwmOffCnt == PARAM_VAL(SEAMLESS_PWM_OFF_CNT))
			{
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Ids, CTRL.INV.ids_ref - ACP.INV.ids);
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Iqs, CTRL.INV.iqs_ref - ACP.INV.iqs);
			}
		}
		else if(PRM_PCS[SEAMLESS_PR_STOP].iValue == 2)
		{
			if(CTRL.INV.SEAMLESS.PwmOffCnt < 1 || CTRL.INV.SEAMLESS.PwmOffCnt == PARAM_VAL(SEAMLESS_PWM_OFF_CNT))
			{
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Ids, CTRL.INV.ids_ref - ACP.INV.ids);
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Iqs, CTRL.INV.iqs_ref - ACP.INV.iqs);
			}
			else
			{
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Ids, 0);
				CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Iqs, 0);
			}
		}
		else
		{
			CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Ids, CTRL.INV.ids_ref - ACP.INV.ids);
			CTRL_INV_PRC_InverterPResonat_IS(&CTRL.INV.PRC_Iqs, CTRL.INV.iqs_ref - ACP.INV.iqs);
		}
	}

	if ( INVERTER.uStatus == SYS_INV_RUN )
	{
		if (PRM_PCS[CTRL_PR_V_COMPEN].iValue && CTRL.INV.ctrl_mode == PARAM_CSI_PR)
		{
			/* 141110 전압제어시에는 ref 를 사용하지 않도록 수정.
			 * P+R 전류 제어 모드일 경우 계통 임피던스 영향을 받지 않도록 하기 위함.
			 */
			CTRL.INV.vds_ref = - CTRL.INV.DROOP.v_ref_ramp * INVCTRL.SinTheta;
			CTRL.INV.vqs_ref = - CTRL.INV.DROOP.v_ref_ramp * INVCTRL.CosTheta;
			a = CTRL.INV.PRC_Ids.output + (float)PARAM_VAL(VDS_FF_GAIN) * CTRL.INV.vds_ref;
			b = CTRL.INV.PRC_Iqs.output + (float)PARAM_VAL(VQS_FF_GAIN) * CTRL.INV.vqs_ref;
		}
		else
		{
			a = CTRL.INV.PRC_Ids.output + (float)PARAM_VAL(VDS_FF_GAIN) * ACP.INV.vds;
			b = CTRL.INV.PRC_Iqs.output + (float)PARAM_VAL(VQS_FF_GAIN) * ACP.INV.vqs;
		}
	}
	else
	{
		a = CTRL.INV.PRC_Ids.output + (float)PARAM_VAL(VDS_FF_GAIN) * ACP.INV.vds;
		b = CTRL.INV.PRC_Iqs.output + (float)PARAM_VAL(VQS_FF_GAIN) * ACP.INV.vqs;
	}

	CTRL.INV.va_ref = a;
	CTRL.INV.vb_ref = -0.5 * ( a + SQRT3*b );
	CTRL.INV.vc_ref = -(CTRL.INV.va_ref + CTRL.INV.vb_ref);

	CTRL.INV.va_ref_ad = CTRL.INV.va_ref + CTRL.INV.va_cap_current_compen;
	CTRL.INV.vb_ref_ad = CTRL.INV.vb_ref + CTRL.INV.vb_cap_current_compen;
	CTRL.INV.vc_ref_ad = CTRL.INV.vc_ref + CTRL.INV.vc_cap_current_compen;

	INVCTRL.VaRef = CTRL.INV.va_ref_ad;
	INVCTRL.VbRef = CTRL.INV.vb_ref_ad;
	INVCTRL.VcRef = CTRL.INV.vc_ref_ad;
#endif
}

void PResonatFilter_Creation_IS( PResonatFilter *this, float SampleTime, float CutOffFreq, float HarmonicFreq )
{
	float be, r1, r2, r3;

	this->IS_COEFF.t_sample = SampleTime;
	this->IS_COEFF.f_cutoff = CutOffFreq;
	this->IS_COEFF.f_harmonic = HarmonicFreq;

	this->IS_COEFF.w_cutoff = 2 * PI * this->IS_COEFF.f_cutoff;
	this->IS_COEFF.w_harmonic = 2 * PI * this->IS_COEFF.f_harmonic;
	be = 2/this->IS_COEFF.t_sample;
	r1 = be*be + 2*this->IS_COEFF.w_cutoff*be + this->IS_COEFF.w_harmonic*this->IS_COEFF.w_harmonic;
	r2 = 2 * (this->IS_COEFF.w_harmonic*this->IS_COEFF.w_harmonic - be*be);
	r3 = be*be - 2*this->IS_COEFF.w_cutoff*be + this->IS_COEFF.w_harmonic*this->IS_COEFF.w_harmonic;

	this->IS_COEFF.b0 = be/r1;
	this->IS_COEFF.b2 = -be/r1;
	this->IS_COEFF.a1 = r2/r1;
	this->IS_COEFF.a2 = r3/r1;

	if(INVERTER.uStatus == SYS_INV_INIT	|| INVERTER.uStatus == SYS_INV_FAULT || INVERTER.uStatus == SYS_INV_STOP)
	{
		this->w0 = 0.;
		this->w1 = 0.;
		this->w2 = 0.;
		this->y = 0.; // Output
	}
}

void PResonatFilter_Creation_GC( PResonatFilter *this, float SampleTime, float CutOffFreq, float HarmonicFreq )
{
	float be, r1, r2, r3;

	this->GC_COEFF.t_sample = SampleTime;
	this->GC_COEFF.f_cutoff = CutOffFreq;
	this->GC_COEFF.f_harmonic = HarmonicFreq;

	this->GC_COEFF.w_cutoff = 2 * PI * this->GC_COEFF.f_cutoff;
	this->GC_COEFF.w_harmonic = 2 * PI * this->GC_COEFF.f_harmonic;
	be = 2/this->GC_COEFF.t_sample;
	r1 = be*be + 2*this->GC_COEFF.w_cutoff*be + this->GC_COEFF.w_harmonic*this->GC_COEFF.w_harmonic;
	r2 = 2 * (this->GC_COEFF.w_harmonic*this->GC_COEFF.w_harmonic - be*be);
	r3 = be*be - 2*this->GC_COEFF.w_cutoff*be + this->GC_COEFF.w_harmonic*this->GC_COEFF.w_harmonic;

	this->GC_COEFF.b0 = be/r1;
	this->GC_COEFF.b2 = -be/r1;
	this->GC_COEFF.a1 = r2/r1;
	this->GC_COEFF.a2 = r3/r1;

	if(INVERTER.uStatus == SYS_INV_INIT	|| INVERTER.uStatus == SYS_INV_FAULT || INVERTER.uStatus == SYS_INV_STOP)
	{
		this->w0 = 0.;
		this->w1 = 0.;
		this->w2 = 0.;
		this->y = 0.; // Output
	}
}

void PResonatFilter_Initialize( PResonatFilter *this )
{
	this->w0 = 0.;
	this->w1 = 0.;
	this->w2 = 0.;
	this->y = 0.; // Output
}
