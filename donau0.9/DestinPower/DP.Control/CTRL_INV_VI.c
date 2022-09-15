/*
 * CTRL_INV_VI.c
 *
 *  Created on: 2014. 5. 16.
 *      Author: Seth
 *      Virtual Impedance
 */

#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "CTRL_INV_VI.h"
#include "CC.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "SYS_ESLV.h"


void CTRL_INV_VI_Proceed(void)
{
	volatile float a;

	if (!CTRL.INV.VI.g_enb && !eslvCtrl.g_bEnb)	return;

	if (
			//-(CTRL.INV.VI.g_enb == ON) && CTRL.INV.grid_connected_enb == OFF &&
			(CTRL.INV.VI.g_enb == ON) && /* +150204 June */
			CTRL.INV.VI.one_shot_enb == ON &&
			 ( (INVERTER.uStatus == SYS_INV_RUN || INVERTER.uStatus == SYS_INV_ISLANDING) && ( CTRL.INV.SEAMLESS.pcc_blackout_enb)) /* +150204 June for test */)
	{
		CTRL.INV.VI.enb = ON;
	}
	else
	{
		CTRL.INV.VI.enb = OFF;
	}

	if ( CTRL.INV.VI.enb )
	{
		a =  exp(-CTRL.INV.VI.t_sec);
		CTRL.INV.VI.r = CTRL.INV.VI.R_a * a + CTRL.INV.VI.R_b;
		CTRL.INV.VI.l = CTRL.INV.VI.L_a * a + CTRL.INV.VI.L_b;

		// Linear Graph
#if 0
		CTRL.INV.VI.vds_compen = CTRL.INV.VI.iqs_delay90 * CTRL.INV.VI.r - ACP.INV.RATE.Omega * CTRL.INV.VI.l * ACP.INV.iqs;
		CTRL.INV.VI.vqs_compen = ACP.INV.iqs * CTRL.INV.VI.r + ACP.INV.RATE.Omega * CTRL.INV.VI.l * CTRL.INV.VI.iqs_delay90;
#else   // 150204 June
		// 순간 정전 시에 전류 과도에 영향 받지 않도록 0 으로 set.
		CTRL.INV.VI.vds_compen = 0;
		CTRL.INV.VI.vqs_compen = 0;
#endif

		CTRL.INV.VI.t_sec += CC_tsSample;
		if (CTRL.INV.VI.t_sec > CTRL.INV.VI.T_sec)
			CTRL.INV.VI.one_shot_enb = OFF;
	}
	else
	{
		CTRL.INV.VI.vds_compen = CTRL.INV.VI.iqs_delay90 * CTRL.INV.VI.r - ACP.INV.RATE.Omega * CTRL.INV.VI.l * ACP.INV.iqs;
		CTRL.INV.VI.vqs_compen = ACP.INV.iqs * CTRL.INV.VI.r + ACP.INV.RATE.Omega * CTRL.INV.VI.l * CTRL.INV.VI.iqs_delay90;
		CTRL.INV.VI.t_sec = 0.;
#if 1
		CTRL.INV.VI.r = 0.;
		CTRL.INV.VI.l = 0.;
#endif
	}
}

void CTRL_INV_VI_OneShotEnable(void)
{
	CTRL.INV.VI.one_shot_enb = ON;
}


void CTRL_INV_VI_UpdateParameter(void)
{
	CTRL.INV.VI.g_enb = PARAM_VAL(VI_ENB);
	CTRL.INV.VI.R_p = PARAM_VAL(VI_R_P) * 0.01 * ACP.INV.RATE.Z; // 전환 및 과도시 적용 한다.
	CTRL.INV.VI.R_pi = PARAM_VAL(VI_R_PI) * 0.01 * ACP.INV.RATE.Z; // 모든 상황에 적용되어야 한다.
	CTRL.INV.VI.L_p = PARAM_VAL(VI_L_P) * 0.0001 * ACP.INV.RATE.Z;
	CTRL.INV.VI.L_pi = PARAM_VAL(VI_L_PI) * 0.0001 * ACP.INV.RATE.Z;
	CTRL.INV.VI.T_sec = PARAM_VAL(VI_T_MSEC) * 0.001;
	CTRL.INV.VI.Inverse_T_sec = 1/CTRL.INV.VI.T_sec;
	CTRL.INV.VI.R_a = ( CTRL.INV.VI.R_pi - CTRL.INV.VI.R_p ) / ( exp(-CTRL.INV.VI.T_sec) - 1.0 );
	CTRL.INV.VI.R_b = CTRL.INV.VI.R_p - CTRL.INV.VI.R_a;
	CTRL.INV.VI.L_a = ( CTRL.INV.VI.L_pi - CTRL.INV.VI.L_p ) / ( exp(-CTRL.INV.VI.T_sec) - 1.0 );
	CTRL.INV.VI.L_b = CTRL.INV.VI.L_p - CTRL.INV.VI.L_a;
}

void CTRL_INV_VI_Create(void)
{
	CTRL.INV.VI.r = 0.;
	CTRL.INV.VI.l = 0.;
	CTRL.INV.VI.vds_compen = 0.;
	CTRL.INV.VI.vqs_compen = 0.;
	CTRL.INV.VI.one_shot_enb = OFF;
	CTRL.INV.VI.enb = OFF;
	CTRL.INV.VI.t_sec = 0.;

	CTRL_INV_VI_UpdateParameter();
}

void CTRL_INV_VI_Initialize(void)
{
	CTRL.INV.VI.vds_compen = 0.;
	CTRL.INV.VI.vqs_compen = 0.;
	CTRL.INV.VI.r = 0.;
	CTRL.INV.VI.l = 0.;
	CTRL.INV.VI.t_sec = 0.;
}
