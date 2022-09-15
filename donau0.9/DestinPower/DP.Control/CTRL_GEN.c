/*
 * CTRL_GEN.c
 *
 *  Created on: 2015. 8. 28.
 *      Author: destinPower
 *
 *      Bypass Mode 20 Hill test ��� ( GI��忡�� ���׷����� GC �Ѿ ��� )
 *      1. GI->GC �� �� diffThetaRadian�� ���� �����Ƿ�, accept_rad ���� 0.8�� �ø�.
 *      2. bess���� test mode �� 4�� ���� mc6 ���̱� �� �ܰ�� ���´�. HILL ���� MC�� �ٿ� �ָ� test mode�� 0���� �����Ͽ� ����ǵ���.
 *
 *
 */

#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "CTRL_GEN.h"
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
#include "CTRL_BYP_EVT_OPERATION.h"

GeneratorSyncCheckBlock GenBlock;

void gd(int status)
{
	if( status != GenBlock.debug )
	{
		GenBlock.debugPrev = GenBlock.debug;
	}
	GenBlock.debug = status;
}

// Call CC
void CTRL_GEN_Proceed(void)
{
	volatile float a;
	volatile iir1st *pIIR1;
	volatile PICon *pPICon;
	volatile PIArg *pPIArg;

	if ( !PARAM_VAL(BYP_MODE) == 20 )
	{
		GenBlock.bOn = FALSE;
		return;
	}

	GenBlock.bOn = TRUE;

	//////////////////////////////////////////////////////////////////////////

	//-151215 a = - ACP.PCC.vds * ACP.GEN.vqs + ACP.PCC.vqs * ACP.GEN.vds;
	a = - ACP.INV.vds_ll * ACP.GEN.vqs + ACP.INV.vqs_ll * ACP.GEN.vds;

	// ���� �ؾ� ��.
	//-if(INVERTER.uStatus == SYS_INV_ISLANDING && GenBlock.bGenNormalOk )
	//-	a = - ACP.INV.vds_ll * ACP.GEN.vqs + ACP.INV.vqs_ll * ACP.GEN.vds;

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2(&CTRL.GEN.SYNC.FILTER.iir_w, a);
	}
	else
	{
		IIR1_Filter2_IS(&CTRL.GEN.SYNC.FILTER.iir_w, a);
	}

	/*
	 * 151126 - BYP MODE 30
	 * SYNC_W_PI_MAX �� 1�� �ָ� Resync ��Ȳ���� �� �� ��ũ�� õõ�� �� �����ϳ�,
	 * �ʱ� �⵿ �� Droop�� ������ PCS UV �߻��ϴ� ��찡 �־ Resync��Ȳ������ 1�� ����.
	 */
	if( INVERTER.uStatus == SYS_INV_RE_SYNC && PRM_PCS[BYP_MODE].iValue == 20 )
	{
		CTRL.GEN.SYNC.PI_W.ARG.fMax = (2 * PI * PARAM_VAL(GRID_RATED_FREQ)) * PARAM_VAL(RESYNC_W_PI_MAX) * 0.01;
		CTRL.GEN.SYNC.PI_W.ARG.fMin = - CTRL.GEN.SYNC.PI_W.ARG.fMax;
	}
	else
	{
		CTRL.GEN.SYNC.PI_W.ARG.fMax = (2 * PI * PARAM_VAL(GRID_RATED_FREQ)) * PARAM_VAL(SYNC_W_PI_MAX) * 0.01;
		CTRL.GEN.SYNC.PI_W.ARG.fMin = - CTRL.GEN.SYNC.PI_W.ARG.fMax;
	}
	CTRL.GEN.SYNC.PI_W.ARG.fErr = CTRL.GEN.SYNC.FILTER.iir_w.fOut;
	PICON_PI_ONLY_RunM((&CTRL.GEN.SYNC.PI_W.CORE), (&CTRL.GEN.SYNC.PI_W.ARG));
	CTRL.GEN.SYNC.w = CTRL.GEN.SYNC.PI_W.CORE.fOut;

	//=============
	ACP.GEN.v_pk = sqrt(ACP.GEN.vds * ACP.GEN.vds + ACP.GEN.vqs * ACP.GEN.vqs);

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2(&CTRL.GEN.SYNC.FILTER.iir_v, ACP.GEN.v_pk);
	}
	else
	{
		IIR1_Filter2_IS(&CTRL.GEN.SYNC.FILTER.iir_v, ACP.GEN.v_pk);
	}

	/*
	 * 151106
	 * ��ȿ ������ ���ԵǾ� �ְ�, TR ���Ǵ����� ���� �ٸ� ��� SYNC���¿��� DROOP�� �������� ��ȿ���� ���ϰ� �� �� ����.
	 * ���� ��Ȳ ���� ���� �����ϱ�� ��.
	 */
	CTRL.GEN.SYNC.v = ACP.TR.RATE.RATIO * CTRL.GEN.SYNC.FILTER.iir_v.fOut;
	///////////////////////////////////////////////////////////////

	if( PRM_PCS[INV_TRANSFORMER].iValue == 0)
		GenBlock.diffThetaRadian = (CONTHETA.fRadian) - GEN_THETA.fRadian;
	else
		GenBlock.diffThetaRadian = (CONTHETA.fRadian + PI_6 ) - GEN_THETA.fRadian;  //- PI_6; Transformer

	if ( GenBlock.diffThetaRadian > PI ) GenBlock.diffThetaRadian += -TWO_PI;
	else if ( GenBlock.diffThetaRadian < -PI ) GenBlock.diffThetaRadian += TWO_PI;

	//abs
	if( GenBlock.SET_loadPowerMeterP < 0 )
		GenBlock.SET_loadPowerMeterP *= -1;
	if( GenBlock.SET_loadPowerMeterQ < 0 )
		GenBlock.SET_loadPowerMeterQ *= -1;

	/* Holding Power ��� - PMS�� ���� SET_loadPowerMeterP�� ���������� �����Ѵ�. (Load-PV_PCS) */
	GenBlock.holdingPowerP = (GenBlock.SET_loadPowerMeterP - GenBlock.SET_generatorMinimumPowerP);/* Holding ������ ���� Power Controller P Ref */
	GenBlock.holdingPowerQ = (GenBlock.SET_loadPowerMeterQ - GenBlock.SET_generatorMinimumPowerQ);/* Holding ������ ���� Power Controller Q Ref */

	if( GenBlock.powerDecreaseState == GPC_HOLDING )
	{
		GenBlock.generatorExpectPowerP = GenBlock.SET_generatorMinimumPowerP;
		GenBlock.generatorExpectPowerQ = GenBlock.SET_generatorMinimumPowerQ;
	}
	else if( GenBlock.powerDecreaseState == GPC_DECREASE )
	{
		// 0�� �����ϵ���.
		if( GenBlock.pcsDecreasePowerP <= 0 )
		{
			// ���� ��Ȳ( ratio = minus )
			if( GenBlock.decreaseRatio > 0 )
				GenBlock.decreaseRatio *= -1;
		}
		else
		{
			// ���� ��Ȳ( ratio = plus )
			if( GenBlock.decreaseRatio < 0 )
				GenBlock.decreaseRatio *= -1;
		}

		/* Decrease �Ŀ� ��� */
		GenBlock.generatorExpectPowerP += (GenBlock.generatorExpectPowerP *  GenBlock.decreaseRatio);
		GenBlock.generatorExpectPowerQ += (GenBlock.generatorExpectPowerQ *  GenBlock.decreaseRatio);

		GenBlock.pcsDecreasePowerP = (GenBlock.SET_loadPowerMeterP - (GenBlock.generatorExpectPowerP));/* Decrease ������ ���� Power Controller P Ref */
		GenBlock.pcsDecreasePowerQ = (GenBlock.SET_loadPowerMeterQ - (GenBlock.generatorExpectPowerQ));/* Decrease ������ ���� Power Controller Q Ref */
	}
	else
	{

	}

#if 0
	if ( CTRL.INV.operation_mode == PARAM_OPERATION_MODE_AUTO_IS
			&& INVERTER.uStatus == SYS_INV_RUN
			&& CTRL.INV.SEAMLESS.pcc_blackout_enb )
	{
		if( GPIO_GetStaticSwitchOn() )
			GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
	}
#endif
}

GEN_POWER_CTRL_STATE CTRL_GEN_PowerDecrease()
{
	switch( GenBlock.powerDecreaseState )
	{
	case GPC_NONE:
		// PWM OFF , POWER = 0
		// MOVE TO DECREASE FINISH
//		PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0;
//		PARAM_RAW_VAL(BATT_REMOTE_PCC_Q_REF) = 0;
		GenBlock.timer_hold_p_sec = 0;
		GenBlock.timer_decrease_p_sec = 0;
		break;
	case GPC_HOLDING:
		if( GenBlock.timer_hold_p_sec > GenBlock.SET_powerHoldTime )
			GenBlock.powerDecreaseState = GPC_DECREASE;

		GenBlock.timer_decrease_p_sec = 0;

		PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = GenBlock.holdingPowerP * 10; /* Variant 0.1 */
		PARAM_RAW_VAL(BATT_REMOTE_PCC_Q_REF) = GenBlock.holdingPowerQ * 10;

		break;
	case GPC_DECREASE:

#if 0
		if( GenBlock.timer_decrease_p_sec > GenBlock.SET_powerDecreaseTime )
			GenBlock.powerDecreaseState = GPC_NONE;
#endif
		GenBlock.timer_hold_p_sec = 0;

		// Inv P ref�� 5kW���� ������ ����.
		//-if ( (GenBlock.pcsDecreasePowerP) < 5 )
		if ( fabs(GenBlock.pcsDecreasePowerP) < 5 )
		{
			GenBlock.pcsDecreasePowerP = 0; // 0->1 (160106:1kW-���� ����)
		}

		//-if( (GenBlock.decreasePowerQ) < 5 )
		if( fabs(GenBlock.pcsDecreasePowerQ) <= 5 )
		{
			GenBlock.pcsDecreasePowerQ = 0; // 0->1 (160106:1kW-���� ����)
		}

		if( GenBlock.pcsDecreasePowerP > PARAM_VAL(INV_CAPACITY) )
			GenBlock.pcsDecreasePowerP = PARAM_VAL(INV_CAPACITY);
		if( GenBlock.pcsDecreasePowerQ > PARAM_VAL(INV_CAPACITY) )
			GenBlock.pcsDecreasePowerQ = PARAM_VAL(INV_CAPACITY);


		PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = GenBlock.pcsDecreasePowerP * 10; /* Variant 0.1 */
		PARAM_RAW_VAL(BATT_REMOTE_PCC_Q_REF) = GenBlock.pcsDecreasePowerQ * 10;

		if( GenBlock.pcsDecreasePowerP == 0 && GenBlock.pcsDecreasePowerQ == 0 )
		{
			PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0;
			PARAM_RAW_VAL(BATT_REMOTE_PCC_Q_REF) = 0;
			GenBlock.powerDecreaseState = GPC_NONE;
			GenBlock.bPowerDecreaseEnable = FALSE;
		}

		break;
	}

	return GenBlock.powerDecreaseState;
}

void CTRL_GEN_RampInit()
{
	RAMP_Initialize(GenBlock.pRAMPOutput);
}

void CTRL_GEN_SetCharge()
{
	/*
	 * DC �� ������ Ȯ���Ͽ� OperV Max ���� ������ ������ �����Ѵ�.
	 * ���� ���۷����� {AUTO_CHARGE_P_REF}
	 *
	 * 151127 �ڵ� ���� �� �Ŀ����۷��� �Է¿� Ramp �߰�
	 */
#if 1 //-180201
#if 0 // HILL TESTED
	if (BATCTRL.fltDCBattV_2nd.fOut <= PARAM_VAL(BATT_V_RANGE_MAX) - 20.)
	{
		//- PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = (fabs(PRM_PCS[AUTO_CHARGE_P_REF].iValue)); /* Variant 0.1 */
		PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, -(fabs(PRM_PCS[AUTO_CHARGE_P_REF].iValue)));
	}
	else if( BATCTRL.fltDCBattV_2nd.fOut >= PARAM_VAL(BATT_V_RANGE_MAX) - 6. )
	{
		PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, 0);
	}
#else
	// MCU TESTED
	switch( GenBlock.autoChargingState )
	{
	case GAC_STANDBY:
		gd(71);

#ifdef BYP_EVT_OPERATION
		if(bByp_Evt_Operating == TRUE)
		{
			GenBlock.autoChargingState = GAC_STANDBY;
			// -PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, 0);
			PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0;
			break;
		}
#endif

		if (BATCTRL.fltDCBattV_2nd.fOut <= PARAM_VAL(BATT_V_RANGE_MAX) - 20.)
		{
			RAMP_SetInitOut(GenBlock.pRAMPOutput, 0); //Ramp �ʱ�������� ���� 0->-1 (160106:1kW-���� ����)

			//if(	GenBlock.SET_loadPowerMeterS + fabs(PRM_PCS[AUTO_CHARGE_P_REF].iValue) * PRM_PCS[AUTO_CHARGE_P_REF].fIncDec * 1000
			//	< PARAM_VAL(INV_CAPACITY) * 1000 )
			if(	fabs(PRM_PCS[AUTO_CHARGE_P_REF].iValue) * PRM_PCS[AUTO_CHARGE_P_REF].fIncDec * 1000
				< PARAM_VAL(INV_CAPACITY) * 1000 )
			{
				GenBlock.autoChargingState = GAC_CHARGING;
			}
		}
		else
		{
			//-PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0;
		}
		break;
	case GAC_CHARGING:

#ifdef BYP_EVT_OPERATION
		if(bByp_Evt_Operating == TRUE)
		{
			GenBlock.autoChargingState = GAC_STANDBY;
			RAMP_Change( GenBlock.pRAMPOutput, 0);
			break;
		}
#endif

		//if( (BATCTRL.fltDCBattV_2nd.fOut >= PARAM_VAL(BATT_V_RANGE_MAX) - 6.) ||
		//		GenBlock.SET_loadPowerMeterS + fabs(PRM_PCS[AUTO_CHARGE_P_REF].iValue) * PRM_PCS[AUTO_CHARGE_P_REF].fIncDec * 1000
		//		> PARAM_VAL(INV_CAPACITY) * 1000 )
		if( (BATCTRL.fltDCBattV_2nd.fOut >= PARAM_VAL(BATT_V_RANGE_MAX) - 6.) ||
				fabs(PRM_PCS[AUTO_CHARGE_P_REF].iValue) * PRM_PCS[AUTO_CHARGE_P_REF].fIncDec * 1000
				> PARAM_VAL(INV_CAPACITY) * 1000 )
		{
			gd(72);
			//PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, 0);
			PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0;
			if( fabs(EXCTRL.fltAcPowerP.fOut) < 5000 )
			{
				//PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0; //+ 160412
				GenBlock.autoChargingState = GAC_STANDBY;
			}
		}
		else
		{
			//-PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, -(fabs(PRM_PCS[AUTO_CHARGE_P_REF].iValue)));
			// 160129 �ڵ� ������ ������ ����ǵ��� ����
			//PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, (PRM_PCS[AUTO_CHARGE_P_REF].iValue));
			PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = PRM_PCS[AUTO_CHARGE_P_REF].iValue;

			gd(73);
		}
		break;
	}
#endif
#else//-180201
#if 0 // HILL TESTED
	if (BATCTRL.fltDCBattV_2nd.fOut <= PARAM_VAL(BATT_V_RANGE_MAX) - 20.)
	{
		//- PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = (fabs(PRM_PCS[AUTO_CHARGE_P_REF].iValue)); /* Variant 0.1 */
		PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, -(fabs(PRM_PCS[AUTO_CHARGE_P_REF].iValue)));
	}
	else if( BATCTRL.fltDCBattV_2nd.fOut >= PARAM_VAL(BATT_V_RANGE_MAX) - 6. )
	{
		PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, 0);
	}
#else
	// MCU TESTED
	switch( GenBlock.autoChargingState )
	{
	case GAC_STANDBY:
		gd(71);

#ifdef BYP_EVT_OPERATION
		if(bByp_Evt_Operating == TRUE)
		{
			GenBlock.autoChargingState = GAC_STANDBY;
			// -PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, 0);
			PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0;
			break;
		}
#endif

		if( ODT_Update(INVERTER.odtHoldLoadP, TRUE) == ODT_FINISH )
		{
			GenBlock.autoChargingState = GAC_CHARGING;
		}
#if 0
		else if (BATCTRL.fltDCBattV_2nd.fOut <= PARAM_VAL(BATT_V_RANGE_MAX) - 20.)
		{
			RAMP_SetInitOut(GenBlock.pRAMPOutput, 0); //Ramp �ʱ�������� ���� 0->-1 (160106:1kW-���� ����)
			GenBlock.autoChargingState = GAC_CHARGING;
		}
#endif
		else
		{
			PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = INV_SQRT2 * GenBlock.holdingLoadP * 0.01;
			PCC.fRefP = INV_SQRT2 * GenBlock.holdingLoadP;
			if( PRM_PCS[INV_TRANSFORMER].iValue == 1 )
			{
				EXCTRL.fltPccEqeToCurrentReference = -(PCC.fRefP) * ( 2 / ( EXCTRL.fltEqe.fOut * TR_RATIO /* PCC ���� */ * 3 ) );
				EXCTRL.fltPccEdeToCurrentReference = -(PCC.fRefQ) * ( 2 / ( EXCTRL.fltEqe.fOut * TR_RATIO /* PCC ���� */ * 3 ) );
			}
			else
			{
				EXCTRL.fltPccEqeToCurrentReference = -(PCC.fRefP) * ( 2 / ( EXCTRL.fltEqe.fOut /*  PCC ���� */ * 3 ) );
				EXCTRL.fltPccEdeToCurrentReference = -(PCC.fRefQ) * ( 2 / ( EXCTRL.fltEqe.fOut /* PCC ���� */ * 3 ) );
			}

			INVCTRL.IqeRef = EXCTRL.fltPccEqeToCurrentReference; //-160420 RAMP_Change(PCC.pRAMPOutput, EXCTRL.fltPccEqeToCurrentReference);
			INVCTRL.IdeRef = EXCTRL.fltPccEdeToCurrentReference; //-160420 RAMP_Change(PCC.pRAMPOutput_Q, EXCTRL.fltPccEdeToCurrentReference);
		}

		GenBlock.pRAMPOutput->fOut = PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF);
		GenBlock.pRAMPOutputQ->fOut = PARAM_RAW_VAL(BATT_REMOTE_PCC_Q_REF);

		break;
	case GAC_CHARGING:

#ifdef BYP_EVT_OPERATION
		if(bByp_Evt_Operating == TRUE)
		{
			GenBlock.autoChargingState = GAC_STANDBY;
			RAMP_Change( GenBlock.pRAMPOutput, 0);
			break;
		}
#endif

		if( (BATCTRL.fltDCBattV_2nd.fOut >= PARAM_VAL(BATT_V_RANGE_MAX) - 6.) ||
				GenBlock.SET_loadPowerMeterS + fabs(PRM_PCS[AUTO_CHARGE_P_REF].iValue) * PRM_PCS[AUTO_CHARGE_P_REF].fIncDec * 1000
				> PARAM_VAL(INV_CAPACITY) * 1000 )
		{
			gd(72);
			PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, 0);
			PARAM_RAW_VAL(BATT_REMOTE_PCC_Q_REF) = RAMP_Change( GenBlock.pRAMPOutputQ, 0);
			if( fabs(EXCTRL.fltAcPowerP.fOut) < 5000 )
			{
				PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0; //+ 160412
				PARAM_RAW_VAL(BATT_REMOTE_PCC_Q_REF) = 0;
				GenBlock.autoChargingState = GAC_STANDBY;
			}
		}
		else
		{
			//-PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, -(fabs(PRM_PCS[AUTO_CHARGE_P_REF].iValue)));
			// 160129 �ڵ� ������ ������ ����ǵ��� ����
			PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = RAMP_Change( GenBlock.pRAMPOutput, (PRM_PCS[AUTO_CHARGE_P_REF].iValue));

			gd(73);
		}
		break;
	}
#endif
#endif//-180201
}

void CTRL_GEN_UpdateParameter()
{
	float inv_freq;

	GenBlock.accept_rad = (float)PARAM_VAL(CTRL_IINV_SYNC_TOLERANCE_THETA) * (PI / 180.);
	GenBlock.diffThetaRadian = 0xFFFF;
	GenBlock.SET_powerHoldTime = PARAM_VAL(BATT_HOLD_SEC);
	GenBlock.SET_powerDecreaseTime = PARAM_VAL(BATT_P_DECREASE_SEC);
	GenBlock.SET_generatorMinimumPowerP = PARAM_VAL(BATT_GENERATOR_POWER_P);
	GenBlock.SET_generatorMinimumPowerQ = PARAM_VAL(BATT_GENERATOR_POWER_Q);
	GenBlock.SET_loadPowerMeterP = PARAM_VAL(BATT_POWER_METER_P);
	GenBlock.SET_loadPowerMeterQ = PARAM_VAL(BATT_POWER_METER_Q);
#if DOUBLE_CONTROL == 1
	GenBlock.decreaseRatio = ((1. / GenBlock.SET_powerDecreaseTime) * CC_tsSample) * 2; // 151021 /2
#else
	GenBlock.decreaseRatio = ((1. / GenBlock.SET_powerDecreaseTime) * CC_tsSample);
#endif

	if( PRM_PCS[SYNC_W_PI_KP_DIVISION].iValue < 1 )
	{
		PRM_PCS[SYNC_W_PI_KP_DIVISION].iValue = 1;
	}

	CTRL.GEN.SYNC.PI_W.K.fP = (float)PARAM_VAL(SYNC_W_PI_KP) / (float)PRM_PCS[SYNC_W_PI_KP_DIVISION].iValue;
	CTRL.GEN.SYNC.PI_W.K.fIT = (float)PARAM_VAL(SYNC_W_PI_KI);
	CTRL.GEN.SYNC.PI_W.K.fA = 1. / CTRL.GEN.SYNC.PI_W.K.fP;

	inv_freq = 	PARAM_VAL(GRID_RATED_FREQ);
	CTRL.GEN.SYNC.PI_W.ARG.fMax = (2 * PI * inv_freq) * PARAM_VAL(SYNC_W_PI_MAX) * 0.01;
	CTRL.GEN.SYNC.PI_W.ARG.fMin = - CTRL.GEN.SYNC.PI_W.ARG.fMax;

	IIR1_UpdateCoeff(&CTRL.GEN.SYNC.FILTER.iir_w, CC_tsSample, PARAM_VAL(SYNC_W_IIR_CUTOFF));
	IIR1_UpdateCoeff(&CTRL.GEN.SYNC.FILTER.iir_v, CC_tsSample, PARAM_VAL(SYNC_V_IIR_CUTOFF));
}

void CTRL_GEN_Create(void)
{
	memset(&GenBlock, 0, sizeof(GenBlock));

	float t_sample_main, t_sample_is, delta;

	t_sample_main = CTRL_FILTER_GetSampleTime(CTRL_CC_PERIOD);
	t_sample_is = CTRL_FILTER_GetSampleTime(IS_CCP);

	IIR1_Creation_IS(&CTRL.GEN.SYNC.FILTER.iir_w, t_sample_main, 10. , t_sample_is);
	IIR1_Creation_IS(&CTRL.GEN.SYNC.FILTER.iir_v, t_sample_main, 5. , t_sample_is);

	PICon_(&CTRL.GEN.SYNC.PI_W.CORE);
	PIArg_(&CTRL.GEN.SYNC.PI_W.ARG, &CTRL.GEN.SYNC.PI_W.K);
	CTRL.GEN.SYNC.PI_W.ARG.fAlpha = 1; // 1 ==> PI ����, 0 ==> IP ����

	CTRL_GEN_UpdateParameter();

#if DBUG_MODE == 3
	GenBlock.odtGenNormal = Odt_(&GenBlock.ODT_GEN_NORMAL, 5000, 5 /*ms SystemState*/);
#else
	GenBlock.odtGenNormal = Odt_(&GenBlock.ODT_GEN_NORMAL, 20000, 5 /*ms SystemState*/);
#endif
	GenBlock.odtGenSyncOk = Odt_(&GenBlock.ODT_GEN_SYNC_OK, 5000, 5 /*ms SystemState*/);
	GenBlock.odtGridConnectDelay = Odt_(&GenBlock.ODT_GridConnectDelay, 1000, 5 /*ms SystemState*/);
	GenBlock.odtMC6OpenCheck = Odt_(&GenBlock.ODT_MC6_OPEN_CHECK, 1000, 5 /*ms SystemState*/);
#ifndef DBUG_MODE
#ifndef STABLEEN
	delta = 0.05; // period 5ms, 160129 0.1 -> 0.05	// 0.05 -> 16.07.28 STABLEEN 0.15 10sec
#else
	//delta = 0.15; // period 5ms, 160129 0.1 -> 0.05	// 0.05 -> 16.07.28 STABLEEN 0.15 10sec // 3kW/sec
	delta = 1.5; // 30kW/sec
#endif
#else
	delta = 0.5; // period 5ms, 160129 0.1 -> 0.05	// 0.05 -> 16.07.28 STABLEEN 0.15 10sec
#endif
	GenBlock.pRAMPOutput = Ramp_(&GenBlock.POWER_RAMP_OUTPUT, delta);
	RAMP_SetDelta(GenBlock.pRAMPOutput, delta);
	RAMP_Initialize(GenBlock.pRAMPOutput);

	GenBlock.bRunWithoutPMS = FALSE;

}

Bool CTRL_GEN_NormalOK(void)
{
	if (ACP.GEN.v_pk > 279. ) // 380�� 90%. �ܺ� TR�� �Ϻ� �𸣹Ƿ� �ϴ� ���Ѹ� �ֱ�� ��. 141112
		return TRUE;
	else
		return FALSE;
}

Bool CTRL_GEN_PCC_SyncOK( void )
{
//#if DBUG_MODE == 2
//	if( GenBlock.bGeneratorSyncOk )
//		return 1;
//	else
//		return 0;
//#endif
	//  w = theta omega. radian * 60 Hz
	if ( (fabs(GenBlock.diffThetaRadian) < (GenBlock.accept_rad)) )
		return TRUE;
	else
		return FALSE;
}

void CTRL_GEN_ClearTimer()
{
	GenBlock.timer_decrease_p_sec = 0;
	GenBlock.timer_hold_p_sec = 0;
}
