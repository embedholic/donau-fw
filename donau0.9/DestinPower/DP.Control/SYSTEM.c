/*
 * SYSTEM.Logos.c
 *
 *  Created on: 2013. 8. 5.
 *      Author: destinPower
 */

/*
 *  TEST MODE 2(계통 연계 전압 제어)를 사용하지 않을 경우, 0로 놓아 코드 가독성을 높임.
 *  TEST MODE 2는 전압제어로 파라미터 VDC DISCHARGE REF(SIMULATOR DC REF보다 작게 설정) 와
 *  CURRENT LIMIT(5%부터 서서히 올려서 확인)을 이용하여 장비를 제어함.
 *
 *  SWI에서 NvSRAM 접근 하지 말 것.
 */
#include <math.h>
#include <string.h>

#include "SYSTEM.h"
#include "ODT.h"
#include "FAULT.h"
#include "Event.h"
#include "MVP.h"
#include "CC.h"
#include "EADC.h"
#include "trace.h"
#include "PWM.h"
#include "MCCB.h"
#include "MathConst.h"
#include "RMS.h"
#include "LEVELCHECK.h"
#include "PHASECHECK.h"
#include "MCB.h"
#include "parameter.h"
#include "Prm_pcs.h"
//-#include "COSPHIcontrol.h"
#include "NvSRAM.h"
#include "CAN.h"
#include "CAN_GBI.h"
#include "MO.h"
#include "Task.h"
#include "GPIO.h"
#include "CTRL_INV_VI.h"
#include "CTRL_MODE.h"
#include "SYS_ESLV.h"
//#include "CTRL_GEN.h"
#include "CTRL_BYP_EVT_OPERATION.h"
#include "SagEventHistory.h"

#define SYS_VOLT_HYSTERESIS 2.0  /* 2.0V */
#define SYS_FREQ_HYSTERESIS 0.05  /* 0.05Hz */
#define SYS_PHASE_ERR_LEVEL 100

#define SYS_20MSEC_PERIOD 4 /* 5msec * 4 = 20msec */
#define SYS_10MSEC_PERIOD 2 /* 5msec * 2 = 10msec */
#define SYS_UF_OF_CHECK_NUM	100 /* 주파수가 이 횟수 만큼 동안 연속해서 문제가 있다면 고장 발생 */

#define SYS_OC_LEVEL 1.8
#define SYS_INPUT_I_LIMIT_LEVEL 1.25
#define SYS_OV_LEVEL 1.1
#define SYS_UV_LEVEL 0.9
#define SYS_BYP_INV_EQUAL_LEVEL 2 /* V */
#define SYS_CLEAR_CNT 50
#define MPP_VDC_REF_STEP	5.0



extern	AC_Panel	ACP;
extern	Controller	CTRL;

enum
{
	SYS_INV_STATE_TRAN_FAULT,
	SYS_INV_STATE_TRAN_FAULT_RESET,
	SYS_INV_STATE_TRAN_STOP,
#ifdef Fixed_State_Machine
	SYS_INV_STATE_TRAN_SCR_ON,
#endif
	SYS_INV_STATE_TRAN_DC_CHARGE,
#ifdef Fixed_State_Machine
	SYS_INV_STATE_TRAN_EVE_DC_CHARGE,
#endif
	SYS_INV_STATE_TRAN_AC_GENERATE,
	SYS_INV_STATE_TRAN_START_SYNC,
	SYS_INV_STATE_TRAN_RUN,
	SYS_INV_STATE_TRAN_ISLANDING,
	SYS_INV_STATE_TRAN_RESYNC,
	SYS_INV_STATE_TRAN_BYP_EVT_OPERATION,
	SYS_INV_STATE_TRAN_TEST_ON,
	SYS_INV_STATE_TRAN_TEST_OFF
};

Button BUTTON;
Inverter INVERTER;
GridArg GRID_ARG;
GeneratorConnectPredict GenConn;
Bool bDcChargerOnOff = FALSE;
int 	InverterStateShadow;

#if 0 //by JCNET
static Button *pButton;
#else
Button *pButton;
#endif
static Uns Cnt20ms = 0;
static Uns u20msCnt = 0;
static Uns u20msUnitForEvent = 0;
static UInt16 uiSysCallCount = 0;

static Bool bResync_OFUF_Disable = FALSE;
UInt16 RecloserSwitchCmd = 0;
#define RECLOSER_ON_CMD()  (RecloserSwitchCmd = 1)
#define RECLOSER_OFF_CMD()  (RecloserSwitchCmd = 2)
#define RECLOSER_CLR_CMD()  (RecloserSwitchCmd = 0)

// STAUS_MC6
#define RECLOSER_STATUS() !1

Bool GetDcChargerOnOff(void);
void SYS_CheckBatteryCurrent();
void SYS_SetGeneratorConnectPredict()
{
	GenConn.bPredictGenConnect = 0;
	GenConn.prevInverterIqe = 0;

	if( PRM_PCS[GEN_CONNECT_CHK_STX_POWER].iValue < 0 )
		PRM_PCS[GEN_CONNECT_CHK_STX_POWER].iValue = 0;

	if( PRM_PCS[GEN_CONNECT_I_DIDT].iValue > 100 )
		PRM_PCS[GEN_CONNECT_I_DIDT].iValue = 100;
	if( PRM_PCS[GEN_CONNECT_I_DIDT].iValue < 0 )
		PRM_PCS[GEN_CONNECT_I_DIDT].iValue = 0;

	GenConn.genConnectCheckStartPower = PRM_PCS[GEN_CONNECT_CHK_STX_POWER].iValue * 1000.;
	GenConn.genConnectCurrentDidtLevel = PRM_PCS[GEN_CONNECT_I_DIDT].iValue;
}

#if 0 //by JCNET
#pragma CODE_SECTION (SYS_UpdateInverterStatus, "dp_ctrl")
static void SYS_UpdateInverterStatus(void);
#else
#pragma CODE_SECTION (SYS_UpdateStateMachine, "dp_ctrl")
extern void SYS_UpdateStateMachine(void);
#endif

#if SYSTEM_EXECUTION_PERIOD
Uns SYS_startTime;
Uns SYS_endTime;
Uns SYS_uExecutionPeriod;
#endif

#pragma CODE_SECTION (SYS_UpdateParameter, "dp_ctrl")
void SYS_UpdateParameter(void)
{
#if DBUG_MODE == 0
	int GridFreq;
#else
	float GridFreq = 60;
#endif

	GridFreq = PRM_PCS[GRID_RATED_FREQ].iValue;

#if DBUG_MODE == 0
	if (GridFreq < 50 || GridFreq > 60)
		GridFreq = 60;
#else
#endif

	ACP.INV.RATE.P3 = (float) PRM_PCS[INV_CAPACITY].iValue * 1000.;
	ACP.INV.RATE.Q3 = (float) PRM_PCS[INV_CAPACITY].iValue * 1000.;

//	GRID_ARG.RATE.fLineVoltage = PRM_PCS[GRID_RATED_VOLT].iValue;
	ACP.PCC.RATE.Vll = (float) PRM_PCS[GRID_RATED_VOLT].iValue;
	ACP.PCC.RATE.Vph = ACP.PCC.RATE.Vll * INV_SQRT3;
	ACP.PCC.RATE.Vph_pk = ACP.PCC.RATE.Vph * SQRT2;
//	GRID_ARG.RATE.fFreq = GridFreq;

#if DBUG_MODE == 0
	ACP.PCC.RATE.Freq = GridFreq;
#else
	GridFreq = GridFreq + (PRM_PCS[DGT_DO1].iValue * 0.1 );
	ACP.PCC.RATE.Freq = GridFreq;
#endif

	ACP.PCC.RATE.Omega = 2 * PI * ACP.PCC.RATE.Freq;
//	GRID_ARG.RATE.fCurrent = ((float) PRM_PCS[INV_CAPACITY].iValue * 1000 / (3.0 * GRID_ARG.RATE.fPhaseVoltage));
	ACP.PCC.RATE.Iph = (ACP.INV.RATE.P3 / (3.0 * ACP.PCC.RATE.Vph));
	ACP.PCC.RATE.Z = ACP.PCC.RATE.Vll * ACP.PCC.RATE.Vll / ACP.INV.RATE.P3;

	INVERTER.fTRRatio = TR_RATIO;
//	INVERTER.RATE.fLineVoltage = INV_RATED_VOLTAGE;

	if(PRM_PCS[INV_TRANSFORMER].iValue == 0 ) /* TR LESS TYPE */
		ACP.INV.RATE.Vll = ACP.PCC.RATE.Vll;
	else
		ACP.INV.RATE.Vll = INV_RATED_VOLTAGE;

	ACP.INV.RATE.Vph = ACP.INV.RATE.Vll * INV_SQRT3;
	ACP.INV.RATE.Vph_pk = ACP.INV.RATE.Vph * SQRT2;
	ACP.INV.RATE.Freq = GridFreq;
	ACP.INV.RATE.Omega = 2 * PI * ACP.INV.RATE.Freq;
	ACP.INV.RATE.Iph = (ACP.INV.RATE.P3 / (3.0 * ACP.INV.RATE.Vph));
	ACP.INV.RATE.Iph_pk = ACP.INV.RATE.Iph * SQRT2;

	ACP.TR.RATE.RATIO = ACP.INV.RATE.Vll / ACP.PCC.RATE.Vll;
	ACP.INV.RATE.Z = ACP.INV.RATE.Vll * ACP.INV.RATE.Vll / ACP.INV.RATE.P3;

	SYS_SetLevel2Protection(PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);

	SYS_SetLevel2SeamlessProtection();
	SYS_SetGeneratorConnectPredict();
}

/*
 * Call: SYS_Create(), PARAM_UpdateAll<<PRM_PCS[INV_CAPACITY].bChange>>
 */
void SYS_UpdateKpccKiccLi(void)
{
	float Li, Kp, Ki, Wc;

	Li = FILTER_INDUCTANCE_LI;
	Wc = 2 * PI * 60;
	Kp = 2 * Li * 1e-6 * Wc * 100;
	Ki = Li * 1e-6 * Wc * Wc * 100;


	PRM_PCS[CTRL_LI].iValue = (int)Li;
	PRM_PCS[CTRL_CC_P_GAIN].iValue = (int)Kp;
	PRM_PCS[CTRL_CC_I_GAIN].iValue = (int)Ki;
}

/* Call: PARAM_UpdateAll<<CTRL_LI>>

   - LI 파라미터의 varient는 1이고 500k장비 150uH / 4 일 경우 37.5 이므로
   함수 내에서 LI 파라미터를 직접 사용하지 않고,
   float type 매개 변수로 값을 넘기도록 한다.
*/
#pragma CODE_SECTION (SYS_UpdateLiViaParam, "dp_ctrl")
void SYS_UpdateLiViaParam(float fValue)
{
	float Li, Kp, Ki, Wc;

	Li = fValue;
	Wc = 2 * PI * 60;
	Kp = 2 * Li * 1e-6 * Wc * 100;
	Ki = Li * 1e-6 * Wc * Wc * 100;

	PRM_PCS[CTRL_CC_P_GAIN].iValue = (int)Kp;
	PRM_PCS[CTRL_CC_I_GAIN].iValue = (int)Ki;

	CC_UpdateGains();
}

#pragma CODE_SECTION (SYS_UpdateLI, "dp_ctrl")
void SYS_UpdateLI()
{

}

#pragma CODE_SECTION (SYS_Create, "dp_ctrl")
void SYS_Create(void)
{
	Uns uPeriod = 5;

	memset(&BUTTON, 0, sizeof(BUTTON));
	memset(&INVERTER, 0, sizeof(INVERTER));
	memset(&GRID_ARG, 0, sizeof(GRID_ARG));

	pButton = &BUTTON;
	RecloserSwitchCmd = 0;

#if PCS_AUTO_RESTART_ENB == 0
	SYS_ClearCommand();
#endif

	pButton->bTestRun = OFF;
	pButton->bInverter = OFF;
	INVERTER.gridReconnect.bEnb = FALSE;
	uiSysCallCount = 0;
	INVERTER.RUN_LOAD_OC_DELAY_CNT = 0;

	INVERTER.uPrevStatus = SYS_INV_STOP;
	INVERTER.uStatus = SYS_INV_STOP;

	INVERTER.odtDisconnect = Odt_(&INVERTER.ODT_INV_DISCONNECT, SYS_INV_ODT_DISCONNECT, uPeriod); // + (PRM_PCS[GRID_TIME_SHIFT].iValue * 1000) -> Start 딜레이로 이동
	INVERTER.odtCb1AFault = Odt_(&INVERTER.ODT_CB1A_FAULT, SYS_INV_ODT_CB1A_FAULT, uPeriod);
	INVERTER.odtMC8Fault = Odt_(&INVERTER.ODT_MC8_FAULT, 3000, uPeriod);
	INVERTER.odtMC9Fault = Odt_(&INVERTER.ODT_MC9_FAULT, 3000, uPeriod);
	INVERTER.odtBattCbOn = Odt_(&INVERTER.ODT_INV_BATTCB_ON, SYS_INV_ODT_BATTCB_ON, uPeriod);
	INVERTER.odtCb2Off = Odt_(&INVERTER.ODT_CB2_OFF, SYS_INV_ODT_CB2_OFF, uPeriod);
	INVERTER.odtDcCharge = Odt_(&INVERTER.ODT_DC_CHARGE, SYS_INV_ODT_DCV_CHARGE, uPeriod);

	INVERTER.odtIinvBuildup = Odt_(&INVERTER.ODT_INV_IINV_BUILDUP,
			PARAM_RAW_VAL(CTRL_IINV_VOLTAGE_BUILDUP_TIME), uPeriod);
	INVERTER.odtIinvSync = Odt_(&INVERTER.ODT_INV_SYNC,
			PARAM_RAW_VAL(CTRL_IINV_PLL_STABILIZE_TIME), uPeriod);
	INVERTER.odtIinvSyncTimeOut = Odt_(&INVERTER.ODT_INV_SYNC_TIME_OUT,
			SYS_INV_ODT_SYNC_TIME_OUT, uPeriod);

	INVERTER.odtGI_SlaveDelay = Odt_(&INVERTER.ODT_SLAVE_DELAY, (PRM_PCS[GRID_TIME_SHIFT].iValue * 1000) , uPeriod);
	INVERTER.odtBypNormal = Odt_(&INVERTER.ODT_BYP_NORMAL, SYS_INV_ODT_RESYNC_BYP_OK, uPeriod);		//	STABLEEN 200ms -> 100ms
	INVERTER.odtBypassDelay = Odt_(&INVERTER.ODT_BYPASS_DELAY, 100, uPeriod);
	INVERTER.odtResyncCb2Delay = Odt_(&INVERTER.ODT_RESYNC_CB2_DELAY, 50, uPeriod);
	INVERTER.odtSeamlessDisable = Odt_(&INVERTER.ODT_SEAMLESS_DISABLE, 5000, uPeriod);
	INVERTER.odtEveCheck = Odt_(&INVERTER.ODT_EVE_CHECK, 3000, uPeriod);
	INVERTER.odtBypassVok = Odt_(&INVERTER.ODT_BYPASS_VOK, 3000, uPeriod);
	INVERTER.odtHoldLoadP = Odt_(&INVERTER.ODT_HOLD_LOAD_P, PARAM_RAW_VAL(HOLD_LOAD_POWER_TIME), uPeriod);

	SYS_UpdateParameter();
	SYS_UpdateKpccKiccLi();

	if( PRM_PCS[COSPHI_FACTOR_LIMIT].iValue < 80 )
	{
		PRM_PCS[COSPHI_FACTOR_LIMIT].iValue = 80;
	}
	else if(PRM_PCS[COSPHI_FACTOR_LIMIT].iValue > 100)
	{
		PRM_PCS[COSPHI_FACTOR_LIMIT].iValue = 100;
	}

	SYS_ESLV_Create();
	//	16.07.28 Yang
	CTRL_BYP_EVT_OPERATION_Create();
	Sag_Evt_History_Create();

	InverterStateShadow = SYS_INV_INIT;
}

float fInvVabcMean;
float fGridE[SYS_LINE_NUM];
float fBypassE[SYS_LINE_NUM];


unsigned int gi_acgen_delay = 0;
Bool bMonitorOn = 0;

//by JCNET
// 기존 InverterStateTransition 관련 함수는 사용하지 않아서 SYSTEM_state.c로 빼내고 include형태로 함
// 화일을 단순화 목적
#if 0 //by JCNET
#include "SYSTEM_old_state.inc"
#endif
Bool SYS_Get20msUnitFotEvent()
{
	return u20msUnitForEvent;
}

/*******************************************************************/
/* c_int25 - 2.5msec 인터럽트함수                                  	   */
/* Parameters : void                                               */
/* Returns : void                                                  */
/*******************************************************************/
#pragma CODE_SECTION (c_int25, "dp_ctrl")
void c_int25(void)
{
	static Bool b5msec = FALSE;

	// FRT TEST 부분 삭제함.
	if (!bSystemStarted)
		return;

#if SYSTEM_EXECUTION_PERIOD
	SYS_startTime = CLK_gethtime();
	if ( SYS_startTime < SYS_endTime )
	SYS_uExecutionPeriod = (0xFFFFFFFF - SYS_endTime + SYS_startTime) * 0.0266667;
	else
	SYS_uExecutionPeriod = (SYS_startTime - SYS_endTime) * 0.0266667;
#endif

	if (b5msec == FALSE)
	{
		FLT_Handling();

		// 151113 상태 접점 인식이 느려 제어기 전환에 지연이 생겨 Digital Input 을 제어 루틴에서 수행.
#if FAST_DI_POLLING == 0
		MCCB_UpdateAllStatus();
#endif

		CAN_Timer_5ms();
		CAN_GbiTimer_5ms();

		b5msec = TRUE;
	}
	else
	{
		// 5ms
#if 0 //by JCNET
		SYS_UpdateInverterStatus();
#else
		SYS_UpdateStateMachine();
#endif
		//-TEST모드에서 MC를 강제로 control할 경우에는 주석해지.
		//if (PRM_PCS[CTRL_TEST_MODE].iValue == 0 ) // Only Normal Mode
		{
			SYS_UpdateMCbStatus(M_CB1_BATT);
#if STABLEEN_HILL_ENABLE == 0
//by JCNET			SYS_UpdateMCbStatus(M_CB4_SSW);
			SYS_UpdateMCbStatus(M_CB2_SSW);
#endif
//by JCNET			SYS_UpdateMCbStatus(M_CB3);
		}


		Cnt20ms++;
		if (Cnt20ms >= SYS_20MSEC_PERIOD)
		{
			//b20msTick = TRUE;
			if (++u20msCnt >= 50)
				u20msCnt = 0;
			u20msUnitForEvent = u20msCnt << 10;
			Cnt20ms = 0;
		}

		b5msec = FALSE;
	}

#if SYSTEM_EXECUTION_PERIOD
	SYS_endTime = CLK_gethtime();
#endif

}

void SYS_AddInfoNode(void)
{
	TRC_AddNode(TRC_81_SYS_INV_STAT, TRC_INT, &INVERTER.uStatus);
#if SYSTEM_EXECUTION_PERIOD
	TRC_AddNode( TRC_88_SYS_PERIOD_TIME, TRC_INT, &SYS_uExecutionPeriod);
#endif
}

void SYS_SetLevel2Protection(Bool bEnable)
{
	int i;
	for (i = SYS_GRID_OV_LEVEL1; i <= SYS_INV_UF_LEVEL1; i++)
	{
		SYS_UpdateLevel2ProtectionLevel(i, bEnable);
	}

	if (bEnable)
	{
		for (i = SYS_GRID_OV_LEVEL2; i <= SYS_GRID_UF_LEVEL2; i++)
		{
			SYS_UpdateLevel2ProtectionLevel(i, bEnable);
		}
	}
}

#pragma CODE_SECTION (SYS_SetLevel2SeamlessProtection, "dp_ctrl")
void SYS_SetLevel2SeamlessProtection()
{
	float TripLevel;

	TripLevel = ACP.PCC.RATE.Freq - PARAM_VAL(GRID_UF_LEVEL2_SEAMLESS);
	LVL_Initialize(&ObjLVL.GridUFLevel2_seamless, TripLevel, SYS_FREQ_HYSTERESIS, PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue);

	TripLevel = ACP.PCC.RATE.Freq + PARAM_VAL(GRID_OF_LEVEL2_SEAMLESS);
	LVL_Initialize(&ObjLVL.GridOFLevel2_seamless, TripLevel, SYS_FREQ_HYSTERESIS, PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue);
}

#pragma CODE_SECTION (SYS_UpdateLevel2ProtectionLevel, "dp_ctrl")
void SYS_UpdateLevel2ProtectionLevel(Uns uLevel, Bool bLevel2Enable)
{
	float hysteresis;
	Uns uTripCnt;
	float TripLevel;

	switch (uLevel)
	{
	case SYS_GRID_UV_LEVEL1:
		TripLevel = ACP.PCC.RATE.Vph * (float) PARAM_VAL(GRID_UV_LEVEL1) * 0.01;
		hysteresis = SYS_VOLT_HYSTERESIS * INV_SQRT3;

		if (bLevel2Enable)
		{
			uTripCnt = (PRM_PCS[GRID_UV_LEVEL1_TRIP_TIME].iValue / 20) - 2;
		}
		else
			uTripCnt = PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue;

		LVL_Initialize(&ObjLVL.GridUVLevel1[0], TripLevel, hysteresis, uTripCnt);
		LVL_Initialize(&ObjLVL.GridUVLevel1[1], TripLevel, hysteresis, uTripCnt);
		LVL_Initialize(&ObjLVL.GridUVLevel1[2], TripLevel, hysteresis, uTripCnt);

		//if( PRM_PCS[CTRL_BYP_V_GRID_TEST_MODE].iValue & 0x1 )
		{
			LVL_Initialize(&ObjLVL.BypassUVLevel1[0], TripLevel, hysteresis, uTripCnt);
			LVL_Initialize(&ObjLVL.BypassUVLevel1[1], TripLevel, hysteresis, uTripCnt);
			LVL_Initialize(&ObjLVL.BypassUVLevel1[2], TripLevel, hysteresis, uTripCnt);
		}

		break;
	case SYS_GRID_UV_LEVEL2:
		TripLevel = ACP.PCC.RATE.Vph * (float) PARAM_VAL(GRID_UV_LEVEL2) * 0.01;
		hysteresis = SYS_VOLT_HYSTERESIS * INV_SQRT3;
		uTripCnt = (PRM_PCS[GRID_UV_LEVEL2_TRIP_TIME].iValue / 20) - 2;
		LVL_Initialize(&ObjLVL.GridUVLevel2[0], TripLevel, hysteresis, uTripCnt);
		LVL_Initialize(&ObjLVL.GridUVLevel2[1], TripLevel, hysteresis, uTripCnt);
		LVL_Initialize(&ObjLVL.GridUVLevel2[2], TripLevel, hysteresis, uTripCnt);

		break;
	case SYS_GRID_OV_LEVEL1:
		TripLevel = ACP.PCC.RATE.Vph * (float) PARAM_VAL(GRID_OV_LEVEL1) * 0.01;
		hysteresis = SYS_VOLT_HYSTERESIS * INV_SQRT3;

		if (bLevel2Enable)
		{
			uTripCnt = (PRM_PCS[GRID_OV_LEVEL1_TRIP_TIME].iValue / 20) - 2;
		}
		else
			uTripCnt = PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue;

		LVL_Initialize(&ObjLVL.GridOVLevel1[0], TripLevel, hysteresis, uTripCnt);
		LVL_Initialize(&ObjLVL.GridOVLevel1[1], TripLevel, hysteresis, uTripCnt);
		LVL_Initialize(&ObjLVL.GridOVLevel1[2], TripLevel, hysteresis, uTripCnt);

		//if( PRM_PCS[CTRL_OPTION].iValue & 0x2 )
		{
			LVL_Initialize(&ObjLVL.BypassOVLevel1[0], TripLevel, hysteresis, uTripCnt);
			LVL_Initialize(&ObjLVL.BypassOVLevel1[1], TripLevel, hysteresis, uTripCnt);
			LVL_Initialize(&ObjLVL.BypassOVLevel1[2], TripLevel, hysteresis, uTripCnt);
		}
		break;
	case SYS_GRID_OV_LEVEL2:
		TripLevel = ACP.PCC.RATE.Vph * (float) PARAM_VAL(GRID_OV_LEVEL2) * 0.01;
		hysteresis = SYS_VOLT_HYSTERESIS * INV_SQRT3;
		uTripCnt = (PRM_PCS[GRID_OV_LEVEL2_TRIP_TIME].iValue / 20) - 2;
		LVL_Initialize(&ObjLVL.GridOVLevel2[0], TripLevel, hysteresis, uTripCnt);
		LVL_Initialize(&ObjLVL.GridOVLevel2[1], TripLevel, hysteresis, uTripCnt);
		LVL_Initialize(&ObjLVL.GridOVLevel2[2], TripLevel, hysteresis, uTripCnt);
		break;
	case SYS_GRID_UF_LEVEL1:
		TripLevel = ACP.PCC.RATE.Freq - PARAM_VAL(GRID_UF_LEVEL1);

		if (bLevel2Enable)
		{
			uTripCnt = (PRM_PCS[GRID_UF_LEVEL1_TRIP_TIME].iValue / 20) - 2;
		}
		else
			uTripCnt = PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue;

		LVL_Initialize(&ObjLVL.GridUFLevel1, TripLevel, SYS_FREQ_HYSTERESIS, uTripCnt);
		LVL_Initialize(&ObjLVL.BypassUFLevel1, TripLevel, SYS_FREQ_HYSTERESIS, uTripCnt);

		break;
	case SYS_GRID_UF_LEVEL2:
		TripLevel = ACP.PCC.RATE.Freq - PARAM_VAL(GRID_UF_LEVEL2);
		uTripCnt = (PRM_PCS[GRID_UF_LEVEL2_TRIP_TIME].iValue / 20) - 2;
		LVL_Initialize(&ObjLVL.GridUFLevel2, TripLevel, SYS_FREQ_HYSTERESIS, uTripCnt);
		break;
	case SYS_GRID_OF_LEVEL1:
		TripLevel = ACP.PCC.RATE.Freq + PARAM_VAL(GRID_OF_LEVEL1);
		if (bLevel2Enable)
		{
			uTripCnt = (PRM_PCS[GRID_OF_LEVEL1_TRIP_TIME].iValue / 20) - 2;
		}
		else
			uTripCnt = PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue;

		LVL_Initialize(&ObjLVL.GridOFLevel1, TripLevel, SYS_FREQ_HYSTERESIS, uTripCnt);
		LVL_Initialize(&ObjLVL.BypassOFLevel1, TripLevel, SYS_FREQ_HYSTERESIS, uTripCnt);

		break;
	case SYS_GRID_OF_LEVEL2:
		TripLevel = ACP.PCC.RATE.Freq + PARAM_VAL(GRID_OF_LEVEL2);
		uTripCnt = (PRM_PCS[GRID_OF_LEVEL2_TRIP_TIME].iValue / 20) - 2;
		LVL_Initialize(&ObjLVL.GridOFLevel2, TripLevel, SYS_FREQ_HYSTERESIS, uTripCnt);
		break;

	case SYS_INV_UV_LEVEL1:
		TripLevel = ACP.INV.RATE.Vph * PARAM_VAL(IS_UV_LEVEL1) * 0.01;
		uTripCnt = PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue;
		LVL_Initialize(&ObjLVL.InvUV, TripLevel, SYS_VOLT_HYSTERESIS * INV_SQRT3, uTripCnt);
		break;
	case SYS_INV_OV_LEVEL1:
		TripLevel = ACP.INV.RATE.Vph * PARAM_VAL(IS_OV_LEVEL1) * 0.01;
		uTripCnt = PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue;
		LVL_Initialize(&ObjLVL.InvOV, TripLevel, SYS_VOLT_HYSTERESIS * INV_SQRT3, uTripCnt);
		break;
	case SYS_INV_UF_LEVEL1:
		TripLevel = ACP.INV.RATE.Freq - PARAM_VAL(IS_UF_LEVEL1);
		uTripCnt = PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue;
		LVL_Initialize(&ObjLVL.InvUF, TripLevel, SYS_FREQ_HYSTERESIS, uTripCnt);
		break;
	case SYS_INV_OF_LEVEL1:
		TripLevel = ACP.INV.RATE.Freq + PARAM_VAL(IS_OF_LEVEL1);
		uTripCnt = PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue;
		LVL_Initialize(&ObjLVL.InvOF, TripLevel, SYS_FREQ_HYSTERESIS, uTripCnt);
		break;
	}
}

#pragma CODE_SECTION (SYS_ReConnectionCondition, "dp_ctrl")
Bool SYS_ReConnectionCondition(void)
{
	return !PARAM_RAW_VAL(GRID_RECONNECT_COND_ENB)? TRUE :
	((ACP.PCC.RATE.Freq -2.5 <= EXCTRL.fltFreq.fOut && EXCTRL.fltFreq.fOut <= ACP.PCC.RATE.Freq + 0.05)
			&& (ACP.PCC.RATE.Vph*0.95 <= EXCTRL.fltFsGridV[0].fOut
					&& ACP.PCC.RATE.Vph*0.95 <= EXCTRL.fltFsGridV[1].fOut
					&& ACP.PCC.RATE.Vph*0.95 <= EXCTRL.fltFsGridV[2].fOut)
	);
}

void SYS_CheckBatteryCurrent()
{
#if	DISABLE_CURRENT_SENSOR_FLT == 0

	if( CC_GetCVCEnable() && INVCTRL.IqeRef != 0)
	{
		// 운전 중 DC측 배터리 전류가 나오지 않을 경우.
		if( fabs(INVERTER.MEASURE[0].fCurrent) > 50 )
		{
			if( fabs(BATCTRL.fltI.fOut) < 2) // 2s (1초이상 )
			{
				BATCTRL.uBattIFaultCount++;
				if( BATCTRL.uBattIFaultCount > 800 )
				{
					FLT_Raise(FLTH_BATT_I);
					BATCTRL.uBattIFaultCount = 0;
				}
			}
			else
			{
				// Added 140310
				BATCTRL.uBattIFaultCount = 0;
			}
		}
		else
			BATCTRL.uBattIFaultCount = 0;
	}

#endif
}

Bool GetDcChargerOnOff(void)
{
	return bDcChargerOnOff;
}

#pragma CODE_SECTION (Prevent_Immediate_Grid_Reconnect, "dp_ctrl")
void Prevent_Immediate_Grid_Reconnect(void)
{
	/*------------- RUN Sate에서 Grid Fault 시 5분간 재투입 방지  -------------------*/
	if(FLT_GetGridFailureEvent() == ON && !INVERTER.gridReconnect.bEnb)
		INVERTER.gridReconnect.bEnb = TRUE;

	if(INVERTER.gridReconnect.bEnb)
	{
		if( INVERTER.gridReconnect.uiProgressionSecond > PRM_PCS[DGT_REV_00].iValue )
		{
			// ES-LV 타입에서 정전 상황에 자동으로 GI MODE - SYSTEM ON 해야 할 경우 하단 if 문 삭제.
			//180731	if( PRM_PCS[SYS_OPTION].iValue & 0x10)
			//180731		SYS_SetInverterCommand(ON); // 재기동
			INVERTER.gridReconnect.bEnb = FALSE;
			FLT_SetGridFailureEvent(FALSE);
		}
	}
	/*--------------------------------------------------------------*/
}

/*
 * call: MVP_Process20msec()
 */
#pragma CODE_SECTION (SYS_VoltFreqMonitor, "dp_ctrl")
void SYS_VoltFreqMonitor(void)
{
	float InvFreq, GridFreq;//, BypFreq;
	Bool bLevel2ProtectionEnable = PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1;
	int i;
	Uns uGridOVLevel1Flag = 0;
	Uns uGridUVLevel1Flag = 0;
	Uns uGridOVLevel2Flag = 0;
	Uns uGridUVLevel2Flag = 0;
	Uns uBypassOVLevel1Flag = 0;
	Uns uBypassUVLevel1Flag = 0;

	fInvVabcMean = 0;

// TODO .. level check skipped now for test by JCNET
// JCNET!!!
//	return;

	bMonitorOn = TRUE;

	for (i = 0; i < 3; i++)
	{
		fInvVabcMean += RMS_GetInstantValue(pRMSConE, i);
		fGridE[i] = EXCTRL.fltFsGridV[i].fOut;
		fBypassE[i] = EXCTRL.fltFsBypV[i].fOut;

		LVL_CheckOver(&ObjLVL.GridOVLevel1[i], fGridE[i]);
		LVL_CheckUnder(&ObjLVL.GridUVLevel1[i], fGridE[i]);
		uGridOVLevel1Flag += ObjLVL.GridOVLevel1[i].bFlag;
		uGridUVLevel1Flag += ObjLVL.GridUVLevel1[i].bFlag;

		LVL_CheckOver(&ObjLVL.BypassOVLevel1[i], fBypassE[i]);
		LVL_CheckUnder(&ObjLVL.BypassUVLevel1[i], fBypassE[i]);
		uBypassOVLevel1Flag += ObjLVL.BypassOVLevel1[i].bFlag;
		uBypassUVLevel1Flag += ObjLVL.BypassUVLevel1[i].bFlag;
	}
	fInvVabcMean *= INV_3;

	if (bLevel2ProtectionEnable)
	{
		for (i = 0; i < 3; i++)
		{
			LVL_CheckOver(&ObjLVL.GridOVLevel2[i], fGridE[i]);
			LVL_CheckUnder(&ObjLVL.GridUVLevel2[i], fGridE[i]);
			uGridOVLevel2Flag += ObjLVL.GridOVLevel2[i].bFlag;
			uGridUVLevel2Flag += ObjLVL.GridUVLevel2[i].bFlag;
		}
	}

	InvFreq = INVCTRL.fltFreq.fOut;
	GridFreq = EXCTRL.fltFreq.fOut;
	//BypFreq = EXCTRL.fltFreq_Byp.fOut;

#if FRT_TEST_MODE	// hooking
	if ( PARAM_RAW_VAL(ADD_VIRT_FREQ) > 0)
	{
		GridFreq = PARAM_VAL(ADD_VIRT_FREQ);
	}
#endif

	if (PRM_PCS[CTRL_TEST_MODE].iValue == 1)
	{
			FLT_Clear(FLTH_INV_OV);
			FLT_Clear(FLTH_INV_UV);
			FLT_Clear(FLTH_INV_OF);
			FLT_Clear(FLTH_INV_UF);

			FLT_Clear(FLTH_GRID_OV_LEVEL1);
			FLT_Clear(FLTH_GRID_UV_LEVEL1);
			FLT_Clear(FLTH_GRID_OF_LEVEL1);
			FLT_Clear(FLTH_GRID_UF_LEVEL1);

			FLT_Clear(FLTH_GRID_OV_LEVEL2);
			FLT_Clear(FLTH_GRID_UV_LEVEL2);
			FLT_Clear(FLTH_GRID_OF_LEVEL2);
			FLT_Clear(FLTH_GRID_UF_LEVEL2);
	}
	else
	{
		if ((INVERTER.uStatus == SYS_INV_RUN ||
				INVERTER.uStatus == SYS_INV_ISLANDING || INVERTER.uStatus == SYS_INV_RE_SYNC )&& (/*160331*/ INVERTER.bRun == ON) )
		{
			if( bMonitorOn )
				LVL_CheckOver(&ObjLVL.InvOV, fInvVabcMean);

			if (ObjLVL.InvOV.bFlag)
				FLT_Raise(FLTH_INV_OV);
#if DISABLE_AUTO_FLT_RST == 0
			else
				FLT_Clear(FLTH_INV_OV);
#endif

			// 150128 June for 전기안전연구원
			if( INVERTER.uStatus == SYS_INV_ISLANDING )
			{
				ObjLVL.InvUV.uTripCnt = 20;
			}
			else
			{
				ObjLVL.InvUV.uTripCnt = PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue;
			}

			if(bMonitorOn)
				LVL_CheckUnder(&ObjLVL.InvUV, fInvVabcMean);

			if (ObjLVL.InvUV.bFlag)
			{

				FLT_Raise(FLTH_INV_UV);
			}
#if DISABLE_AUTO_FLT_RST == 0
			else
				FLT_Clear(FLTH_INV_UV);
#endif

			if( !bMonitorOn || CTRL.GRT.bEnable ) // 160912
			{
				ObjLVL.InvOF.bFlag = FALSE;
				ObjLVL.InvOF.uCnt = 0;
			}
			else
			{
				LVL_CheckOver(&ObjLVL.InvOF, InvFreq);
			}

			if (ObjLVL.InvOF.bFlag)
				FLT_Raise(FLTH_INV_OF);
#if DISABLE_AUTO_FLT_RST == 0
			else
				FLT_Clear(FLTH_INV_OF);
#endif

			if( !bMonitorOn || CTRL.GRT.bEnable) // 160912
			{
				//wait...
				ObjLVL.InvUF.bFlag = FALSE;
				ObjLVL.InvUF.uCnt = 0;
			}
			else
			{
				LVL_CheckUnder(&ObjLVL.InvUF, InvFreq);
			}

			if (ObjLVL.InvUF.bFlag)
				FLT_Raise(FLTH_INV_UF);
#if DISABLE_AUTO_FLT_RST == 0
			else
				FLT_Clear(FLTH_INV_UF);
#endif

		}

		/*
		 * Check GRID side fault
		 */
		// Over Volt Level1
		if (uGridOVLevel1Flag)
		{
#if 0 // 180626 GRID OF, UF -> 고장 처리
			if(CTRL.INV.operation_mode != PARAM_OPERATION_MODE_GC)
			{
				if (INVERTER.uStatus != SYS_INV_STOP || INVERTER.uStatus != SYS_INV_FAULT )
				{
					FLT_Raise(FLTL_GRID_OV);
				}
			}
			else
#endif
				FLT_Raise(FLTH_GRID_OV_LEVEL1);
		}
		else
		{
			FLT_Clear(FLTL_GRID_OV);
#if DISABLE_AUTO_FLT_RST == 0
			FLT_Clear(FLTH_GRID_OV_LEVEL1);
#endif
		}


		// Under Volt Level1
		if (uGridUVLevel1Flag)
		{
#if 0 // 180626 GRID OF, UF -> 고장 처리
			if(CTRL.INV.operation_mode != PARAM_OPERATION_MODE_GC )
			{
				if (INVERTER.uStatus != SYS_INV_STOP || INVERTER.uStatus != SYS_INV_FAULT )
				{
					FLT_Raise(FLTL_GRID_UV);
				}
			}
			else
#endif
				//FLT_Raise(FLTH_GRID_UV_LEVEL1);
				FLT_Raise(FLTL_GRID_UV);
		}
		else
		{
			FLT_Clear(FLTL_GRID_UV);
#if DISABLE_AUTO_FLT_RST == 0
			FLT_Clear(FLTH_GRID_UV_LEVEL1);
#endif
		}

		if(PRM_PCS[BESS72].iValue == 1 && INVERTER.uStatus == SYS_INV_RE_SYNC)
			bResync_OFUF_Disable = TRUE;
		else
			bResync_OFUF_Disable = FALSE;


		// Over Freq Level1
		if( CTRL.GRT.bEnable || bResync_OFUF_Disable ) // 160912
		{
			ObjLVL.GridOFLevel1.bFlag = FALSE;
			ObjLVL.GridOFLevel1.uCnt = 0;
		}
		else
		{
			LVL_CheckOver(&ObjLVL.GridOFLevel1, GridFreq );
		}

		if (ObjLVL.GridOFLevel1.bFlag)
		{
#if 0 // 180626 GRID OF, UF -> 고장 처리
			if(CTRL.INV.operation_mode != PARAM_OPERATION_MODE_GC)
			{
				if (INVERTER.uStatus != SYS_INV_STOP || INVERTER.uStatus != SYS_INV_FAULT )
				{
					if(INVERTER.uStatus != SYS_INV_RE_SYNC)
						FLT_Raise(FLTL_GRID_OF);
				}
				FLT_Clear(FLTH_GRID_OF_LEVEL1);
			}
			else
#endif
				FLT_Raise(FLTH_GRID_OF_LEVEL1);
		}
		else
		{
			FLT_Clear(FLTL_GRID_OF);
#if DISABLE_AUTO_FLT_RST == 0
			FLT_Clear(FLTH_GRID_OF_LEVEL1);
#endif
		}

#if 0 /* 180626 GRID OF UF WARN 제거 */
		/*
		 * 1603
		 * Over Freq Level2 for seamless
		 */
		if( CTRL.GRT.bEnable || bResync_OFUF_Disable ) // 160912
		{
			ObjLVL.GridOFLevel2_seamless.bFlag = FALSE;
			ObjLVL.GridOFLevel2_seamless.uCnt = 0;
		}
		else
		{
			LVL_CheckOver(&ObjLVL.GridOFLevel2_seamless, GridFreq );
		}

		if (ObjLVL.GridOFLevel2_seamless.bFlag)
		{
			if(CTRL.INV.operation_mode != PARAM_OPERATION_MODE_GC)
			{
				if (INVERTER.uStatus == SYS_INV_RUN )
					FLT_Raise(FLTL_GRID_OF2_SEAMLESS);
			}
		}
		else
		{
			FLT_Clear(FLTL_GRID_OF2_SEAMLESS);
		}
#endif

		// Under Freq Level1
		if( CTRL.GRT.bEnable || bResync_OFUF_Disable ) // 160912
		{
			ObjLVL.GridUFLevel1.bFlag = FALSE;
			ObjLVL.GridUFLevel1.uCnt = 0;
		}
		else
		{
			// Under Freq Level1
			LVL_CheckUnder(&ObjLVL.GridUFLevel1, GridFreq);
		}

		if (ObjLVL.GridUFLevel1.bFlag)
		{
#if 0 // 180626 GRID OF, UF -> 고장 처리
			if(CTRL.INV.operation_mode != PARAM_OPERATION_MODE_GC)
			{
				if (INVERTER.uStatus != SYS_INV_STOP || INVERTER.uStatus != SYS_INV_FAULT )
				{
					if(INVERTER.uStatus != SYS_INV_RE_SYNC)
						FLT_Raise(FLTL_GRID_UF);
				}
			}
			else
#endif
				FLT_Raise(FLTH_GRID_UF_LEVEL1);
		}
		else
		{
			FLT_Clear(FLTL_GRID_UF);
#if DISABLE_AUTO_FLT_RST == 0
			FLT_Clear(FLTH_GRID_UF_LEVEL1);
#endif
		}

#if 0 /* 180626 GRID OF UF WARN 제거 */
		/*
		 * 1603
		 * Under Freq Level2 for seamless
		 */
		if( CTRL.GRT.bEnable || bResync_OFUF_Disable) // 160912
		{
			ObjLVL.GridUFLevel2_seamless.bFlag = FALSE;
			ObjLVL.GridUFLevel2_seamless.uCnt = 0;
		}
		else
		{
			LVL_CheckUnder(&ObjLVL.GridUFLevel2_seamless, GridFreq);
		}

		if (ObjLVL.GridUFLevel2_seamless.bFlag)
		{
			if(CTRL.INV.operation_mode != PARAM_OPERATION_MODE_GC)
			{
				if (INVERTER.uStatus == SYS_INV_RUN )
					FLT_Raise(FLTL_GRID_UF2_SEAMLESS);
			}
		}
		else
		{
			FLT_Clear(FLTL_GRID_UF2_SEAMLESS);
		}
#endif

		if (bLevel2ProtectionEnable)
		{
			// Over Volt Level2
			if (uGridOVLevel2Flag)
			{
					FLT_Raise(FLTH_GRID_OV_LEVEL2);
			}
#if DISABLE_AUTO_FLT_RST == 0
			else
				FLT_Clear(FLTH_GRID_OV_LEVEL2);
#endif

			// Under Volt Level2
			if (uGridUVLevel2Flag)
			{
				FLT_Raise(FLTH_GRID_UV_LEVEL2);
			}
#if DISABLE_AUTO_FLT_RST == 0
			else
				FLT_Clear(FLTH_GRID_UV_LEVEL2);
#endif

			// Over Freq Level2
			LVL_CheckOver( &ObjLVL.GridOFLevel2, GridFreq );
			if (ObjLVL.GridOFLevel2.bFlag)
			{
				FLT_Raise(FLTH_GRID_OF_LEVEL2);
			}
#if DISABLE_AUTO_FLT_RST == 0
			else
			FLT_Clear(FLTH_GRID_OF_LEVEL2);
#endif
			// Under Freq Level2
			LVL_CheckUnder(&ObjLVL.GridUFLevel2, GridFreq);
			if (ObjLVL.GridUFLevel2.bFlag)
			{
				FLT_Raise(FLTH_GRID_UF_LEVEL2);
			}
#if DISABLE_AUTO_FLT_RST == 0
			else
				FLT_Clear(FLTH_GRID_UF_LEVEL2);
#endif
		}
		else // not Level2 Protection
		{
#if DISABLE_AUTO_FLT_RST == 0
//			FLT_Clear(FLTH_GRID_OV_LEVEL2);
//			FLT_Clear(FLTH_GRID_UV_LEVEL2);
//			FLT_Clear(FLTH_GRID_OF_LEVEL2);
//			FLT_Clear(FLTH_GRID_UF_LEVEL2);
#endif
		}
	}
}

#pragma CODE_SECTION (SYS_VoltFreqFaultClear, "dp_ctrl")
// Call: MVP_Process20msec(). FreqMonitor 후 바로 수행
void SYS_VoltFreqFaultClear(void)
{
	float InvFreq;
	float GridFreq, BypFreq;
	int i;

	InvFreq = INVCTRL.fltFreq.fOut;
	GridFreq = EXCTRL.fltFreq.fOut;
	BypFreq = EXCTRL.fltFreq_Byp.fOut;

#if FRT_TEST_MODE	// hooking
	if ( PARAM_RAW_VAL(ADD_VIRT_FREQ) > 0)
	{
		GridFreq = PARAM_VAL(ADD_VIRT_FREQ);
	}
#endif

	/*
	 * INV side
	 */
	LVL_CheckOverClear(&ObjLVL.InvOV, fInvVabcMean);
	LVL_CheckUnderClear(&ObjLVL.InvUV, fInvVabcMean);
	LVL_CheckOverClear(&ObjLVL.InvOF, InvFreq);
	LVL_CheckUnderClear(&ObjLVL.InvUF, InvFreq);

	/*
	 * GRID side
	 */
	// LEVEL1
	for (i = 0; i < 3; i++)
	{
		LVL_CheckOverClear(&ObjLVL.GridOVLevel1[i], fGridE[i]);
		LVL_CheckUnderClear(&ObjLVL.GridUVLevel1[i], fGridE[i]);
	}

	LVL_CheckOverClear(&ObjLVL.GridOFLevel1, GridFreq);
	LVL_CheckUnderClear(&ObjLVL.GridUFLevel1, GridFreq);

	LVL_CheckOverClear(&ObjLVL.GridOFLevel2_seamless, GridFreq);
	LVL_CheckUnderClear(&ObjLVL.GridUFLevel2_seamless, GridFreq);


	// LEVEL2
	if (PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1)
	{
		for (i = 0; i < 3; i++)
		{
			LVL_CheckOverClear(&ObjLVL.GridOVLevel2[i], fGridE[i]);
			LVL_CheckUnderClear(&ObjLVL.GridUVLevel2[i], fGridE[i]);
		}
		LVL_CheckOverClear(&ObjLVL.GridOFLevel2, GridFreq);
		LVL_CheckUnderClear(&ObjLVL.GridUFLevel2, GridFreq);
	}

	/*
	 * BYPASS side
	 */
	// LEVEL1
	for (i = 0; i < 3; i++)
	{
		LVL_CheckOverClear(&ObjLVL.BypassOVLevel1[i], fBypassE[i]);
		LVL_CheckUnderClear(&ObjLVL.BypassUVLevel1[i], fBypassE[i]);
	}
	LVL_CheckOverClear(&ObjLVL.BypassOFLevel1, BypFreq);
	LVL_CheckUnderClear(&ObjLVL.BypassUFLevel1, BypFreq);
}

#pragma CODE_SECTION (SYS_BattVoltFaultClear, "dp_ctrl")
void SYS_BattVoltFaultClear(void)
{
	static unsigned int cnt = 0;
	if (FLT_GetEachStatus(FLTH_BATT_OV) && BATCTRL.fltDCBattVForCC.fOut < PARAM_VAL(BATT_OV_LEVEL)) //-> 13.3.3 OP LVL 에서 BATT_OV_LEVEL 으로 수정함.
	{
		if (++cnt > 500 /*10000 / 20*/)
		{ //10 sec (this function will be called every 20ms)
#if DISABLE_AUTO_FLT_RST == 0
			FLT_Clear(FLTH_BATT_OV);
#endif
			cnt = 0;
		}
	}
	else
	{
		cnt = 0;
	}
}
//13.3.3
#pragma CODE_SECTION (SYS_BattUnderVoltFaultClear, "dp_ctrl")
void SYS_BattUnderVoltFaultClear(void)
{
	static unsigned int cnt = 0;
	//if (FLT_GetEachStatus(FLTH_BATT_UV) && BATCTRL.fltDCBattVForCC.fOut > PARAM_VAL(BATT_UV_LEVEL))
	if(FLT_GetEachStatus(FLTH_BATT_UV))
	{
		if (++cnt > 250 /*5000 / 20*/)
		{ //5 sec (this function will be called every 20ms)
#if DISABLE_AUTO_FLT_RST == 0
			FLT_Clear(FLTH_BATT_UV);
#endif
			cnt = 0;
		}
	}
	else
	{
		cnt = 0;
	}
}
void SYS_SetNVSRAMRunCommand(Bool ON_OFF)
{
	ReqCmdIdle.bReqSetCmd = TRUE;
	ReqCmdIdle.SetCmdVal = ON_OFF;

	//180731 ON 상태를 기록하여 제어전원이 재 인가되어도 기동 되도록 한다.
#if PCS_AUTO_RESTART_ENB == 1
	NvSRAM_SetRunCommand(ON_OFF);
#endif
}

void SYS_ClearCommand(void)
{
	pButton->bTestRun = OFF;
	pButton->bInverter = OFF;
	SYS_SetNVSRAMRunCommand(OFF );
}

void SYS_SetTestRunCommand(Bool ON_OFF)
{
	pButton->bTestRun = ON_OFF;
	if (ON_OFF == ON )
		EVT_Store(EVT_TESTRUN_ON);
	else
		EVT_Store(EVT_TESTRUN_OFF);
}

Bool SYS_CheckTestRunCmd(void)
{
	if (pButton->bTestRun)
		return TRUE;
	else
		return FALSE;
}

void SYS_SetInverterCommand(Bool ON_OFF)
{
	pButton->bInverter = ON_OFF;
	SYS_SetNVSRAMRunCommand(ON_OFF);
}

void SYS_CheckNVSRAMRun(void)
{
#if PCS_AUTO_RESTART_ENB == 1
	if (NvSRAM_CheckRunCommand())
	{
		pButton->bInverter = ON;
		EVT_Store(EVT_AUTO_RESTART);
	}
#endif
}


/*
 *********************************************************************************************************
 *                                         인버터 운전 명령 검사
 *
 * Description: MMI 또는 CMT에서 발생한 인버터 운전 명령 검사
 *
 * Arguments  : none
 *
 * Returns    :
 *********************************************************************************************************
 */
Bool SYS_CheckInvCmdOn(void)
{
	if (pButton->bInverter)
		return TRUE;
	else
		return FALSE;
}

#if 0 //[CO] by JCNET
#pragma CODE_SECTION (SYS_Byp20_StopFault, "dp_ctrl")
void SYS_Byp20_StopFault()
{
	//float Ratio;

	//Ratio = PARAM_VAL(GRID_UV_LEVEL1) * 0.01;
#if 0
	if( !CTRL_BYP_NormalOK() || (fBypassE[0] < ACP.PCC.RATE.Vph * Ratio)
			|| (fBypassE[1] < ACP.PCC.RATE.Vph * Ratio) || (fBypassE[2] < ACP.PCC.RATE.Vph * Ratio))
	{
		gd(81);
		GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);

		if( MC_GetStatus(STATUS_CB4) )
		{
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
				MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);
			else
				MC_UpdateStatus(CTRL_CB4_ON, OPEN);
		}
		ODT_Initialize(GenBlock.odtGridConnectDelay);
	}
	else if( CTRL_BYP_NormalOK() && (fBypassE[0] > ACP.PCC.RATE.Vph * Ratio)
			&& (fBypassE[1] > ACP.PCC.RATE.Vph * Ratio) && (fBypassE[2] > ACP.PCC.RATE.Vph * Ratio))
	{
		if (ODT_Update(GenBlock.odtGridConnectDelay, TRUE) == ODT_FINISH)
		{
			/*	2016.06.10	Yang - STABLEEN	*/
			if(INVERTER.uStatus == SYS_INV_FAULT)
			{
				GPIO_StaticSwitch(GPIO_STATIC_SW_ON);

				if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
					MCB_UpdateCmd(M_CB4_SSW, CB_CMD_ON);
				else
					MC_UpdateStatus(CTRL_CB4_ON, CLOSED);
			}	//	if(INVERTER.uStatus == SYS_INV_FAULT)
/*
#ifdef BYP_EVT_OPERATION
			else if(INVERTER.uStatus == SYS_INV_STOP && bSSW_CB2_Disable == FALSE
					&& GenBlock.SET_loadPowerMeterS < PRM_PCS[PM_OVERLOAD_LV1_KVA].iValue * PRM_PCS[PM_OVERLOAD_LV1_KVA].fIncDec * 1000)
#else
*/
			else if(INVERTER.uStatus == SYS_INV_STOP)
//#endif
			{
				//-180210 ?? on->off
				if(MC_GetStatus(STATUS_CB4))
					GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);

#if 1 //???
				if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
				{
					if(!MC_GetStatus(STATUS_CB4))
						MCB_UpdateCmd(M_CB4_SSW, CB_CMD_ON);
				}
				else
				{
					if(!MC_GetStatus(STATUS_CB4))
						MC_UpdateStatus(CTRL_CB4_ON, CLOSED);
				}
#endif
			}	// else if(INVERTER.uStatus == SYS_INV_STOP)
			/*	2016.06.10	Yang - STABLEEN	*/
		}	//	if (ODT_Update(GenBlock.odtGridConnectDelay, TRUE) == ODT_FINISH)
	}
	else
	{
		ODT_Initialize(GenBlock.odtGridConnectDelay);
		gd(8);
	}
#else
	if(INVERTER.uStatus == SYS_INV_FAULT)
	{
		GPIO_StaticSwitch(GPIO_STATIC_SW_ON);
		if(!MC_GetStatus(STATUS_CB4))
		{
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
				MCB_UpdateCmd(M_CB4_SSW, CB_CMD_ON);
			else
				MC_UpdateStatus(CTRL_CB4_ON, CLOSED);
		}
	}
	else
	{
		//200205 CB4 태엽이 감기지 않은 상태로, MCU 제어전원이 ON, 시스템 State는 STOP일 경우 SSW, CB4모두 OFF상태임. SSW를 붙여 준다. --> off 코일에 신호를 줘서 태엽을 감아 놓고 붙이도록 해야 함.
		if(MC_GetStatus(STATUS_CB4))
		{
			GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
		}
		else
		{
			GPIO_StaticSwitch(GPIO_STATIC_SW_ON); //+200205

			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
				MCB_UpdateCmd(M_CB4_SSW, CB_CMD_ON);
			else
				MC_UpdateStatus(CTRL_CB4_ON, CLOSED);


		}
	}
#endif
}
#endif //by JCNET

#pragma CODE_SECTION (SYS_FAN_Control, "dp_ctrl")
void SYS_FAN_Control(void)
{
    /* Fault Status */
    if(INVERTER.uStatus == SYS_INV_FAULT)
    {
//BY JCNET        MC_UpdateStatus(CTRL_SCFAN_CTRL, OPEN);
        //MC_UpdateStatus(CTRL_SSEFAN_CTRL, CLOSED);
    }
    else if(INVERTER.uStatus == SYS_INV_TEST_MODE)
    {

    }
    else
    {
        /* SCE PNL */
        if((EVE.bStatus == TRUE) || IVC.uStatus == CVC_ENABLE)
        {
//by JCNET            MC_UpdateStatus(CTRL_SCFAN_CTRL, CLOSED);
        }
        else
        {
 //by JCNET           MC_UpdateStatus(CTRL_SCFAN_CTRL, OPEN);
        }

        /* SSE PNL */
        if( GPIO_GetStaticSwitch() )
        {
            //MC_UpdateStatus(CTRL_SSEFAN_CTRL, CLOSED);
        }
        else
        {
            //MC_UpdateStatus(CTRL_SSEFAN_CTRL, OPEN);
        }
    }
}

