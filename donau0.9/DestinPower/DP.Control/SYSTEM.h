/*
 * SYSTEM.Logos.h
 *
 *  Created on: 2013. 8. 5.
 *      Author: destinPower
 */

#ifndef SYSTEM_LOGOS_H_
#define SYSTEM_LOGOS_H_

#include "LGS_Common.h"
#include "ODT.h"
#include "CTRL_GEN.h"
#include "SagEventHistory.h"
#include "EVE_DC_CHARGER.h"


#define SYSTEM_EXECUTION_PERIOD 0

typedef enum _SYS_STATE{
	SYS_INV_INIT				,
	SYS_INV_FAULT				,
	SYS_INV_STOP				,
#ifdef Fixed_State_Machine
	SYS_INV_SCR_ON				,
#endif
	SYS_INV_DC_CHARGE			,
#ifdef Fixed_State_Machine
	SYS_INV_EVE_DC_CHARGE		,
#endif
	SYS_INV_AC_GENERATE			,
	SYS_INV_START_SYNC			,
	SYS_INV_RUN					,
	SYS_INV_ISLANDING			,
	SYS_INV_RE_SYNC				,
	SYS_INV_BYP_EVT_OPERATION	,
	SYS_INV_TEST_MODE
}SYSTEM_STATE;


/* ON 컨맨드 후 DC Charge 전이 딜레이 */
#if DBUG_MODE == 3 || DBUG_MODE == 2
#define SYS_INV_ODT_DISCONNECT		5000	/* 5sec 대기 */
#else
#define SYS_INV_ODT_DISCONNECT		5000	/* msec 10-> 5 */
#endif

/* Grid 측 Sync Time 시간 */
#if DBUG_MODE == 2
#define SYS_INV_ODT_SYNC_TIME_OUT		3000 	// msec
#else
#ifndef STABLEEN
#define SYS_INV_ODT_SYNC_TIME_OUT		3000 	// msec
#else
#define SYS_INV_ODT_SYNC_TIME_OUT		5000 	// msec
#endif
#endif


/* 초충 고장 체크 시간 */
#define SYS_INV_ODT_CB1A_FAULT      3000

/* 초충 시간 */
#if DBUG_MODE == 3 /* HILL */ || DBUG_MODE == 2
#define SYS_INV_ODT_DCV_CHARGE      3000   /* 10 Sec */
#else
//by JCNET #define SYS_INV_ODT_DCV_CHARGE      5000	/* 초충 시간 10-> 11 Sec (신안500kW에서 1초 차이로 DC Link가 70%가 되지 않음)*/
#define SYS_INV_ODT_DCV_CHARGE      (Uns)15000    /* DONAU 15 seconds by JCNET*/
#endif

/* CB1A ON, CB1 ON 상태에서 유지 시간 */
#if DBUG_MODE == 3 /* HILL */
#define SYS_INV_ODT_BATTCB_ON			1000
#else

// 14.1.9 CB1과 CB1A가 동시에 붙도록 수정. CB1A OFF가 안되는 현상 때문.
#define SYS_INV_ODT_BATTCB_ON			5000 	/* msec */

#endif

/* CB2 OFF 고장 체크 시간 */
#define SYS_INV_ODT_CB2_OFF			5000	/* msec */

#define SYS_INV_ODT_RESYNC_BYP_OK   100 /* msec */

#define SYS_LINE_NUM				3

extern Bool bDcChargerOnOff;

typedef struct
{
	float prevInverterIqe;
	float genConnectCheckStartPower;
	float genConnectCurrentDidtLevel;
	Bool bPredictGenConnect;
}GeneratorConnectPredict;

typedef struct
{
//	float fFreq;
	float fPhaseVoltage;
	float fCurrent;
	float fPower;
} Measure;

// 계통 고장 - 5분간 재투입 방지 기능
typedef struct
{
	Bool bEnb;
	unsigned int uiProgressionSecond;
}GRID_RECONECT_PROTECT;


typedef struct
{
    Uns 	uPrevStatus;
    SYSTEM_STATE 	uStatus;

#if IIVC_ENB
	Bool	bRequestInvThetaInit;
	Bool	bInitInvOn;
#endif
	Bool 	bRun;

//    Rate	RATE;
    Measure	MEASURE[SYS_LINE_NUM];

    Odt		*odtDisconnect;
	Odt 	ODT_INV_DISCONNECT;
    Odt		*odtCb1AFault;
	Odt 	ODT_CB1A_FAULT;
	Odt 	*odtDcCharge;
	Odt		ODT_DC_CHARGE;
    Odt		*odtBattCbOn;
	Odt 	ODT_INV_BATTCB_ON;
    Odt		*odtCb2Off;
	Odt 	ODT_CB2_OFF;
	Odt 	*odtBypassDelay;
	Odt		ODT_BYPASS_DELAY;
	Odt		*odtResyncCb2Delay;
	Odt		ODT_RESYNC_CB2_DELAY;

	Odt		*odtIinvBuildup; /* AC 생성 시간 */
	Odt 	ODT_INV_IINV_BUILDUP;
    Odt		*odtIinvSync;
	Odt 	ODT_INV_SYNC;
    Odt		*odtIinvSyncTimeOut;
	Odt 	ODT_INV_SYNC_TIME_OUT;

	Odt		*odtMc1A;
	Odt		ODT_INV_MC_1A;

	Odt		*odtGI_SlaveDelay;
	Odt		ODT_SLAVE_DELAY;

	Odt		*odtBypNormal;
	Odt		ODT_BYP_NORMAL;

	Odt		*odtBypOverLoad;
	Odt		ODT_BYP_OVERLOAD;
	Odt		*odtBypOverLoadClear;
	Odt		ODT_BYP_OVERLOAD_CLEAR;

	Odt		*odtSCROverLoad;
	Odt		ODT_SCR_OVERLOAD;
	Odt		*odtSCROverLoadClear;
	Odt		ODT_SCR_OVERLOAD_CLEAR;

	Odt		*odtPMOverLoad;
	Odt		ODT_PM_OVERLOAD;
	Odt		*odtPMOverLoadClear;
	Odt		ODT_PM_OVERLOAD_CLEAR;

	Odt		*odtSCROnDelay;
	Odt		ODT_SCR_ON_DELAY;
	Odt		*odtSCROffDelay;
	Odt		ODT_SCR_OFF_DELAY;

	Odt		*odtCB2OnDelay;
	Odt		ODT_CB2_ON_DELAY;
	Odt		*odtCB2OffDelay;
	Odt		ODT_CB2_OFF_DELAY;

	Odt		*odtSagCompTime;
	Odt		ODT_SAG_COMP_TIME;

	Odt		*odtSSWCB2Disable;
	Odt		ODT_SSW_CB2_DISABLE;

	Odt		*odtSeamlessDisable;
	Odt		ODT_SEAMLESS_DISABLE;

	Odt		*odtEveCheck;
	Odt		ODT_EVE_CHECK;

	Odt 	*odtBypassVok;
	Odt		ODT_BYPASS_VOK;

	Odt		*odtHoldLoadP;
	Odt		ODT_HOLD_LOAD_P;

    Odt     *odtMC8Fault;
    Odt     ODT_MC8_FAULT;
    Odt     *odtMC9Fault;
    Odt     ODT_MC9_FAULT;

	float 	fTRRatio;

	Bool 	bDcChargeOK;
	GRID_RECONECT_PROTECT gridReconnect;

	/* 151105
	 * MC4가 ON 된 후 5초간은 TR측 Z 성분을 없애야 Droop제어가 안정적으로 동작하여 timer 추가.
	 * byp30 HILL test중에...
	 */
	UInt16 mc4On_timer_start;
	UInt16 mc4On_timer; /* MC4가 ON 된 후 Count 시작하여 5초 후 0 */
	UInt16 bRemove_Z_TR;

	UInt16 RUN_LOAD_OC_DELAY_CNT;

} Inverter;

typedef struct
{
    Bool bTestRun;
    Bool bInverter;
    Bool bStart;	// Start:DC Charge -> AC GEN, RUN
    				// Stop: Run -> DC Charge
    Bool bPwmOnOff;

    Bool bDerating; // OCI: current limit 50%
} Button;

typedef struct
{
//    Rate	RATE;
    Measure	MEASURE[SYS_LINE_NUM];
    Measure	MEASURE_BYP[SYS_LINE_NUM];
    Odt		*odtInputVoltNormalHold;
    float	fTotalPower;
	float	fVoltPositive[3];
} GridArg;



enum
{
	SYS_GRID_OV_LEVEL1 = 0,
	SYS_GRID_UV_LEVEL1,
	SYS_GRID_OF_LEVEL1,
	SYS_GRID_UF_LEVEL1,

	SYS_INV_OV_LEVEL1,
	SYS_INV_UV_LEVEL1,
	SYS_INV_OF_LEVEL1,
	SYS_INV_UF_LEVEL1,

	SYS_GRID_OV_LEVEL2,
	SYS_GRID_UV_LEVEL2,
	SYS_GRID_OF_LEVEL2,
	SYS_GRID_UF_LEVEL2

/*	SYS_INV_OV_LEVEL2,
	SYS_INV_UV_LEVLE2,
	SYS_INV_OF_LEVLE2,
	SYS_INV_UF_LEVEL2,*/
};


void SYS_UpdateParameter(void);
void SYS_Create(void);
void SYS_ClearCommand( void );
void SYS_SetTestRunCommand(Bool ON_OFF);
void SYS_SetInverterCommand(Bool ON_OFF);
void SYS_CheckNVSRAMRun( void );

Bool SYS_CheckInvCmdOn( void );
void SYS_UpdateStatus( void );
void SYS_AddInfoNode( void );
void SYS_VoltFreqMonitor(void);

void SYS_UpdateKpccKiccLi(void);
void SYS_UpdateLiViaParam(float fValue);
void SYS_UpdateLI();
void SYS_Off(void);
Bool SYS_Get20msUnitFotEvent(void);
void SYS_VoltFreqFaultClear();
void SYS_BattVoltFaultClear();
void SYS_BattUnderVoltFaultClear(); //13.3.3

void c_int25( void );

void SYS_UpdateLevel2ProtectionLevel(Uns uLevel, Bool bLevel2Enable);
void SYS_SetLevel2Protection(Bool bEnable);
void SYS_SetLevel2SeamlessProtection();

Bool SYS_ReConnectionCondition(void);
void SYS_SetGeneratorConnectPredict(void);
void Prevent_Immediate_Grid_Reconnect(void);
void SYS_Byp20_StopFault(void);
void SYS_FAN_Control(void);

extern Inverter INVERTER;
extern GridArg GRID_ARG;
extern Button  BUTTON;
extern UInt16 RecloserSwitchCmd;
extern float fBypassE[SYS_LINE_NUM];

#if DBUG_MODE == 2
#define GENERATOR_MC() GenBlock.genSw
/* FR타입에서는 CB3 상태 출력으로 사용 됨. MG모델에서는 사용되지 않도록 수정해야 함. */
#define GENERATOR_MC_ON() GenBlock.genSw = 1
#define GENERATOR_MC_OFF() GenBlock.genSw = 0
#else
	#if MC6_SIMUL_ON == 0
		#define GENERATOR_MC() MCCB_MIRROR.Status.BitVal.bMC6
		/* FR타입에서는 CB3 상태 출력으로 사용 됨. MG모델에서는 사용되지 않도록 수정해야 함. */
		#define GENERATOR_MC_ON() MC_UpdateStatus(CTRL_CB3_STATE, CLOSED)
		#define GENERATOR_MC_OFF() MC_UpdateStatus(CTRL_CB3_STATE, OPEN)
	#else
		#define GENERATOR_MC() GenBlock.genSw
		/* FR타입에서는 CB3 상태 출력으로 사용 됨. MG모델에서는 사용되지 않도록 수정해야 함. */
		#define GENERATOR_MC_ON() GenBlock.genSw = 1
		#define GENERATOR_MC_OFF() GenBlock.genSw = 0
	#endif
#endif

// STAUS_ACB3 STATUS -------------------------------------------------
#if DBUG_MODE == 2
#define GENERATOR_ACB3() GenBlock.bGenVCB_Status
#else
// HILL의 1번 MCU 에서는 ACB3번신호를 MC3에 신호를 주지만, 2번 MCU에는 주지 않으므로 CAN을 이용하여 1번이 받으면 2번도 수신한 것으로 인식하도록 한다.

#if HILL_ACB3_STATUS_FROM_CAN == 1
#define GENERATOR_ACB3() MCCB_MIRROR.Status.BitVal.bDoor
#else
#define GENERATOR_ACB3() MCCB_MIRROR.Status.BitVal.bMC6
#endif

#endif
//---------------------------------------------------------------------


// CANADA M1-3 SW, F1 CB STATUS ---------------------------------------
#if DBUG_MODE == 2
#define CANADA_SW() GenBlock.genSw
#define CANADA_CB() GenBlock.bypSw
#else
#ifndef STABLEEN
#define CANADA_SW() MCCB_MIRROR.Status.BitVal.bADIB1
#define CANADA_CB() MCCB_MIRROR.Status.BitVal.bADIB2
#else
#define CANADA_SW() MCCB_MIRROR.Status.BitVal.bADIB1
#define CANADA_CB() MCCB_MIRROR.Status.BitVal.bCB1E
#endif
#endif /* SYSTEM_LOGOS_H_ */
#define CANADA_SW_OR_CB_OFF() (!CANADA_SW() || !CANADA_CB())
#define CANADA_SW_AND_CB_ON() (CANADA_SW() && CANADA_CB())
#endif

#define BYP_MODE_CANADA() ( PRM_PCS[BYP_MODE].iValue == 100 || PRM_PCS[BYP_MODE].iValue == 101 )
//---------------------------------------------------------------------


// STABLEEN
extern Bool GetDcChargerOnOff(void);

extern int 	InverterStateShadow;
