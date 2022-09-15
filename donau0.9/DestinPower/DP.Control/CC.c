/*
 * CC.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */
//#define JCNET_EMUL
//#define SWAP_BYP_OUTEA
#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "CC.h"
#include "FAULT.h"
#include "MVP.h"
#include "trace.h"
#include "PEAK.h"
#include "RMS.h"
//-#include "FREQ.h"
#include "RAMP.h"
#include "PHASECHECK.h"
#include "FastRms.h"
#include "pwm.h"
#include "epwm.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "VariablePeriod.h"
#include "EADC.h"
#include "XINTF_ADC.h"
#include "SYSTEM.h"

//-#include "CosphiControl.h"
#include "DBUG.h"
#include "Modbus232.h"
#include "DSP28x_Project.h"
#include "MO.h"
#include "CTRL_INV_PWM_GENERATE.h"
#include "GPIO.h"
#include "CTRL_FILTER.h"
#include "MCCB.h"
#include "TraceFlash.h"
#include "CTRL_GEN.h"
//#include "SagEventHistory.h"

#define FRT_MODE_VOTE_LIMIT	(100)
#define FRT_MODE_VOTE_THRESHOLD	(3)

#if DOUBLE_CONTROL == 1
#define CC_OC_COUNT_MAX 3 // 5kHz 에서 400us, 4kHz에서 375us
#define CC_LOAD_OC_COUNT_MAX 500
#define CC_CCI_OC_COUNT 3
#define CC_OV_UV_COUNT 10
#define CC_DC_ABNORMAL_COUNT 100
#else
#define CC_OC_COUNT_MAX 2
#define CC_CCI_OC_COUNT 2
#define CC_OV_UV_COUNT 5
#endif

#define	CC_IREF_RAMP_US			(20000)
#define	CC_IdREF_RAMP_US		(20000)
float aaa = 0;
// 연속적으로 OC가 검출되면 고장발생
// 인자 설명
//     a ==> float : 순시전류
//     b ==> float : Max Level
//     c ==> float : Min Level
#define CkeckOC(a, b, c) \
		if ( a >= b || a <= c ) \
		++(a##_OC_Count); \
		else \
		a##_OC_Count = 0;

#define CCI_NoGating()	\
		(INVCTRL.CCI_Status == CCI_NOGATING || INVCTRL.CCI_2ndOC_Status == CCI_NOGATING)

#define LIB_SoftStart_v2(srcRef, dstRef,delta ) \
		(dstRef - srcRef >= delta ) ? srcRef + delta:	\
				(dstRef - srcRef <= -delta ) ? srcRef - delta : dstRef

float CC_fWe = 0.; 			// 2PI * Grid Frequency
float CC_fWeInverse = 0.; 	// fWe 역수
float CC_fDTheta = 0.;		// fWe * CC_tsCC (Delta 각??)
float CC_tsCC = 0.;			// 제어주기( 5KHz = 200us ) * double control = tsCC / 2
float CC_tsSample = 0.;		// Sampling time ( double control = tsCC / 2 )
float CC_tsSampleInv = 0.;	// 1 / Sampling time

float ScaleGating = 0.;		// Periperal Clock / 2 ( PWM 값 적용 전 scale )
//float TccIVdc = 0.;			// CC_tsCC / DCLinkV2
float timer_sec_r = 0; // 1234
float timer_sec_s = 0; // 1234
float timer_sec_t = 0; // 1234

float timer_sec_shadow = 0; // 1234

Bool CC_bVoltageFilterShadowing = FALSE;
Bool CC_bFilterShadowing = FALSE;
Bool CC_bFsRmsFilterShadowing = FALSE;
Bool CC_bFreqFilterShadowing = FALSE;
Bool CC_bPowerControlFilterShadowing = FALSE;


//by JCNET pButton 추가
extern Button *pButton;

// CVC => IVC Inverter DC Voltage Control
#if DBUG_MODE == 0
InvVoltCtrl IVC;
#else
InvVoltCtrl IVC;
#endif
static PICon IVC_PI;
static PIArg IVC_PIArg;
static Ramp IVC_RAMP_OUTPUT, IVC_RAMP_INPUT;

// ICC: DC  Inverter Current Control
#if DBUG_MODE == 0
#else
static
#endif
InvCurrentCtrl ICC;
//ConCurrentCtrl CCC;// Converter Current Control by JCNET

static PICon ICC_PI;
static PIArg ICC_PIArg;
static Ramp ICC_RAMP_OUTPUT;

// PCC: Grid Power Control(GRID)
#if DBUG_MODE == 0
//static InvPowerCtrl PCC;
InvPowerCtrl PCC;
#else
InvPowerCtrl PCC;
#endif
static PICon PCC_PI;
static PIArg PCC_PIArg;
static PICon PCC_PI_Q;
static PIArg PCC_PIArg_Q;
static Ramp PCC_RAMP_OUTPUT;
static Ramp PCC_RAMP_OUTPUT_Q;

// Frequency 계산
ThetaDetect GRIDTHETA, CONTHETA, GRIDTHETA_AI/*for AntiIslanding*/, GRID_BYP_THETA, GEN_THETA;
/*by JCNET */
ThetaDetect SCCTHETA;
static  PICon SCCTHETA_PI;
static PIArg  SCCTHETA_PIArg;
/**/
static Gain PLL_K;
static PICon GRIDTHETA_PI, CONTHETA_PI, GRIDTHETA_PI_AI, GRID_BYP_THETA_PI, GEN_THETA_PI;
static PIArg GRIDTHETA_PIArg, CONTHETA_PIArg, GRIDTHETA_PIArg_AI, GRID_BYP_THETA_PIArg_AI, GEN_THETA_PIArg;

BattCtrl BATCTRL;
InverterCtrl INVCTRL;
ExternalCtrl EXCTRL;
CtrlOption OPTION;

SagCompenCtrl SCCTRL; //by JCNET

#if IIVC_ENB
InitInvVoltCtrl InitInvVControl_d, InitInvVControl_q;
InitInverter InitINV;
#endif

static Ramp RAMPIdeRef;
static Ramp RAMPIdeQV;
static Ramp RAMPIdePPN;
static Ramp RAMPIqeRef;
static Ramp RAMPIqeRefDelay;

LoadInfo loadInfo;
BC_STATE BCState;

#if CC_EXECUTION_TIME
UInt32 CC_uExecutionTime = 0;
#endif

#if CC_EXECUTION_PERIOD
UInt32 CC_uExecutionTime2 = 0;
#endif

AC_Panel ACP;
Controller CTRL;

public float ZeroCrossingTimeDetect(ThetaDetect *this, int phase, Bool* di)
{
#if ZERO_COROSSING_METHOD == 0
	if (this->fRadian < 0.0)
	{
		return ((-this->fRadian / PI) * 0.0083333333333);
	}
	else
	{
		return (1 - (this->fRadian / PI)) * 0.0083333333333;
	}
#elif ZERO_COROSSING_METHOD == 1
	float radian = 0.0;

	if( phase == 1 ) // S
		radian = this->fRadian + 4.18879 ; // +240 degree
	else if( phase == 2) // T
		radian = this->fRadian + 2.0944 ;  // +120 degree
	else
		radian = this->fRadian;

	if (radian > PI)
	{
		radian += -TWO_PI;
	}
	else if (this->fRadian < -PI)
	{
		radian += TWO_PI;
	}

	if (radian < 0.0)
	{
		*di = 0;
		return ((-radian / PI) * (PI / GRID_BYP_THETA.fFreqW) );
	}
	else
	{
		*di = 1;
		return (1 - (radian / PI)) * (PI / GRID_BYP_THETA.fFreqW);
	}

#endif
}

/*
 * call : cint11()
 * when : CVC DISABLE 상태
 */
#pragma CODE_SECTION (ConverterInitialize, "dp_ctrl")
static inline void ConverterInitialize(void)
{
	IVC.fRef = 0;
	IVC.fRef_command = 0;
	IVC.bRAMPInputInitEnb = TRUE;
	PICON_Initialize(IVC.pPI);
	IVC.fPIOut = 0;
	RAMP_Initialize(IVC.pRAMPOutput);

	//13.3.23
	ICC.fRef = 0;
	PICON_Initialize(ICC.pPI);
	ICC.fPIOut = 0;
	RAMP_Initialize(ICC.pRAMPOutput);

	//130620
	PCC.fRefP = 0;
	PCC.fRefQ = 0;
	PICON_Initialize(PCC.pPI);
	PCC.fPIOut = 0;
	RAMP_Initialize(PCC.pRAMPOutput);

	PICON_Initialize(PCC.pPI_Q);
	PCC.fPIOut_Q = 0;
	RAMP_Initialize(PCC.pRAMPOutput_Q);

	INVCTRL.IqeRef = 0;
	INVCTRL.IdeRef = 0;
	INVCTRL.VdeIntegOut = 0.;
	INVCTRL.VqeIntegOut = 0.;
	INVCTRL.VdePIOut = 0.;
	INVCTRL.VqePIOut = 0.;
	INVCTRL.VdeRef = 0.;
	INVCTRL.VqeRef = 0.;
	INVCTRL.IdeErr = 0.;
	INVCTRL.IqeErr = 0.;
	INVCTRL.VdeEmfComp = 0.;
	INVCTRL.VqeEmfComp = 0.;
	INVCTRL.VdsRef = 0.;
	INVCTRL.VqsRef = 0.;
	INVCTRL.VaRef = 0.;
	INVCTRL.VbRef = 0.;
	INVCTRL.VcRef = 0.;
	INVCTRL.VanRef = 0.;
	INVCTRL.VbnRef = 0.;
	INVCTRL.VcnRef = 0.;
	INVCTRL.VsnRef = 0.;
	INVCTRL.EPeakInverse = 0;

	RAMP_Initialize(INVCTRL.pRampIdeRef);
	RAMP_Initialize(INVCTRL.pRampIdeQV);
	RAMP_Initialize(INVCTRL.pRampIdePPN);
	RAMP_Initialize(INVCTRL.pRampIqeRef);
	RAMP_Initialize(INVCTRL.pRampIqeRefDelay);
}

#if IIVC_ENB
/*
 * call: IIVCCreate()
 * call: InitInvInitialize()
 */

static void IIVC_Initialize( InitInvVoltCtrl *this )
{
	this->fRef = 0.;

	PICON_Initialize(this->pPI);
	this->fPIOut = 0.;
	this->fOut = 0.;

	RAMP_Initialize(this->pRAMPInput);
	RAMP_Initialize(this->pRAMPOutput);
}

/*
 * call: cint11()
 * when: CVC DISABLE 상태, Grid SYNC 모드인 경우
 */

static void InitInvInitialize( void )
{
	IIVC_Initialize(&InitInvVControl_d);
	IIVC_Initialize(&InitInvVControl_q);

	InitINV.VaRef = 0.;
	InitINV.VbRef = 0.;
	InitINV.VcRef = 0.;
	InitINV.VdsRef = 0.;
	InitINV.VqsRef = 0.;
	InitINV.VdeRef = 0.;
	InitINV.VqeRef = 0.;
}

#endif

/*
 * call: CC_Create()
 * arg this: CONTHETA, GRIDTHETA
 */
static void ThetaDetectCreate(ThetaDetect *this, PICon *pPI, PIArg *pPIArg,
		Gain *pK)
{
	this->pARG = PIArg_(pPIArg, pK);

	this->pARG->fCompen = 0.;
	this->pPI = PICon_(pPI);

	this->fFbk = 0.;
	this->fFreqW = 0.;
	this->fFreq = 0.;
	this->fRadian = 0.;
	this->fPIOut = 0.;
	this->bRMSChangeEnable = FALSE;
}

/*
 * comm: Freq를 계산하기 위함
 * call: cint11()
 * when: start block ( Freq PLL )
 * arg this: GRIDTHETA, CONTHETA
 */
//by JCNET static
void ThetaDetectRun(ThetaDetect *this, float EdeFlt)
{
	this->pfRadian = this->fRadian;
	this->pARG->fErr = -EdeFlt;
	this->pARG->fFbk = EdeFlt;
	this->fPIOut = PICON_Run(this->pPI, this->pARG);

	this->fFreqW = this->fPIOut + CC_fWe;

	this->fRadian += this->fFreqW * CC_tsSample;
	if (this->fRadian > PI)
	{
		this->fRadian += -TWO_PI;
		this->bRMSChangeEnable = TRUE;
	}
	else if (this->fRadian < -PI)
	{
		this->fRadian += TWO_PI;
	}

	this->fFreq = this->fFreqW * INV_TWO_PI;

}

static void ThetaDetectRun_Omega(ThetaDetect *this, float Ede)
{
	this->pARG->fErr = -Ede;
	this->pARG->fFbk = Ede;
	this->fPIOut = PICON_Run(this->pPI, this->pARG);

	this->fFreqW = this->fPIOut + CC_fWe;
}

/*
 * call: CC_Create()
 * arg this: CONTHETA, GRIDTHETA
 */
static inline void CC_SetThetaDetectLimit(ThetaDetect *this, float Min,
		float Max)
{
	this->pARG->fMin = Min;
	this->pARG->fMax = Max;
}

/*
 * comm: Grid와 Inverter의 Freq를 리턴
 * call: ACI_ResponseGRIDINFO, ACI_ResponseINVInfo
 *
 */
float CC_GetFreq(ThetaDetect *this)
{
	return this->fFreq;
}

float CC_GetActualPowerP(void)
{
	return EXCTRL.fltAcPowerP.fOut;
}
float CC_GetActualPowerQ(void)
{
	return EXCTRL.fltAcPowerQ.fOut;
}

float CC_GetActualPowerP_2nd(void)
{
	return EXCTRL.fltAcPowerP_2nd.fOut;
}
float CC_GetActualPowerQ_2nd(void)
{
	return EXCTRL.fltAcPowerQ_2nd.fOut;
}

Bool bTintOneshot_r = 0;
Bool bTintOneshot_s = 0;
Bool bTintOneshot_t = 0;

float CC_GetPowerFactor(void)
{
	return INVCTRL.fPowerFactor;
}

void CC_InitLoadInfo()
{
	loadInfo.fIQE_PEAK = (((float) PRM_PCS[INV_CAPACITY].iValue * 1000.)
			/ (INV_RATED_VOLTAGE * SQRT3)) * SQRT2;
	loadInfo.fIQE_PEAK_INVERSE = 1 / loadInfo.fIQE_PEAK;
}
//float LoadP;
float CC_GetCurrentLoadI()
{
	//LoadP = fabs((INVCTRL.fltIqe.fOut * loadInfo.fIQE_PEAK_INVERSE) * 100.);
	// Percent
	return fabs((INVCTRL.fltIqe.fOut * loadInfo.fIQE_PEAK_INVERSE) * 100.);

	//LoadP = (INVCTRL.Ia / INVERTER.RATE.fCurrent) * 100.;
}

void CC_PCC_Gating()
{
	PCC.uState = GPC_GATING;
}
void CC_PCC_IdleTimeInit()
{
	PCC.uIdleTime = 0;
}
void CC_PCC_DisablePWM()
{
	/*
	 * Idle time(BATT_PCC_PWM_DISABLE_SEC)sec 동안 ref가 0로 유지되면 CVC_DISABLE
	 * ref가 INV_CAPACITY의 5% 이상이 되면 CVC DISALBE에서 CVC ENABLE로 변경
	 *
	 * CC_MODE_CP AND STANDBY State
	 */

	//if ( PCC.fRefP == 0 && PCC.fRefQ == 0)
	if ( PCC.fRefP == 0 && PCC.fRefQ == 0 && bByp_Evt_Operating == FALSE)
	{
		if (PCC.uIdleTime >= PARAM_VAL(BATT_PCC_PWM_DISABLE_SEC))
		{
			// CVC Disable
			PCC.uState = GPC_NOGATING;
			PCC.uIdleTime = 0;
		}
	}
#ifdef BYP_EVT_OPERATION
	else if( bByp_Evt_Operating == TRUE && PCC.fRefP == 0 && PCC.fRefQ == 0 )
	{
		// CVC Disable
		PCC.uState = GPC_NOGATING;
		PCC.uIdleTime = 0;
	}
#endif
	else
	{
		PCC.uState = GPC_GATING;
		PCC.uIdleTime = 0;
	}
}

void CC_BCTransIdle()
{
	BCState.powerFlow = PFLOW_STANDBY;

	if (BCState.srcOder == SRC_REMOTE)
	{
		PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF)= 0;
		PARAM_RAW_VAL(BATT_REMOTE_PCC_Q_REF) = 0;
		PARAM_RAW_VAL(BATT_DCHG_PWR)= 0; // 58 - Power Ref
		PARAM_RAW_VAL(BATT_VF_CHECK_TIME) = 0; //61 - Q ref
		PARAM_RAW_VAL(BATT_REMOTE_POWER_FLOW) = PFLOW_STANDBY;
	}
	else
	{
		// Local
		PARAM_RAW_VAL(BATT_LOCAL_POWER_FLOW) = PFLOW_STANDBY;
	}
}
void CC_BCRemotePowerFlowCheck()
{
	float fBattV;
	//주의! Idle에서 벗어날 경우 반드시 제어기 초기화!. 에러가 쌓임.
	if (BCState.ccMode == CC_MODE_CP)
	{
		if (PARAM_VAL(BATT_REMOTE_PCC_P_REF) < 0)
		{
			BCState.powerFlow = PFLOW_CHARGE;
		}
		else if (PARAM_VAL(BATT_REMOTE_PCC_P_REF) > 0)
		{
			BCState.powerFlow = PFLOW_DISCHARGE;
		}
		else
		{
			// Idle 상황에서 무효전력이 나가도록 Q 레퍼런스를 0 로 만들지 않음
			PARAM_RAW_VAL(BATT_REMOTE_POWER_FLOW) = PFLOW_STANDBY;
			BCState.powerFlow = PFLOW_STANDBY;
		}

		fBattV = BATCTRL.fltDCBattV_2nd.fOut;

		if (BCState.powerFlow == PFLOW_CHARGE)
		{
			if ( PARAM_VAL(BATT_V_RANGE_MAX) <= fBattV )
			{
				//(1) VDC 한계값 검사
				CC_BCTransIdle();
			}
		}
		else if (BCState.powerFlow == PFLOW_DISCHARGE)
		{
			if ( PARAM_VAL(BATT_V_RANGE_MIN) >= fBattV )
			{
				//(1) VDC 한계값 검사
				CC_BCTransIdle();
			}
		}
	}
	else
		CC_BCTransIdle();

	if (BCState.powerFlow != BCState.prevPowerFlow)
	{
		if (BCState.prevPowerFlow == PFLOW_STANDBY)
			BCState.bNeedInit = TRUE;

		BCState.prevPowerFlow = BCState.powerFlow;

		//if( BCState.ccMode == CC_MODE_CCCV || BCState.ccMode == CC_MODE_CV)
		//CC_IVC_OutRampInit();
	}
}

void CC_BCLocalPowerFlowCheck()
{
	float fBattV;

	switch (BCState.ccMode)
	{
	case CC_MODE_CP:
	case CC_MODE_CC:
		fBattV = BATCTRL.fltDCBattV_2nd.fOut;

		if (BCState.powerFlow == PFLOW_CHARGE)
		{
			if ( fBattV >= PARAM_VAL(BATT_V_RANGE_MAX) )
			{
				//(1) VDC 한계값 검사
				CC_BCTransIdle();
			}
		}
		else if (BCState.powerFlow == PFLOW_DISCHARGE)
		{
			// 140305 CC 모드에서는 충전만 가능케 함.
			if (BCState.ccMode == CC_MODE_CC)
				CC_BCTransIdle();
			else if (fBattV <= PARAM_VAL(BATT_V_RANGE_MIN))
			{
				//(1) VDC 한계값 검사
				CC_BCTransIdle();
			}
		}
		break;
	case CC_MODE_CCCV:
	case CC_MODE_CV:
		// 목표 전압에 도달 하면, Idle
		if (BCState.powerFlow == PFLOW_CHARGE)
		{
			if (PARAM_VAL(BATT_LOCAL_IVC_V_REF_CHG)	<= BATCTRL.fltDCBattV_2nd.fOut)
				CC_BCTransIdle();
		}
		else if (BCState.powerFlow == PFLOW_DISCHARGE)
		{
			// 140305 CC-CV 모드에서는 충전만 가능케 함.
			if (BCState.ccMode == CC_MODE_CCCV) // 150123 fixed(before:CC_MODE_CC)
				CC_BCTransIdle();
			else if (PARAM_VAL(BATT_LOCAL_IVC_V_REF_DCHG) >= BATCTRL.fltDCBattV_2nd.fOut)
				CC_BCTransIdle();
		}

		break;
	default:
		CC_BCTransIdle();
		break;
	}

	if (BCState.powerFlow != BCState.prevPowerFlow)
	{
		//-if( BCState.prevPowerFlow == PFLOW_STANDBY )
		BCState.bNeedInit = TRUE;

		BCState.prevPowerFlow = BCState.powerFlow;

		//if( BCState.ccMode == CC_MODE_CCCV || BCState.ccMode == CC_MODE_CV)
		//CC_IVC_OutRampInit();
	}

}

// BC 모드와 Flow 파라미터가 변경 되었을 경우.
void CC_BCStateUpdate()
{
	if (PARAM_VAL(BATT_ORDER_SOURCE) == SRC_REMOTE)
	{
		// 리모트의 MODE 업데이트
		BCState.srcOder = SRC_REMOTE;

#if 0 // Reserved
		if( PARAM_RAW_VAL(BATT_REMOTE_CONTROL_MODE) == 0 )
		BCState.ccMode = CC_MODE_CP;
		else if ( PARAM_RAW_VAL(BATT_REMOTE_CONTROL_MODE) > CC_MODE_CV )
		BCState.ccMode = CC_MODE_CP;
		else
		BCState.ccMode = PARAM_RAW_VAL(BATT_REMOTE_CONTROL_MODE);
#else
		BCState.ccMode = CC_MODE_CP;
#endif

		CC_BCRemotePowerFlowCheck();

	}
	else // ( PARAM_VAL(BATT_ORDER_SOURCE) == SRC_LOCAL )
	{
		// 로컬의 MODE와 Power Flow 업데이트
		BCState.srcOder = SRC_LOCAL;

		if (PARAM_RAW_VAL(BATT_LOCAL_CONTROL_MODE)== 0 )
			BCState.ccMode = CC_MODE_CV;
		else if ( PARAM_RAW_VAL(BATT_LOCAL_CONTROL_MODE) > CC_MODE_CV )
			BCState.ccMode = CC_MODE_CV;
		else
			BCState.ccMode = (CC_MODE)PARAM_RAW_VAL(BATT_LOCAL_CONTROL_MODE);

		if (PARAM_VAL(BATT_LOCAL_POWER_FLOW) > PFLOW_DISCHARGE)
			BCState.powerFlow = PFLOW_STANDBY;
		else
			BCState.powerFlow = (POWER_FLOW) PARAM_VAL(BATT_LOCAL_POWER_FLOW);

		CC_BCLocalPowerFlowCheck();
	}
}

#if 0
CC_MODE CC_GetCCMode()
{
	if( BCState.srcOder == SRC_REMOTE )
	{
		if(PARAM_RAW_VAL(BATT_REMOTE_CONTROL_MODE) == 0 || (PARAM_RAW_VAL(BATT_REMOTE_CONTROL_MODE) > CC_MODE_CV) )
		return CC_MODE_CP;

		return (CC_MODE)PARAM_RAW_VAL(BATT_REMOTE_CONTROL_MODE);
	}
	else
	{
		if(PARAM_RAW_VAL(BATT_LOCAL_CONTROL_MODE) == 0 || (PARAM_RAW_VAL(BATT_LOCAL_CONTROL_MODE) > CC_MODE_CV) )
		return CC_MODE_CV;

		return (CC_MODE)PARAM_RAW_VAL(BATT_LOCAL_CONTROL_MODE); // Default
	}
}
#endif

#pragma CODE_SECTION (CC_LOCAL_GetReference, "dp_ctrl")
float CC_LOCAL_GetReference()
{
	float ref;

	// 충전
	if (BCState.powerFlow == PFLOW_CHARGE)
	{
		switch (BCState.ccMode)
		{
		case CC_MODE_CP:
			ref = PARAM_VAL(BATT_LOCAL_PCC_P_REF_CHG);
			// 레퍼런스 충전은 -
			ref *= -1;
			break;
		case CC_MODE_CCCV:
			ref = PARAM_VAL(BATT_LOCAL_IVC_V_REF_CHG);
			break;
		case CC_MODE_CC:
			ref = PARAM_VAL(BATT_LOCAL_ICC_I_REF_CHG);
			break;
		case CC_MODE_CV:
			ref = PARAM_VAL(BATT_LOCAL_IVC_V_REF_CHG);
			break;
		default:		// Default CV 모드
			ref = PARAM_VAL(BATT_LOCAL_IVC_V_REF_CHG);
			break;
		}

	}
	// 방전
	else if (BCState.powerFlow == PFLOW_DISCHARGE)
	{
		switch (PARAM_RAW_VAL(BATT_LOCAL_CONTROL_MODE))
		{
			case CC_MODE_CP:
			ref = PARAM_VAL(BATT_LOCAL_PCC_P_REF_DCHG);
			break;
			case CC_MODE_CCCV:
			//-140305 충전만 가능하다. ref = PARAM_VAL(BATT_LOCAL_IVC_V_REF_DCHG);
			ref = BATCTRL.DCLinkV;
			break;
			case CC_MODE_CC:
			//-140305 충전만 가능하다. ref = PARAM_VAL(BATT_LOCAL_ICC_I_REF_DCHG) * -1;
			ref = 0;
			break;
			case CC_MODE_CV:
			ref = PARAM_VAL(BATT_LOCAL_IVC_V_REF_DCHG);
			break;
			default:// Default CV 모드
			ref = PARAM_VAL(BATT_LOCAL_IVC_V_REF_DCHG);
			break;
		}
	}
	// Stanby
	else
	{
		// STANDBY = IQE REF 0
		switch( PARAM_RAW_VAL(BATT_LOCAL_CONTROL_MODE))
		{
			case CC_MODE_CP:
			ref = 0;
			break;
			case CC_MODE_CCCV:
			ref = BATCTRL.DCLinkV;
			break;
			case CC_MODE_CC:
			ref = 0;
			break;
			case CC_MODE_CV:
			ref = BATCTRL.DCLinkV;
			break;
			default:		// Default CV 모드
			ref = BATCTRL.DCLinkV;
			break;
		}
	}
	return ref;
}

#pragma CODE_SECTION (CC_REMOTE_GetReference, "dp_ctrl")
float CC_REMOTE_GetReference()
{
#if 0
	float ref;
	ref = PARAM_VAL(BATT_REMOTE_PCC_P_REF);
	switch( PARAM_RAW_VAL(BATT_REMOTE_CONTROL_MODE))
	{
		case CC_MODE_CP:
		ref = PARAM_VAL(BATT_REMOTE_PCC_P_REF);
		break;
		case CC_MODE_CCCV:
		ref = BATCTRL.DCLinkV;
		break;
		case CC_MODE_CC:
		ref = 0;
		break;
		case CC_MODE_CV:
		ref = BATCTRL.DCLinkV;
		break;
		default:// Default CP 모드
		ref = PARAM_VAL(BATT_REMOTE_PCC_P_REF);
		break;
	}
	return ref;
#endif
	return PARAM_VAL(BATT_REMOTE_PCC_P_REF);
}
float CC_GetReference()
{
	if (BCState.srcOder == SRC_LOCAL)
	{
		return CC_LOCAL_GetReference();
	}
	else if (BCState.srcOder == SRC_REMOTE)
	{
		return CC_REMOTE_GetReference();
	}

	return CC_LOCAL_GetReference();
}

Bool CC_GetSoftLevel()
{
	if (BCState.ccMode == CC_MODE_CP)
	{
		return PCC.bTrig_softLevel;
	}
	else
	{
		return FALSE;
	}

}

void CC_IVC_OutRampInit()
{
	IVC.bRAMPInputInitEnb = ON;
}

float anti_a, anti_b;
float anti_omega_err;

volatile float a, b, c, d; 				//-	register float a,b;
volatile float Ta, Tb, Tc, Tzero; //Illegal operand combination point : if Ta~Tc == volatile is OK else is NOT OK!
volatile float DCLinkV2;
unsigned int CallCounter1Sec = 0;
unsigned int iCCTemp = 0;
volatile float y, z;

volatile iir1st *pIIR1;
volatile PICon *pPICon;
volatile PIArg *pPIArg;

/*
 *  최종 출력 ICC.fPIOut
 *  충전 전용.
 */
#pragma CODE_SECTION (CC_Mode_CC, "dp_ctrl")
static void CC_Mode_CC()
{
	volatile float a;

	ICC.fRefCommand = CC_GetReference();

	// I Reference Limit Level
	a = (PARAM_VAL(INV_CAPACITY) * 1000) / PARAM_VAL(BATT_V_RANGE_MAX); // 전류

	// 출력 가능 최대 전류 a 보다 레퍼런스 전류가 크다면
	if (a < PARAM_VAL(BATT_LOCAL_ICC_I_REF_CHG))
		ICC.fRefCommand = a;

	PRM_PCS[BATT_4].iValue = (int) ICC.fRefCommand;
#if 1
	/*[ PI Controller Run - Current Controller](New)**************************************/
	a = ACP.INV.RATE.Iph_pk * INVCTRL.fCurrentLimit * 0.01;

	// LIMIT
	if (BCState.powerFlow == PFLOW_CHARGE)
	{

		ICC.pARG->fMax = fabs(a);
		ICC.pARG->fMin = 0;
	}
	else
	{
		// CC는 충전만 가능하고, Standby 모드에서는 IQE 레퍼런스를 강제로 0으로 만들기로 하였음.
		ICC.pARG->fMax = 0;
		ICC.pARG->fMin = 0;
	}

	ICC.pARG->fMax = a;
	ICC.pARG->fMin = -(a);

	ICC.fRef = ICC.fRefCommand; // CHECK RAMP 없음.
	ICC.pARG->fErr = ICC.fRef - BATCTRL.fltI2.fOut;
	ICC.pARG->fFbk = BATCTRL.fltI2.fOut;

	PICON_RunM(ICC.pPI, ICC.pARG, ICC.fPIOut);
#endif
}

// 최종 출력 IVC.fPIOut
//by JCNET
// 정류기 DC Link 전압제어기
static void CC_Mode_CV()
{
	volatile float a;

	//-IVC.fRef_command = (PARAM_VAL(BATT_LOCAL_IVC_V_REF));
	IVC.fRef_command = CC_GetReference();

	if (IVC.fRef_command > PARAM_VAL(BATT_V_RANGE_MAX))
		IVC.fRef_command = PARAM_VAL(BATT_V_RANGE_MAX);
	else if (IVC.fRef_command < PARAM_VAL(BATT_V_RANGE_MIN))
		IVC.fRef_command = PARAM_VAL(BATT_V_RANGE_MIN);

	if (IVC.bRAMPInputInitEnb == ON )
	{
		RAMP_SetInitOut(IVC.pRAMPInput, BATCTRL.DCLinkV); //Ramp 초기시작지점 설정
		IVC.bRAMPInputInitEnb = OFF;
	}

	PRM_PCS[BATT_4].iValue = (int) IVC.fRef_command;

	/*
	 * =================================================================================
	 * DC Voltage Controller BLOCK (0)
	 */
	IVC.fRef = RAMP_Change(IVC.pRAMPInput, IVC.fRef_command);
	IVC.pARG->fErr = IVC.fRef - BATCTRL.DCLinkV;
	IVC.pARG->fFbk = BATCTRL.DCLinkV;

	// Iqe는 Peak값이고 사용자가 입력하는 정보는 RMS값이므로 변환 필요
	a = ACP.INV.RATE.Iph_pk * INVCTRL.fCurrentLimit * 0.01;

// 150803
#if CV_I_LMT_USE == 1
	if( a > CV_I_LMT_VALUE )
		a = CV_I_LMT_VALUE;
#endif

	IVC.pARG->fMax = a;  // 충전
	IVC.pARG->fMin = -a; // 방전


	// -------------------------------------------------------------------------------*/
	if (PRM_PCS[CTRL_POWER_COMPEN].iValue)
		IVC.pARG->fCompen = BATCTRL.fPowerFilterd_1st * INVCTRL.EPeakInverse
				* 2. * INV_3;
	else
		IVC.pARG->fCompen = 0.;

	/*[ PI Controller Run - Voltage Controller](3)**************************************/
	PICON_RunM(IVC.pPI, IVC.pARG, IVC.fPIOut);
//++JCNET
    IVC.iqe_ref = RAMP_Change(IVC.pRAMPOutput, IVC.fPIOut);
//--
}
#if 0
//++JCNET
static void CC_Mode_DQ_current_control()
{
    volatile float a;
    float Lf = 100.0; // Read from system parameter ??
    CCC.ide_ref = ACP.INV.RATE.Iph_pk * INVCTRL.fCurrentLimit * 0.01;
    CCC.iqe_ref = IVC.iqe_ref;

    CCC.fRefCommand_D = CCC.ide_ref;
    CCC.fRefCommand_Q = CCC.iqe_ref;

    a = ACP.INV.RATE.Iph_pk * INVCTRL.fCurrentLimit * 0.01;

    CCC.pARG_D->fMax = a;
    CCC.pARG_D->fMin = -(a);
    CCC.fRef_D = CCC.fRefCommand_D; // CHECK RAMP 없음.
    CCC.pARG_D->fErr = CCC.fRef_D - INVCTRL.Ide;
    CCC.pARG_D->fFbk = INVCTRL.Ide;

    PICON_RunM(CCC.pPI_D, CCC.pARG_D, CCC.fPI_DOut_D);

    CCC.pARG_Q->fMax = a;
    CCC.pARG_Q->fMin = -(a);
    CCC.fRef_Q = CCC.fRefCommand_Q; // CHECK RAMP 없음.
    CCC.pARG_Q->fErr = CCC.fRef_Q - INVCTRL.Iqe;
    CCC.pARG_Q->fFbk = INVCTRL.Iqe;

    PICON_RunM(CCC.pPI_Q, CCC.pARG_Q, CCC.fPI_DOut_Q);

    CCC.vde_ref = 0                   - CCC.fPI_DOut_D + Lf * TWO_PI * PARAM_VAL(GRID_RATED_FREQ) * CCC.iqe_ref;
    CCC.vqe_ref = INVCTRL.fltEqe.fOut - CCC.fPI_DOut_Q - Lf * TWO_PI * PARAM_VAL(GRID_RATED_FREQ) * CCC.ide_ref;

    TRANSFORM_ROTATE_THETA_INVERSE(CCC.vde_ref, CCC.vqe_ref,
            INVCTRL.CosTheta, INVCTRL.SinTheta, INVCTRL.IdsRef,
            INVCTRL.IqsRef);

}
//--
#endif
// 최종 출력 ICC.fPIOut
static void CC_Mode_CCCV()
{
	volatile float a;

	//-IVC.fRef_command = (PARAM_VAL(BATT_LOCAL_IVC_V_REF));
	IVC.fRef_command = CC_GetReference();

	// Range Check
	if (IVC.fRef_command > PARAM_VAL(BATT_V_RANGE_MAX))
		IVC.fRef_command = PARAM_VAL(BATT_V_RANGE_MAX);
	else if (IVC.fRef_command < PARAM_VAL(BATT_V_RANGE_MIN))
		IVC.fRef_command = PARAM_VAL(BATT_V_RANGE_MIN);

	PRM_PCS[BATT_4].iValue = (int) IVC.fRef_command;

	if (IVC.bRAMPInputInitEnb == ON )
	{
		RAMP_SetInitOut(IVC.pRAMPInput, BATCTRL.DCLinkV); //Ramp 초기시작지점 설정
		IVC.bRAMPInputInitEnb = OFF;
	}

	/*
	 * =================================================================================
	 * DC Voltage Controller BLOCK (0)
	 */
	IVC.fRef = RAMP_Change(IVC.pRAMPInput, IVC.fRef_command);
	IVC.pARG->fErr = IVC.fRef - BATCTRL.DCLinkV;
	IVC.pARG->fFbk = BATCTRL.DCLinkV;

	// [DC Voltage Controller  Limit](2)**************************************************/
	if (BCState.powerFlow == PFLOW_CHARGE)
	{
		IVC.pARG->fMax = fabs(PARAM_VAL(BATT_IVC_PCC_CURRENT_LMT_CHG));
		IVC.pARG->fMin = 0;
	}
#if 1
	else
	{
		IVC.pARG->fMax = 0; // 출력이 전류 레퍼런스 이므로 출력 전류 0
		IVC.pARG->fMin = 0;
	}
#else
	else if(BCState.powerFlow == PFLOW_DISCHARGE)
	{
		IVC.pARG->fMax = 0;
		IVC.pARG->fMin = PARAM_VAL(BATT_IVC_PCC_CURRENT_LMT_DCHG);
	}
	else
	{
		IVC.pARG->fMax = PARAM_VAL(BATT_IVC_PCC_CURRENT_LMT_CHG);
		IVC.pARG->fMin = -PARAM_VAL(BATT_IVC_PCC_CURRENT_LMT_DCHG);
	}
#endif
	// -------------------------------------------------------------------------------*/

	if (PRM_PCS[CTRL_POWER_COMPEN].iValue)
		IVC.pARG->fCompen = BATCTRL.fPowerFilterd_1st * INVCTRL.EPeakInverse
				* 2. * INV_3;
	else
		IVC.pARG->fCompen = 0.;

	/*[ PI Controller Run - Voltage Controller](3)**************************************/
	PICON_RunM(IVC.pPI, IVC.pARG, IVC.fPIOut);

#if 1
	/*[ PI Controller Run - Current Controller](New)**************************************/
	//13.3.23
	//	b = ACP.INV.RATE.Iph_pk;
	a = ACP.INV.RATE.Iph_pk * INVCTRL.fCurrentLimit * 0.01;

	ICC.pARG->fMax = a;
	ICC.pARG->fMin = -(a);

	ICC.fRef = RAMP_Change(IVC.pRAMPOutput, IVC.fPIOut);
	ICC.pARG->fErr = ICC.fRef - BATCTRL.fltI2.fOut;
	ICC.pARG->fFbk = BATCTRL.fltI2.fOut;

	PICON_RunM(ICC.pPI, ICC.pARG, ICC.fPIOut);
#endif
}

static void CC_Mode_CP_P()
{
	//float fTemp = 0;

	// 최대 레퍼런스 120%. code 상으로 임시 제한.
	if (PCC.fRefP
			> PARAM_VAL(INV_CAPACITY) * 1000.f * 1.2 /*Inverter capacity*/)
	{
		PCC.fRefP = PARAM_VAL(INV_CAPACITY) * 1000.f * 1.2;
	}
	else if (PCC.fRefP < -(PARAM_VAL(INV_CAPACITY) * 1000.f) * 1.2)
	{
		PCC.fRefP = -(PARAM_VAL(INV_CAPACITY) * 1000.f * 1.2);
	}

	PRM_PCS[BATT_4].iValue = (int) PCC.fRefP / 1000.;

#if 0
	// +- 5% 이내의 reference 명령이 들어오면 0 로 둠.
	if( fabs(PCC.fRefP) <= PARAM_RAW_VAL(INV_CAPACITY) * 50 /* *1000w * 5% */)
	{
		PCC.fRefP = 0;
	}
#endif
#if 0
	if (PCC.fRefP != PCC.fRefBefore)
	{
		PCC.fRefBefore = PCC.fRefP;
		PCC.bTrig_softLevel = 0;
		PCC.bTrig_vdcLimit = 0;
		PCC.uTriggerTime = 0;
	}

	// 마이너스 값: 충전    플러스 값: 방전
	/*[(1) Charge: vdc_ref <= vdc  Discharge: vdc ref >= vdc   if true : ref = 0 ]*/
	/*[(2) Limit vdc * BATT_VDCREF_DCHG ~ vdc * -BATT_VDCREF_CHG ] */
	// 테스트를 위해 방전만 가능하도록 할 경우: PCC.fRef = fabs(PCC.fRef);
	if (BCState.powerFlow == PFLOW_DISCHARGE)
	{
		/*
		 *  Discharge
		 */
		// Limit.
		fTemp = BATCTRL.fltDCBattV_2nd.fOut
				* PARAM_VAL(BATT_IVC_PCC_CURRENT_LMT_DCHG);

		if (PCC.fRefP > fTemp)
			PCC.fRefP = fTemp;

		// 방전 전압 레퍼런스 + 5V 지점 부터는 파워 레퍼런스를 10%만 주도록 한다.
		if (BATCTRL.fltDCBattV_2nd.fOut <= PARAM_VAL(BATT_V_RANGE_MIN) + 5. /* 40 */
		|| PCC.bTrig_softLevel)
		{
			if (PCC.uTriggerTime >= 10 || PCC.bTrig_softLevel)
			{
				PCC.fRefP = PCC.fRefP * PCC.fRef_limit;
				PCC.bTrig_softLevel = 1;
				PCC.uTriggerTime = 0;
			}
		}
		else
			PCC.uTriggerTime = 0;
	}
	else if (BCState.powerFlow == PFLOW_CHARGE)
	{
		/*
		 * Charge
		 */
		//(2) Limit. ref가 0이면 limit 할 필요가 없음.
#if 0
		fTemp = BATCTRL.fltDCBattV_2nd.fOut
				* -PARAM_VAL(BATT_IVC_PCC_CURRENT_LMT_CHG);

		if (PCC.fRefP < fTemp /*마이너스 값!!*/)
			PCC.fRefP = fTemp;
#endif

		// 충전 전압 레퍼런스 - 5V 지점 부터는 파워 레퍼런스를 10%만 주도록 한다.
		if (BATCTRL.fltDCBattV_2nd.fOut >= PARAM_VAL(BATT_V_RANGE_MAX) - 5 /* 40 */
		|| PCC.bTrig_softLevel)
		{
			if (PCC.uTriggerTime >= 10 /*3sec*/|| PCC.bTrig_softLevel)
			{
				PCC.fRefP = PCC.fRefP * PCC.fRef_limit;
				PCC.bTrig_softLevel = 1;
				PCC.uTriggerTime = 0;
			}
		}
		else
			PCC.uTriggerTime = 0;
	}
	else
	{
		//else 이면 PCC.fRef 는 항상 0이다.
		PCC.bTrig_softLevel = 0;
		PCC.bTrig_vdcLimit = 0;
		PCC.uTriggerTime = 0;		// 151130
		PCC.bTrig_softLevel = 0; 	// 151130
	}
#endif
	/*
	 * [ fltAcPower P Q 값 계산 ] : 항시 계산 되도록 코드 이동
	 */

	/*[ PI Controller Run - P Power Controller]**************************************/
	a = ACP.INV.RATE.Iph_pk * INVCTRL.fCurrentLimit * 0.01 ;

	PCC.pARG->fMax = a;
	PCC.pARG->fMin = -(a);

	if( PRM_PCS[CTRL_REV_REMOTEPLIMIT].iValue > 110. )
		PRM_PCS[CTRL_REV_REMOTEPLIMIT].iValue = 110;

	if( PRM_PCS[CTRL_REV_REMOTEPLIMIT].iValue < 90. )
		PRM_PCS[CTRL_REV_REMOTEPLIMIT].iValue = 90;

	// 유효전력 Control
	PCC.pARG->fErr = EXCTRL.fltAcPowerP.fOut - PCC.fRefP;
	PCC.pARG->fFbk = EXCTRL.fltAcPowerP.fOut;
	//	PCC.pARG->fCompen = PCC.fRefP * INVCTRL.EPeakInverse * 2. * INV_3;

	PICON_RunM(PCC.pPI, PCC.pARG, PCC.fPIOut);
//	PCC.fPIOut += - PCC.fRefP * INVCTRL.EPeakInverse * 2. * INV_3;
}

#pragma CODE_SECTION (CC_Mode_CP_Q, "dp_ctrl")
static void CC_Mode_CP_Q()
{
	float qLimit = 0;
	float pfRate = 0;

	// PF Limit = 0.9

	/* 안티 아일랜딩 ------------------------------------------------------- */
	if (PRM_PCS[CTRL_ENABLE_ANTI_ISLANDING].iValue == 1	&& CTRL.INV.ctrl_mode != PARAM_VSI_PR)
	{
		//anti_omega_err = GRIDTHETA_AI.fFreqW - (GRID_ARG.RATE.fFreq * TWO_PI)/* *2PI하여 Omega로 변환 */;
		anti_omega_err = GRIDTHETA_AI.fltFreqW.fOut - ACP.PCC.RATE.Omega;
		b = (float) PARAM_VAL(CTRL_APS_LINE_DEADBAND) * TWO_PI;	//PRM_PCS[CTRL_APS_LINE_DEADBAND].fIncDec;

		// CONTHETA.fFreq가 DEADBAND를 벗어나면, 동작한다.
		if (anti_omega_err > b || anti_omega_err < -b)
		{
			b = PARAM_VAL(CTRL_ANTI_ISLANDING_K_FACTOR);
			if (b <= 0)
				b = 1;

			// tan과 TAN는 값이 다른게 나옴.
			// ( (0.059927625196 * sin((128.57142857)*a)) + ((0.3333333333)*(0.059927625196 * sin((128.57142857)*a))*(0.059927625196 * sin((128.57142857)*a))*(0.059927625196 * sin((128.57142857)*a))) + ((0.1333333333)*(0.059927625196 * sin((128.57142857)*a))*(0.059927625196 * sin((128.57142857)*a))*(0.059927625196 * sin((128.57142857)*a))*(0.059927625196 * sin((128.57142857)*a))*(0.059927625196 * sin((128.57142857)*a))) )
			// BEFORE a = TAN( (APS_LINE_COEF1 * b ) * sin(APS_LINE_COEF2 * (anti_omega_err * INV_TWO_PI)) );

			c = PARAM_VAL(GRID_OF_LEVEL1);
			d = PARAM_VAL(GRID_UF_LEVEL1);

			if( d > c )
				c = d; /* 둘 중 큰 수를 c에 넣는다.*/

			// 둘 중 큰 수가 1 보다 큰지 검사 한다.
			if( c > 1. )
				d = c; /*1(Hz) 이상의 차를 구한다.*/
			else
				d = 1.;/* 1보다 작을 경우 default 1 로 설정한다. */

			a = TAN(
					(APS_LINE_COEF1 * ( d /*1Hz이상의차:비율곱*/ ) * b /*KFACTOR*/ )	* 	sin( (DEGREE_90_RADIAN *( anti_omega_err / (d * TWO_PI)/*1Hz이상의차*/ ))  )
					);

			b = (float) PARAM_VAL(CTRL_REACTIVE_POWER_LIMIT) * d;//*(islading_frequency_Limit이 1HZ이상 부터는 * islading_frequency_Limit) ; //PRM_PCS[CTRL_REACTIVE_POWER_LIMIT].fIncDec;

#if 0 /* DEBUG */
			anti_a = a;
			anti_b = b;
#endif

			if (a >= b)
				a = b;
			else if (a <= -b)
				a = -b;

			// 파워 제어일 경우 Q 레퍼런스를 이용.
			if( PCC.fRefQ < 0 )
			{
				// Q측 레퍼런스가 음수일 경우 - HILL 실험 결과에 의하여 +-부호를 반대로 취함.
				PCC.fRefQ = (PCC.fRefQ) + fabs(a * PCC.fRefP);
			}
			else
			{
				// Q측 레퍼런스가 양수일 경우
				PCC.fRefQ = (PCC.fRefQ) - fabs(a * PCC.fRefP);
			}

		}
	}
	/* 안티 아일랜딩 ------------------------------------------------------- */

	pfRate = PARAM_VAL(INV_CAPACITY) * 1000.f / PARAM_VAL(COSPHI_FACTOR_LIMIT);
	pfRate *= pfRate; // sqr

	qLimit = sqrt( pfRate - fabs( EXCTRL.fltAcPowerP.fOut * EXCTRL.fltAcPowerP.fOut ) );
	// Q Limit
	if (PCC.fRefQ > qLimit)
		PCC.fRefQ = qLimit;
	else if (PCC.fRefQ < -(qLimit) )
		PCC.fRefQ = -qLimit;

	/*------------------------------------------------------------------------------*/
	/*[ PI Controller Run - Q Power Controller]**************************************/
	a = ACP.INV.RATE.Iph_pk * INVCTRL.fCurrentLimit * 0.01; // P와 동일?

	PCC.pARG_Q->fMax = a;
	PCC.pARG_Q->fMin = -(a);

	PCC.pARG_Q->fErr = EXCTRL.fltAcPowerQ.fOut - PCC.fRefQ;
	PCC.pARG_Q->fFbk = EXCTRL.fltAcPowerQ.fOut;			 	// 무효전력 Control

	PICON_RunM(PCC.pPI_Q, PCC.pARG_Q, PCC.fPIOut_Q);
}

#if 0
void c_int11_CCCV_before()
{
	//+130806 - 방전만 이용.
	IVC.fRef_command = (PARAM_VAL(BATT_LOCAL_IVC_V_REF_DCHG));

	if (IVC.bRAMPInputInitEnb == ON )
	{
		RAMP_SetInitOut(IVC.pRAMPInput, BATCTRL.DCLinkV); //Ramp 초기시작지점 설정
		IVC.bRAMPInputInitEnb = OFF;
	}

	/*
	 * =================================================================================
	 * DC Voltage Controller BLOCK (0)
	 */
	IVC.fRef = RAMP_Change(IVC.pRAMPInput, IVC.fRef_command);
	IVC.pARG->fErr = IVC.fRef - BATCTRL.fltDCBattVForCC.fOut;
	IVC.pARG->fFbk = BATCTRL.fltDCBattVForCC.fOut;

	// Iqe는 Peak값이고 사용자가 입력하는 정보는 RMS값이므로 변환 필요
	//	b = INVERTER.RATE.fCurrent * SQRT2;
	a = ACP.INV.RATE.Iph_pk * INVCTRL.fCurrentLimit * 0.01;

	IVC.pARG->fMax = a; //0->a; 131016
	IVC.pARG->fMin = -a; // 방전

	if (PRM_PCS[CTRL_POWER_COMPEN].iValue)
		IVC.pARG->fCompen = BATCTRL.fPowerFilterd_1st * INVCTRL.EPeakInverse
				* 2. * INV_3;
	else
		IVC.pARG->fCompen = 0.;

	/*[ PI Controller Run - Voltage Controller](3)**************************************/
	PICON_RunM(IVC.pPI, IVC.pARG, IVC.fPIOut);
}
#endif

static void CC_CalcInstantPower(void)
{
	/* Instantaneous Power Calculation -> Lowpass filter	100Hz		*/
	// 1.5 즉 3/2 를 곱하는 이유는 Transform 과정에 2/3 만큼의 성분이 추가되었기 때문에 다시 없애는 과정이다.
#if 0 /* GRID 센싱 */
	PCC.fPowerP = 1.5 * ( EXCTRL.pccEqs*EXCTRL.trIqs + EXCTRL.pccEds*EXCTRL.trIds );
	PCC.fPowerQ = 1.5 * ( EXCTRL.pccEqs*EXCTRL.trIds - EXCTRL.pccEds*EXCTRL.trIqs );
#else /* Inv V 센싱 */
	PCC.fPowerP = 1.5
			* ((INVCTRL.Eqs * INVCTRL.Iqs) + (INVCTRL.Eds * INVCTRL.Ids));
	PCC.fPowerQ = 1.5
			* ((INVCTRL.Eqs * INVCTRL.Ids) - (INVCTRL.Eds * INVCTRL.Iqs));
#endif

	PCC.fPowerP *= -1;
	PCC.fPowerQ *= -1;

	EXCTRL.GridPowerP = EXCTRL.fltFsBypI[0].fOut * GRID_ARG.MEASURE[0].fPhaseVoltage
						+ EXCTRL.fltFsBypI[1].fOut * GRID_ARG.MEASURE[1].fPhaseVoltage
						+ EXCTRL.fltFsBypI[2].fOut * GRID_ARG.MEASURE[2].fPhaseVoltage;
	EXCTRL.LoadPowerP = EXCTRL.GridPowerP + EXCTRL.fltAcPowerP.fOut;

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2(&EXCTRL.fltAcPowerP, PCC.fPowerP);
		IIR1_Filter2(&EXCTRL.fltAcPowerQ, PCC.fPowerQ);

		IIR1_Filter2(&EXCTRL.fltAcPowerP_2nd, EXCTRL.fltAcPowerP.fOut);
		IIR1_Filter2(&EXCTRL.fltAcPowerQ_2nd, EXCTRL.fltAcPowerQ.fOut);
	}
	else
	{
		IIR1_Filter2_IS(&EXCTRL.fltAcPowerP, PCC.fPowerP);
		IIR1_Filter2_IS(&EXCTRL.fltAcPowerQ, PCC.fPowerQ);

		IIR1_Filter2_IS(&EXCTRL.fltAcPowerP_2nd, EXCTRL.fltAcPowerP.fOut);
		IIR1_Filter2_IS(&EXCTRL.fltAcPowerQ_2nd, EXCTRL.fltAcPowerQ.fOut);
	}
}

static void CC_CalcPowerFactor(void)
{
	INVCTRL.fPowerFactor =
			fabs(EXCTRL.fltAcPowerP.fOut)
					/ sqrt(
							(EXCTRL.fltAcPowerP.fOut * EXCTRL.fltAcPowerP.fOut)
									+ (EXCTRL.fltAcPowerQ.fOut
											* EXCTRL.fltAcPowerQ.fOut));
}



// DEBUG varient. 추후 삭제.
float fBypOmega_Max = 0;
float fBypOmega_Min = 0;
void CC_BypOmegaReset()
{
	fBypOmega_Max = fBypOmega_Min = GRID_BYP_THETA.fltFreqW.fOut;
}

unsigned int tranCnt = 0;
Bool bBlockSeamless = FALSE;

#if CC_EXECUTION_PERIOD
volatile UInt32 start_count2;
volatile UInt32 end_count2;
#endif

#if CC_EXECUTION_TIME
volatile UInt32 start_count;
volatile UInt32 end_count;
#endif
//float a1,a2,b1,b2,c1,c2;
#pragma CODE_SECTION (c_int11, "dp_ctrl")
void c_int11(int iCallOrder)
{
#if CC_INTERRUPT_DOUBLE
	static Uns EV_ID;
#endif

#if 0	//	160503 bChange 들어 올때만 함수 동작해서 이상동작 하는 것 처럼 보여서 수정함
if( PRM_PCS[CTRL_TEST_MODE].iValue != 0 )
{
	if(PRM_PCS[SYS_REV_5].iValue == 1 )
	{
		GPIO_StaticSwitch(GPIO_STATIC_SW_ON);
	}
	else if( PRM_PCS[SYS_REV_5].iValue == 2)
	{
		GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
	}
}
#endif

	a = b = Ta = Tb = Tc = Tzero = DCLinkV2 = z = y = 0 ;

	// Watchdog
	ServiceDog();
	TEST_LedToggle(0);

	if (CallCounter1Sec++ >= CC_tsSampleInv) // 1초
	{
		CallCounter1Sec = 0;

		if( IVC.uStatus == CVC_ENABLE )
			PCC.uIdleTime++;
		else
			PCC.uIdleTime = 0;

		PCC.uTriggerTime++;

		for (iCCTemp = 0; iCCTemp < PEBB_COUNT_MAX; iCCTemp++)
			MO.uMO_Off_Wait_Delay[iCCTemp]++;

		// Power Factor Calc Display용이므로 1초에 1회만 호출
		CC_CalcPowerFactor();
	}

	if (!bSystemStarted)
	{
		return;
	}

#if CC_EXECUTION_TIME
	start_count = Clock_getTicks(); //CLK_gethtime the timer counter register is incremented every four CPU cycles
#endif

#if CC_EXECUTION_PERIOD
	// CPU Clock : 150MHz
	// 4/150 = 26.6666667nsec = 0.0266667usec
	start_count2 = Clock_getTicks();
	if (end_count2 < start_count2)
		//-CC_uExecutionTime2= (start_count2- end_count2) * 0.0266667;
		CC_uExecutionTime2= (start_count2- end_count2) * 10/*10us*/;
	else
		CC_uExecutionTime2 = (0xFFFFFFFF - start_count2 + end_count2) * 10;

//#if DOUBLE_CONTROL == 1
//	if( CC_uExecutionTime2 > 100)
//#else
//	if( CC_uExecutionTime2 > 200)
//#endif

#endif

	EADC_ADC_Result(); //
	XINTF_ADC_MUX_TOGGLE(); // ADC value를 가져온 후 Mux Toggle

#if DBUG_MODE == 1 || DBUG_MODE == 2 && IIVC_ENB
	if (IVC.uStatus == CVC_INIT_VOLTAGE_BUILD)
	{
#if 0
		a = 1./INVERTER.RATE.fPhaseVoltagePeak;
		EADC.pAdcInvEa->i16Val *= a * InitInvVControl_q.fRefP;
		EADC.pAdcInvEb->i16Val *= a * InitInvVControl_q.fRefP;
		EADC.pAdcInvEc->i16Val *= a * InitInvVControl_q.fRefP;
#endif
	}
#endif

// Converter Current
    EADC_GetAnalogValue(EADC.pAdcInvIa, &INVCTRL.Ia);

#if ADC_DECTECT_S == 1
    EADC_GetAnalogValue(EADC.pAdcInvIb, &INVCTRL.Ib);
#else
#if ADC_DECTECT_S == 0
#error "T상 먼저 ADC해야 함."
#endif
    EADC_GetAnalogValue(EADC.pAdcInvIc, &INVCTRL.Ic);
    INVCTRL.Ib = -(INVCTRL.Ia + INVCTRL.Ic);
#endif

#if ADC_DECTECT_T == 1
    EADC_GetAnalogValue(EADC.pAdcInvIc, &INVCTRL.Ic);
#else
#if ADC_DECTECT_S == 0
#error "ERROR"
#endif
    INVCTRL.Ic = -(INVCTRL.Ia + INVCTRL.Ib);
#endif

#ifdef JCNET_EMUL
    INVCTRL.Ia = 2110.0;
    INVCTRL.Ib = 2120.0;
    INVCTRL.Ic = 2130.0;
#endif
	ACP.INV.ia = -INVCTRL.Ia;
	ACP.INV.ib = -INVCTRL.Ib;
	ACP.INV.ic = -INVCTRL.Ic;

#if 0
	INVCTRL.trIa = -(INVCTRL.trIb + INVCTRL.trIc);  // adc R상 고장으로 인한 임시 처리
	INVCTRL.trIb = -(INVCTRL.trIa + INVCTRL.trIc);// S상 검출 안할 경우.
#endif
	a = ACP.INV.RATE.Iph_pk * (float) PRM_PCS[INV_OC_LEVEL].iValue * 0.01;
	b = -a;

	CkeckOC(INVCTRL.Ia, a, b);
	CkeckOC(INVCTRL.Ib, a, b);
	//Added
	CkeckOC(INVCTRL.Ic, a, b);

	//	16.07.29	Yang
	//CTRL_BYP_EVT_OPERATION_SCR_OC();

#if 0 // before
	if (INVCTRL.Ia_OC_Count >= CC_OC_COUNT_MAX
			|| INVCTRL.Ib_OC_Count >= CC_OC_COUNT_MAX
			|| INVCTRL.Ic_OC_Count >= CC_OC_COUNT_MAX)
	{
		FLT_Raise(FLTH_INV_OC);
		INVCTRL.Ia_OC_Count = 0;
		INVCTRL.Ib_OC_Count = 0;
		INVCTRL.Ic_OC_Count = 0;
	}
#else

#if 0 //by JCNET
	// 150128 June
	if( INVERTER.uStatus == SYS_INV_ISLANDING )
	{
		tranCnt = 0;
		bBlockSeamless = FALSE;

		//151125 BYP_30. OC Count 증가. IS전환 시 과도에 PWM OFF를 하여 0접압부터 전압이 살기 시작하여 Count를 5->25로 늘림.
		if (INVCTRL.Ia_OC_Count >= 25
				|| INVCTRL.Ib_OC_Count >= 25
				|| INVCTRL.Ic_OC_Count >= 25)
		{
			CTRL.IRT.bPwmOff = TRUE;

			if( CTRL.IRT.uOCSkipCntMax < CTRL.IRT.uOCSkipCnt )
				CTRL.IRT.uOCSkipCntMax = CTRL.IRT.uOCSkipCnt;

			if( CTRL.IRT.bPwmOff == TRUE )
			{
				FLT_Raise(FLTL_OC_WARN_IS);
			}

			if( CTRL.IRT.uOCSkipCnt++ > CC_tsSampleInv )
			{
				FLT_Raise(FLTH_INV_OC);
				INVCTRL.Ia_OC_Count = 0;
				INVCTRL.Ib_OC_Count = 0;
				INVCTRL.Ic_OC_Count = 0;
				CTRL.IRT.uOCSkipCnt = 0;
			}
		}
		else
		{
			CTRL.IRT.bPwmOff = FALSE;
			CTRL.IRT.uOCSkipCnt = 0;
		}
	}
	else
#endif //by JCNET
	{
		if(INVERTER.uStatus == SYS_INV_RUN)
		{
			if(tranCnt++ >= PRM_PCS[TRAN_CNT].iValue)
			{
				tranCnt = PRM_PCS[TRAN_CNT].iValue + 1;
				bBlockSeamless = FALSE;
			}
			else
				bBlockSeamless = TRUE;
		}
		else
		{
			tranCnt = 0;
			bBlockSeamless = FALSE;
		}

		CTRL.IRT.bPwmOff = FALSE;
		CTRL.IRT.uOCSkipCnt = 0;

		if (INVCTRL.Ia_OC_Count >= CC_OC_COUNT_MAX
				|| INVCTRL.Ib_OC_Count >= CC_OC_COUNT_MAX
				|| INVCTRL.Ic_OC_Count >= CC_OC_COUNT_MAX)
		{
			FLT_Raise(FLTH_INV_OC);
			INVCTRL.Ia_OC_Count = 0;
			INVCTRL.Ib_OC_Count = 0;
			INVCTRL.Ic_OC_Count = 0;
		}
	}
#endif

#if 0 //by JCNET
	CTRL_SEAMLESS_PWM_OFF();
#endif

#if 0 //by JCNET. Ea,Eb,Ec를 검출된 adc가 아닌 계산(두 상의 차이로 만들어내는)을 위해 무조건 grid_connected_enb는 ON을 추함..
	if ((CTRL.INV.operation_mode == PARAM_OPERATION_MODE_GC
			|| CTRL.INV.operation_mode == PARAM_OPERATION_MODE_AUTO_IS)
			&& (INVERTER.uStatus == SYS_INV_RUN))
		CTRL.INV.grid_connected_enb = ON;
	else
		CTRL.INV.grid_connected_enb = OFF;
#else
    CTRL.INV.grid_connected_enb = ON;
#endif // by JCNET
	if (!CTRL.INV.grid_connected_enb)
	{
		a = ACP.INV.RATE.Iph_pk * (float) PRM_PCS[VI_LEVEL].iValue * 0.01;
		b = -a;
		if (INVCTRL.Ia >= a || INVCTRL.Ia <= b || INVCTRL.Ib >= a
				|| INVCTRL.Ib <= b || INVCTRL.Ic >= a || INVCTRL.Ic <= b)
		{
			CTRL_INV_VI_OneShotEnable();
			if (PRM_PCS[TRC_TRACE_MODE].iValue == TRC_VI_ENB_STOP)
				TRC_StopTrace();
		}
	}

#if 0 // 150128 for 전기안전연구원 IslandingRightThrow. OC 발생하는 코드가 있어서 주석처리.
	/*
	 * ! Replace below code with better logic
	 * When inverter returns from fault state to normal state, CCI state should be started at OCCHECK.
	 * It will not cause significant problems even if the below codes are not excuted,
	 * but excute below codes for safety.
	 */
	if (INVERTER.uPrevStatus == SYS_INV_FAULT
			&& INVERTER.uStatus != SYS_INV_FAULT)
	{
		INVCTRL.CCI_Status = CCI_OCCHECK;
		INVCTRL.CCI_2ndOC_Status = CCI_OCCHECK;
		INVCTRL.CCI_OCCheckCnt = 0;
	}

	switch (INVCTRL.CCI_Status)
	{
	case CCI_OCCHECK:
		if (INVCTRL.Ia_OC_Count >= CC_CCI_OC_COUNT
				|| INVCTRL.Ic_OC_Count >= CC_CCI_OC_COUNT)
		{
			if (++INVCTRL.CCI_OCCheckCnt >= 3)
			{
				FLT_Raise(FLTH_INV_OC);

			}
			else
			{
				INVCTRL.CCI_Status = CCI_NOGATING;
				INVCTRL.CCI_NoGatingCnt = 0;
			}
		}
		break;
	case CCI_NOGATING:
		if (++INVCTRL.CCI_NoGatingCnt
				> ceil(0.5 / (CC_tsCC * (float) PARAM_VAL(GRID_RATED_FREQ))))
		{
			if (INVCTRL.FrtMode != FRT_NORMAL)
			{
				RAMP_Initialize(INVCTRL.pRampIqeRefDelay);
				INVCTRL.CCI_Status = CCI_DELAY;
				INVCTRL.CCI_DelayCnt = 0;

				INVCTRL.CCI_2ndOC_Status = CCI_OCCHECK;
			}
			else
			{	// Ordinary OC situation
				INVCTRL.CCI_Status = CCI_OCCHECK;
			}

		}
		break;
	case CCI_DELAY:
		// Use IqeRefDelay for {CTRL_FRT_IQ_RAMP_TIME}
#if DOUBLE_CONTROL == 1
		if (++INVCTRL.CCI_DelayCnt > (PARAM_RAW_VAL(CTRL_FRT_IQ_RAMP_USE_TIME)*1000/ PARAM_RAW_VAL(CTRL_CC_PERIOD) / 2))
#else
		if (++INVCTRL.CCI_DelayCnt > (PARAM_RAW_VAL(CTRL_FRT_IQ_RAMP_USE_TIME)*1000/ PARAM_RAW_VAL(CTRL_CC_PERIOD)))
#endif
		{
			RAMP_SetInitOut(INVCTRL.pRampIqeRef, INVCTRL.pRampIqeRefDelay->fOut);
			INVCTRL.CCI_OCCheckCnt = 0;
			INVCTRL.CCI_Status = CCI_OCCHECK;
		}

		switch(INVCTRL.CCI_2ndOC_Status)
		{
			case CCI_OCCHECK:
			if( INVCTRL.Ia_OC_Count >= CC_CCI_OC_COUNT || INVCTRL.Ic_OC_Count >= CC_CCI_OC_COUNT )
			{
				INVCTRL.CCI_2ndOC_NoGatingCnt = 0;
				INVCTRL.CCI_2ndOC_Status = CCI_NOGATING;
			}
			break;
			case CCI_NOGATING:
#if DOUBLE_CONTROL == 1
			if( ++INVCTRL.CCI_2ndOC_NoGatingCnt > (float)(PARAM_VAL(CTRL_FRT_IQ_HOLD_TIME)*1000/ (float)PARAM_RAW_VAL(CTRL_CC_PERIOD) / 2))
#else
			if( ++INVCTRL.CCI_2ndOC_NoGatingCnt > (float)(PARAM_VAL(CTRL_FRT_IQ_HOLD_TIME)*1000/ (float)PARAM_RAW_VAL(CTRL_CC_PERIOD)))
#endif
			{
				RAMP_Initialize(INVCTRL.pRampIqeRefDelay);
				INVCTRL.CCI_2ndOC_Status = CCI_OCCHECK;
			}
			break;
			default:
			INVCTRL.CCI_2ndOC_Status = CCI_OCCHECK;
		}

		break;
		default:
		INVCTRL.CCI_Status = CCI_OCCHECK;
		//CON.CCI_OCCheckCnt = 0;
		break;
	}
#endif

//by JCNET TODOO
// DCLinkV와 DCBattV값을 일치시키?
	EADC_GetAnalogValue(EADC.pAdcDCLinkV, &BATCTRL.DCLinkV);
	EADC_GetAnalogValue(EADC.pAdcDCBattV, &BATCTRL.DCBattV);

#ifdef JCNET_EMUL
	BATCTRL.DCLinkV = 123.0;
	BATCTRL.DCBattV = 125.0;
#endif

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2( &BATCTRL.fltDCLinkV, BATCTRL.DCLinkV);
		IIR1_Filter2( &BATCTRL.fltDCLinkV2nd, BATCTRL.fltDCLinkV.fOut);
		IIR1_Filter2( &BATCTRL.fltDCBattVForCC, BATCTRL.DCBattV);
		IIR1_Filter2( &BATCTRL.fltDCBattV, BATCTRL.DCBattV);
		IIR1_Filter2( &BATCTRL.fltDCBattV_2nd, BATCTRL.fltDCBattV.fOut);
	}
	else
	{
		IIR1_Filter2_IS( &BATCTRL.fltDCLinkV, BATCTRL.DCLinkV);
		IIR1_Filter2_IS( &BATCTRL.fltDCLinkV2nd, BATCTRL.fltDCLinkV.fOut);
		IIR1_Filter2_IS( &BATCTRL.fltDCBattVForCC, BATCTRL.DCBattV);
		IIR1_Filter2_IS( &BATCTRL.fltDCBattV, BATCTRL.DCBattV);
		IIR1_Filter2_IS( &BATCTRL.fltDCBattV_2nd, BATCTRL.fltDCBattV.fOut);
	}

	DCLinkV2 = BATCTRL.fltDCLinkV.fOut;
	ACP.INV.tsw_div_vdc = CC_tsCC / DCLinkV2;

	if (PRM_PCS[CTRL_TEST_MODE].iValue != 1)
	{
		a = BATCTRL.DCLinkV;

		if (a > PARAM_VAL(BATT_OV_LEVEL) && INVERTER.uStatus != SYS_INV_STOP)
		{
			if (++INVCTRL.Batt_OVCheckCnt > CC_OV_UV_COUNT)
			{
				FLT_Raise(FLTH_BATT_OV);
				INVCTRL.Batt_OVCheckCnt = 0;
			}
		}
		else
			INVCTRL.Batt_OVCheckCnt = 0;

		//if( PRM_PCS[BYP_MODE].iValue != 20 )
		{
			if (a < PARAM_VAL(BATT_UV_LEVEL) && (IVC.uStatus != CVC_DISABLE || INVERTER.uStatus == SYS_INV_RUN))
			{
				if (++INVCTRL.Batt_UVCheckCnt > CC_OV_UV_COUNT)
				{
					FLT_Raise(FLTH_BATT_UV);
					INVCTRL.Batt_UVCheckCnt = 0;
				}
			}
			else
				INVCTRL.Batt_UVCheckCnt = 0;
		}

		//-Original if (a < PARAM_VAL(BATT_UV_LEVEL) && IVC.uStatus != CVC_DISABLE)
		// 150413 Run 상태 & PWM OFF 에서 UV 검출이 안되어 아래와 같이 수정 함.
		// 150414 하단의 코드는 STOP 상태에서도 Batt UV가 간혈적으로 발생(MG모델)
		//-150414 if (a < PARAM_VAL(BATT_UV_LEVEL) && !(INVERTER.uStatus == SYS_INV_STOP || INVERTER.uStatus == SYS_INV_DC_CHARGE))

		//+150415 June - RUN 상태에서 PWM OFF상태에서도 UV 검출 되도록 재수정.

		//+Added Safety 인증.
		if ((IVC.uStatus != CVC_DISABLE) && (INVERTER.uStatus == SYS_INV_RUN) )
		{
			// 운전 중 Dclink 와 Battery V 전압차 발생시 Fault
			a = BATCTRL.fltDCBattV_2nd.fOut - BATCTRL.fltDCLinkV2nd.fOut;

			if (fabs(a) > PARAM_VAL(BATT_DC_SIDE_V_ABNORMAL_LEVEL))
			{
				if( ++INVCTRL.DcSideAbnormalCnt > CC_DC_ABNORMAL_COUNT )
				{
					FLT_Raise(FLTH_DC_SIDE_ABNORMAL);
					INVCTRL.DcSideAbnormalCnt = 0;
				}
			}
			else
				INVCTRL.DcSideAbnormalCnt = 0;
		}
	}

	/*------------------------------------------------------------------------------------------------------------------*/
	/* GEN Voltage : Sampling -> FS RMS -> IIR Filtering of FS RMS		    											*/
	/*------------------------------------------------------------------------------------------------------------------*/
#if HILL_BYPV_TO_GENV == 1
	EADC_GetAnalogValue(EADC.pAdcBypEa, &EXCTRL.genEa);
	EADC_GetAnalogValue(EADC.pAdcBypEb, &EXCTRL.genEb);
	EADC_GetAnalogValue(EADC.pAdcBypEc, &EXCTRL.genEc);
#elif HILL_CAPI_TO_GENV == 1
	EADC_GetAnalogValue(EADC.pAdcCapIc, &EXCTRL.genEa);
	EADC_GetAnalogValue(EADC.pAdcCapIa, &EXCTRL.genEc);
	EXCTRL.genEb = -(EXCTRL.genEa + EXCTRL.genEc);

	EXCTRL.genEa *= 3.321;
	EXCTRL.genEb *= 3.321;
	EXCTRL.genEc *= 3.321;
#else
	EADC_GetAnalogValue(EADC.pAdcGenEa, &EXCTRL.genEa);
	EADC_GetAnalogValue(EADC.pAdcGenEb, &EXCTRL.genEb);
	EADC_GetAnalogValue(EADC.pAdcGenEc, &EXCTRL.genEc);
#endif
	ACP.GEN.va = EXCTRL.genEa;
	ACP.GEN.vb = EXCTRL.genEb;
	ACP.GEN.vc = EXCTRL.genEc;
	ACP.GEN.vds = (2 * ACP.GEN.va - ACP.GEN.vb - ACP.GEN.vc) * INV_3;
	ACP.GEN.vqs = (-ACP.GEN.vb + ACP.GEN.vc) * INV_SQRT3;
	// Calculate RMS value for GRID voltage
	FASTRMS_AddSample(&FastRmsItems.fsGenV[0], EXCTRL.genEa);
	FASTRMS_AddSample(&FastRmsItems.fsGenV[1], EXCTRL.genEb);
	FASTRMS_AddSample(&FastRmsItems.fsGenV[2], EXCTRL.genEc);
	// Filtering GRID rms value
	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2( &EXCTRL.fltFsGenV[0], sqrt(FastRmsItems.fsGenV[0].fRMS2));
		IIR1_Filter2( &EXCTRL.fltFsGenV[1], sqrt(FastRmsItems.fsGenV[1].fRMS2));
		IIR1_Filter2( &EXCTRL.fltFsGenV[2], sqrt(FastRmsItems.fsGenV[2].fRMS2));
	}
	else
	{
		IIR1_Filter2_IS( &EXCTRL.fltFsGenV[0],	sqrt(FastRmsItems.fsGenV[0].fRMS2));
		IIR1_Filter2_IS( &EXCTRL.fltFsGenV[1],	sqrt(FastRmsItems.fsGenV[1].fRMS2));
		IIR1_Filter2_IS( &EXCTRL.fltFsGenV[2],	sqrt(FastRmsItems.fsGenV[2].fRMS2));
	}
	TRANSFORM_abc_dq(EXCTRL.genEa, EXCTRL.genEb, EXCTRL.genEc, EXCTRL.genEds, EXCTRL.genEqs);

	/* GEN 전압 Freq */
	ThetaDetectRun(&GEN_THETA, EXCTRL.fltEde_gen.fOut); // R : 8us
	EXCTRL.SinTheta_Gen = sin(GEN_THETA.fRadian);
	EXCTRL.CosTheta_Gen = cos(GEN_THETA.fRadian);
	TRANSFORM_ROTATE_THETA(EXCTRL.genEds, EXCTRL.genEqs, EXCTRL.CosTheta_Gen,
			EXCTRL.SinTheta_Gen, EXCTRL.genEde, EXCTRL.genEqe);

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2( &EXCTRL.fltEde_gen, EXCTRL.genEde);
		IIR1_Filter2( &EXCTRL.fltEqe_gen, EXCTRL.genEqe);
		IIR1_Filter2( &EXCTRL.fltFreq_Gen, GEN_THETA.fFreq);
	}
	else
	{
		IIR1_Filter2_IS( &EXCTRL.fltEde_gen, EXCTRL.genEde);
		IIR1_Filter2_IS( &EXCTRL.fltEqe_gen, EXCTRL.genEqe);
		IIR1_Filter2_IS( &EXCTRL.fltFreq_Gen, GEN_THETA.fFreq);
	}

	/*------------------------------------------------------------------------------------------------------------------*/
	/* BYPASS Voltage : Sampling -> FS RMS -> IIR Filtering of FS RMS		    											*/
	/*------------------------------------------------------------------------------------------------------------------*/
	// Vbyp
#ifndef SWAP_BYP_OUTEA
	EADC_GetAnalogValue(EADC.pAdcBypEa, &EXCTRL.bypEa);
	EADC_GetAnalogValue(EADC.pAdcBypEb, &EXCTRL.bypEb);
	EADC_GetAnalogValue(EADC.pAdcBypEc, &EXCTRL.bypEc);
#else
    EADC_GetAnalogValue(EADC.pAdcBypEa, &EXCTRL.pccEa);
    EADC_GetAnalogValue(EADC.pAdcBypEb, &EXCTRL.pccEb);
    EADC_GetAnalogValue(EADC.pAdcBypEc, &EXCTRL.pccEc);
#endif
    //by JCNET for test

#ifdef JCNET_EMUL
    {
#define V_PEAK (700.0/2)
        static float radian = 0.0;
        EXCTRL.bypEa = sin(radian) * V_PEAK;
        EXCTRL.bypEb = sin(radian + 2/3.0 * PI) * V_PEAK;
        EXCTRL.bypEc = sin(radian + 4/3.0 * PI) * V_PEAK;
        radian += TWO_PI/(10000/60.0); // (1/GridFreq)
        if(radian > TWO_PI) radian -= TWO_PI;
    }
#endif
    //end test
#if HILL_CAPI_TO_BYPV == 1
	EADC_GetAnalogValue(EADC.pAdcCapIa, &EXCTRL.bypEa);
	EADC_GetAnalogValue(EADC.pAdcCapIc, &EXCTRL.bypEc);

	//EADC_GetAnalogValue(EADC.pAdcCapIb, &EXCTRL.bypEb);
	EXCTRL.bypEb = -(EXCTRL.bypEa + EXCTRL.bypEc);

	EXCTRL.bypEa *= 1.99;
	EXCTRL.bypEb *= 1.99;
	EXCTRL.bypEc *= 1.99;
#endif

#if HILL_BYPV_TO_GENV == 1
	EADC_GetAnalogValue(EADC.pAdcGenEa, &EXCTRL.bypEa);
	EADC_GetAnalogValue(EADC.pAdcGenEb, &EXCTRL.bypEb);
	EADC_GetAnalogValue(EADC.pAdcGenEc, &EXCTRL.bypEc);
#endif

	ACP.BYP.va = EXCTRL.bypEa;
	ACP.BYP.vb = EXCTRL.bypEb;
	ACP.BYP.vc = EXCTRL.bypEc;
	ACP.BYP.vds = (2 * ACP.BYP.va - ACP.BYP.vb - ACP.BYP.vc) * INV_3;
	ACP.BYP.vqs = (-ACP.BYP.vb + ACP.BYP.vc) * INV_SQRT3;

	// Calculate RMS value for GRID voltage
#if GRID_RATED_VOLTAGE == 480
	FASTRMS_AddSample(&FastRmsItems.fsBypV[0], 0.1 * EXCTRL.bypEa);
	FASTRMS_AddSample(&FastRmsItems.fsBypV[1], 0.1 * EXCTRL.bypEb);
	FASTRMS_AddSample(&FastRmsItems.fsBypV[2], 0.1 * EXCTRL.bypEc);

    if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
    {
        // sqrt7 를 사용하니 보호 시험 정확도가 떨어져서 잘 안됨.
        IIR1_Filter2( &EXCTRL.fltFsBypV[0], 10. * sqrt(FastRmsItems.fsBypV[0].fRMS2));
        IIR1_Filter2( &EXCTRL.fltFsBypV[1], 10. * sqrt(FastRmsItems.fsBypV[1].fRMS2));
        IIR1_Filter2( &EXCTRL.fltFsBypV[2], 10. * sqrt(FastRmsItems.fsBypV[2].fRMS2));
    }
    else
    {
        IIR1_Filter2_IS( &EXCTRL.fltFsBypV[0], 10. * sqrt(FastRmsItems.fsBypV[0].fRMS2));
        IIR1_Filter2_IS( &EXCTRL.fltFsBypV[1], 10. * sqrt(FastRmsItems.fsBypV[1].fRMS2));
        IIR1_Filter2_IS( &EXCTRL.fltFsBypV[2], 10. * sqrt(FastRmsItems.fsBypV[2].fRMS2));
    }
#else
    FASTRMS_AddSample(&FastRmsItems.fsBypV[0], EXCTRL.bypEa);
    FASTRMS_AddSample(&FastRmsItems.fsBypV[1], EXCTRL.bypEb);
    FASTRMS_AddSample(&FastRmsItems.fsBypV[2], EXCTRL.bypEc);

    if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
    {
        // sqrt7 를 사용하니 보호 시험 정확도가 떨어져서 잘 안됨.
        IIR1_Filter2( &EXCTRL.fltFsBypV[0], sqrt(FastRmsItems.fsBypV[0].fRMS2));
        IIR1_Filter2( &EXCTRL.fltFsBypV[1], sqrt(FastRmsItems.fsBypV[1].fRMS2));
        IIR1_Filter2( &EXCTRL.fltFsBypV[2], sqrt(FastRmsItems.fsBypV[2].fRMS2));
    }
    else
    {
        IIR1_Filter2_IS( &EXCTRL.fltFsBypV[0],  sqrt(FastRmsItems.fsBypV[0].fRMS2));
        IIR1_Filter2_IS( &EXCTRL.fltFsBypV[1],  sqrt(FastRmsItems.fsBypV[1].fRMS2));
        IIR1_Filter2_IS( &EXCTRL.fltFsBypV[2],  sqrt(FastRmsItems.fsBypV[2].fRMS2));
    }
#endif

	aaa =  sqrt(FastRmsItems.fsBypV[0].fRMS2); /* XXX 1905 Test Sample */
	TRANSFORM_abc_dq(EXCTRL.bypEa, EXCTRL.bypEb, EXCTRL.bypEc, EXCTRL.bypEds, EXCTRL.bypEqs);

	/*------------------------------------------------------------------------------------------------------------------*/
	/* [Start Block] GRID, INV Voltage : Sampling -> Pos Seq -> abc2xy -> FS RMS -> IIR Filtering of FS RMS		    	*/
	/*------------------------------------------------------------------------------------------------------------------*/

	// Vpcc
#ifndef SWAP_BYP_OUTEA
    EADC_GetAnalogValue(EADC.pAdcOutEa, &EXCTRL.pccEa);
    EADC_GetAnalogValue(EADC.pAdcOutEb, &EXCTRL.pccEb);
    EADC_GetAnalogValue(EADC.pAdcOutEc, &EXCTRL.pccEc);
#else
    EADC_GetAnalogValue(EADC.pAdcOutEa, &EXCTRL.bypEa);
    EADC_GetAnalogValue(EADC.pAdcOutEb, &EXCTRL.bypEb);
    EADC_GetAnalogValue(EADC.pAdcOutEc, &EXCTRL.bypEc);
#endif
    //by JCNET for test

#ifdef JCNET_EMUL
    {
#define V_PEAK (700.0/2)
        static float radian = 0.0;
        EXCTRL.pccEa = sin(radian) * V_PEAK;
        EXCTRL.pccEb = sin(radian + 2/3.0 * PI) * V_PEAK;
        EXCTRL.pccEc = sin(radian + 4/3.0 * PI) * V_PEAK;
        radian += TWO_PI/(10000/60.0); // (1/GridFreq)
        if(radian > TWO_PI) radian -= TWO_PI;
    }
#endif
    //end test
	/*
	 * ----------------------------------------------------------------------------
	 * Positive sequence for GRID voltage
	 * (This is used in RMS module, not FASTSUM)
	 */
	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		EXCTRL.EaDelay90 = APS_Filtering_GC(&EXCTRL.fltEaDelay90, EXCTRL.pccEa);
		EXCTRL.EbDelay90 = APS_Filtering_GC(&EXCTRL.fltEbDelay90, EXCTRL.pccEb);
		EXCTRL.EcDelay90 = APS_Filtering_GC(&EXCTRL.fltEcDelay90, EXCTRL.pccEc);
	}
	else
	{
		EXCTRL.EaDelay90 = APS_Filtering_IS(&EXCTRL.fltEaDelay90, EXCTRL.pccEa);
		EXCTRL.EbDelay90 = APS_Filtering_IS(&EXCTRL.fltEbDelay90, EXCTRL.pccEb);
		EXCTRL.EcDelay90 = APS_Filtering_IS(&EXCTRL.fltEcDelay90, EXCTRL.pccEc);
	}

	EXCTRL.Eap =
			0.5
					* (EXCTRL.pccEa
							- INV_SQRT3 * (EXCTRL.EbDelay90 - EXCTRL.EcDelay90));
	EXCTRL.Ecp =
			0.5
					* (EXCTRL.pccEc
							- INV_SQRT3 * (EXCTRL.EaDelay90 - EXCTRL.EbDelay90));
	//-14.1.15 EXCTRL.Ebp = -(EXCTRL.Eap + EXCTRL.Ecp); // calc
	EXCTRL.Ebp =
			0.5
					* (EXCTRL.pccEb
							- INV_SQRT3 * (EXCTRL.EcDelay90 - EXCTRL.EaDelay90));

	/*------------------------------------------------------------------------------*/

	TRANSFORM_abc_dq(EXCTRL.pccEa, EXCTRL.pccEb, EXCTRL.pccEc, EXCTRL.pccEds, EXCTRL.pccEqs);

	// Calculate RMS value for GRID voltage
#if GRID_RATED_VOLTAGE == 480
	FASTRMS_AddSample(&FastRmsItems.fsGridV[0], 0.1 * EXCTRL.pccEa);
	FASTRMS_AddSample(&FastRmsItems.fsGridV[1], 0.1 * EXCTRL.pccEb);
	FASTRMS_AddSample(&FastRmsItems.fsGridV[2], 0.1 * EXCTRL.pccEc);

    if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
    {
        IIR1_Filter2( &EXCTRL.fltFsGridV[0], 10. * sqrt(FastRmsItems.fsGridV[0].fRMS2));
        IIR1_Filter2( &EXCTRL.fltFsGridV[1], 10. * sqrt(FastRmsItems.fsGridV[1].fRMS2));
        IIR1_Filter2( &EXCTRL.fltFsGridV[2], 10. * sqrt(FastRmsItems.fsGridV[2].fRMS2));
    }
    else
    {
        IIR1_Filter2_IS( &EXCTRL.fltFsGridV[0], 10. * sqrt(FastRmsItems.fsGridV[0].fRMS2));
        IIR1_Filter2_IS( &EXCTRL.fltFsGridV[1], 10. * sqrt(FastRmsItems.fsGridV[1].fRMS2));
        IIR1_Filter2_IS( &EXCTRL.fltFsGridV[2], 10. * sqrt(FastRmsItems.fsGridV[2].fRMS2));
    }
#else
    FASTRMS_AddSample(&FastRmsItems.fsGridV[0], EXCTRL.pccEa);
    FASTRMS_AddSample(&FastRmsItems.fsGridV[1], EXCTRL.pccEb);
    FASTRMS_AddSample(&FastRmsItems.fsGridV[2], EXCTRL.pccEc);

    if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
    {
        IIR1_Filter2( &EXCTRL.fltFsGridV[0], sqrt(FastRmsItems.fsGridV[0].fRMS2));
        IIR1_Filter2( &EXCTRL.fltFsGridV[1], sqrt(FastRmsItems.fsGridV[1].fRMS2));
        IIR1_Filter2( &EXCTRL.fltFsGridV[2], sqrt(FastRmsItems.fsGridV[2].fRMS2));
    }
    else
    {
        IIR1_Filter2_IS( &EXCTRL.fltFsGridV[0], sqrt(FastRmsItems.fsGridV[0].fRMS2));
        IIR1_Filter2_IS( &EXCTRL.fltFsGridV[1], sqrt(FastRmsItems.fsGridV[1].fRMS2));
        IIR1_Filter2_IS( &EXCTRL.fltFsGridV[2], sqrt(FastRmsItems.fsGridV[2].fRMS2));
    }
#endif

	// Seth, 2013-06-08
    EADC_GetAnalogValue(EADC.pAdcInvEa, &INVCTRL.Ea);
    EADC_GetAnalogValue(EADC.pAdcInvEb, &INVCTRL.Eb);
    EADC_GetAnalogValue(EADC.pAdcInvEc, &INVCTRL.Ec);

#ifdef JCNET_EMUL
    {
#define V_PEAK (700.0/2)
        static float radian = 0.0;
        INVCTRL.Ea = sin(radian) * V_PEAK;
        INVCTRL.Eb = sin(radian + 2/3.0 * PI) * V_PEAK;
        INVCTRL.Ec = sin(radian + 4/3.0 * PI) * V_PEAK;
        radian += TWO_PI/(10000/60.0); // (1/GridFreq)
        if(radian > TWO_PI) radian -= TWO_PI;
    }
#endif
	INVCTRL.Eab = INVCTRL.Ea - INVCTRL.Eb;
	INVCTRL.Ebc = INVCTRL.Eb - INVCTRL.Ec;
	INVCTRL.Eca = INVCTRL.Ec - INVCTRL.Ea;

#if INV_V_USE_LTL == 1
	if( PRM_PCS[SYS_REV_5].iValue == 1 )
	{
		INVCTRL.Eab = INVCTRL.Ea;
		INVCTRL.Ebc = -(INVCTRL.Ea + INVCTRL.Ec);
		INVCTRL.Eca = INVCTRL.Ec;

		INVCTRL.Ea = (INVCTRL.Eab - INVCTRL.Eca) * INV_3;
		INVCTRL.Eb = (INVCTRL.Ebc - INVCTRL.Eab) * INV_3;
		INVCTRL.Ec = (INVCTRL.Eca - INVCTRL.Ebc) * INV_3;
	}
#endif

#if 0
	//- Seth, 2013-06-08
	INVCTRL.pccEa = (TR_RATIO_20_380 * INV_SQRT3) * (INVCTRL.Eat - INVCTRL.Ebt);
	INVCTRL.pccEb = (TR_RATIO_20_380 * INV_SQRT3) * (INVCTRL.Ebt - INVCTRL.Ect);
	INVCTRL.pccEc = (TR_RATIO_20_380 * INV_SQRT3) * (INVCTRL.Ect - INVCTRL.Eat);
#endif

	/*
	 * Positive sequence for INV voltage
	 */
	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		INVCTRL.EaDelay90 = APS_Filtering_GC(&INVCTRL.fltEaDelay90, INVCTRL.Ea);
		INVCTRL.EbDelay90 = APS_Filtering_GC(&INVCTRL.fltEbDelay90, INVCTRL.Eb);
		INVCTRL.EcDelay90 = APS_Filtering_GC(&INVCTRL.fltEcDelay90, INVCTRL.Ec);
		CTRL.INV.VI.iqs_delay90 = 0.9752 * SQRT2 * APS_Filtering_GC(&CTRL.INV.VI.fltIqsDelay90, ACP.INV.iqs);
	}
	else
	{
		INVCTRL.EaDelay90 = APS_Filtering_IS(&INVCTRL.fltEaDelay90, INVCTRL.Ea);
		INVCTRL.EbDelay90 = APS_Filtering_IS(&INVCTRL.fltEbDelay90, INVCTRL.Eb);
		INVCTRL.EcDelay90 = APS_Filtering_IS(&INVCTRL.fltEcDelay90, INVCTRL.Ec);
		CTRL.INV.VI.iqs_delay90 = 0.9752 * SQRT2 * APS_Filtering_IS(&CTRL.INV.VI.fltIqsDelay90, ACP.INV.iqs);
	}

	/*
	 INVCTRL.Eap = 0.5 * (INVCTRL.Ea - INV_SQRT3 * (INVCTRL.EbDelay90 - INVCTRL.EcDelay90));
	 INVCTRL.Ecp = 0.5 * (INVCTRL.Ec - INV_SQRT3 * (INVCTRL.EaDelay90 - INVCTRL.EbDelay90));
	 //-14.1.15INVCTRL.Ebp = -(INVCTRL.Eap + INVCTRL.Ecp);
	 INVCTRL.Ebp = 0.5 * (INVCTRL.Eb - INV_SQRT3 * (INVCTRL.EcDelay90 - INVCTRL.EaDelay90));
	 */
	ACP.INV.va_p =
			0.5
					* (INVCTRL.Ea
							- INV_SQRT3
									* (INVCTRL.EbDelay90 - INVCTRL.EcDelay90));
	ACP.INV.vc_p =
			0.5
					* (INVCTRL.Ec
							- INV_SQRT3
									* (INVCTRL.EaDelay90 - INVCTRL.EbDelay90));
	//	ACP.INV.vb_p = -(ACP.INV.va_p + ACP.INV.vc_p);
	ACP.INV.vb_p =
			0.5
					* (INVCTRL.Eb
							- INV_SQRT3
									* (INVCTRL.EcDelay90 - INVCTRL.EaDelay90));

	ACP.INV.va_n =
			0.5
					* (INVCTRL.Ea
							+ INV_SQRT3
									* (INVCTRL.EbDelay90 - INVCTRL.EcDelay90));
	ACP.INV.vc_n =
			0.5
					* (INVCTRL.Ec
							+ INV_SQRT3
									* (INVCTRL.EaDelay90 - INVCTRL.EbDelay90));
	//	ACP.INV.vb_n = -(ACP.INV.va_n + ACP.INV.vc_n);
	ACP.INV.vb_n =
			0.5
					* (INVCTRL.Eb
							+ INV_SQRT3
									* (INVCTRL.EcDelay90 - INVCTRL.EaDelay90));
	//----------------------------------------------------------------------

	/*
	 * Select which sensing data to be used for the dq axies by parameter
	 * Ethos는 Disable
	 */
#if 0 //by JCNET
	if (PARAM_RAW_VAL(CTRL_POS_SEQ_PLL_ENB)&& CTRL.INV.grid_connected_enb)
#else
	if(1) // 무조건 PLL에 사용된 전압은 선간전압으로 계산된 것을 이용함.. 대신
#endif
	{
//by JCNET.. 이 부분이 사용될 것임.
		TRANSFORM_abc_dq(ACP.INV.va_p, ACP.INV.vb_p, ACP.INV.vc_p, INVCTRL.Eds, INVCTRL.Eqs);
		ACP.INV.va = ACP.INV.va_p;
		ACP.INV.vb = ACP.INV.vb_p;
		ACP.INV.vc = ACP.INV.vc_p;
	}
	else
	{
		TRANSFORM_abc_dq(INVCTRL.Ea, INVCTRL.Eb, INVCTRL.Ec, INVCTRL.Eds, INVCTRL.Eqs);
		ACP.INV.va = INVCTRL.Ea;
		ACP.INV.vb = INVCTRL.Eb;
		ACP.INV.vc = INVCTRL.Ec;
	}
	ACP.INV.vds = (2 * ACP.INV.va - ACP.INV.vb - ACP.INV.vc) * INV_3;
	ACP.INV.vqs = (-ACP.INV.vb + ACP.INV.vc) * INV_SQRT3;

	ACP.INV.vds_p = (2 * ACP.INV.va_p - ACP.INV.vb_p - ACP.INV.vc_p) * INV_3;
	ACP.INV.vqs_p = (-ACP.INV.vb_p + ACP.INV.vc_p) * INV_SQRT3;
	ACP.INV.vds_n = (2 * ACP.INV.va_n - ACP.INV.vb_n - ACP.INV.vc_n) * INV_3;
	ACP.INV.vqs_n = (-ACP.INV.vb_n + ACP.INV.vc_n) * INV_SQRT3;

	ACP.INV.vab = INVCTRL.Eab;
	ACP.INV.vbc = INVCTRL.Ebc;
	ACP.INV.vca = INVCTRL.Eca;
	ACP.INV.vds_ll = (2 * ACP.INV.vab - ACP.INV.vbc - ACP.INV.vca) * INV_3;
	ACP.INV.vqs_ll = (-ACP.INV.vbc + ACP.INV.vca) * INV_SQRT3;

	ACP.PCC.va = EXCTRL.pccEa;
	ACP.PCC.vb = EXCTRL.pccEb;
	ACP.PCC.vc = EXCTRL.pccEc;
	//	TRANSFORM_abc_dq(ACP.PCC.va, ACP.PCC.vb, ACP.PCC.vc, ACP.PCC.vds, ACP.PCC.vqs);
	ACP.PCC.vds = (2 * ACP.PCC.va - ACP.PCC.vb - ACP.PCC.vc) * INV_3;
	ACP.PCC.vqs = (-ACP.PCC.vb + ACP.PCC.vc) * INV_SQRT3;
	ACP.PCC.v_pk = sqrt(ACP.PCC.vds * ACP.PCC.vds + ACP.PCC.vqs * ACP.PCC.vqs);
	/*
	 * Calculate RMS value for INV voltage and INV_PS voltage
	 * to detect asymmetric fault
	 */

	// 주의: fsConV의 pBUF가 작을 경우 FASTSUM_INIT case를 반복 수행하면 buffer overflow가 발생함.
	FASTRMS_AddSample(&FastRmsItems.fsConV[0], INVCTRL.Ea);
	FASTRMS_AddSample(&FastRmsItems.fsConV[1], INVCTRL.Eb);
	FASTRMS_AddSample(&FastRmsItems.fsConV[2], INVCTRL.Ec);
	FASTRMS_AddSample(&FastRmsItems.fsConVp[0], ACP.INV.va_p);
	FASTRMS_AddSample(&FastRmsItems.fsConVp[1], ACP.INV.vb_p);
	FASTRMS_AddSample(&FastRmsItems.fsConVp[2], ACP.INV.vc_p);

	/*
	 * Filtering INV & INV_PS RMS value
	 */
	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2(&INVCTRL.fltFsConV[0], sqrt(FastRmsItems.fsConV[0].fRMS2));
		IIR1_Filter2(&INVCTRL.fltFsConV[1], sqrt(FastRmsItems.fsConV[1].fRMS2));
		IIR1_Filter2(&INVCTRL.fltFsConV[2], sqrt(FastRmsItems.fsConV[2].fRMS2));

		IIR1_Filter2(&INVCTRL.fltFsConVp[0],
				sqrt(FastRmsItems.fsConVp[0].fRMS2));
		IIR1_Filter2(&INVCTRL.fltFsConVp[1],
				sqrt(FastRmsItems.fsConVp[1].fRMS2));
		IIR1_Filter2(&INVCTRL.fltFsConVp[2],
				sqrt(FastRmsItems.fsConVp[2].fRMS2));
	}
	else
	{
		IIR1_Filter2_IS(&INVCTRL.fltFsConV[0],
				sqrt(FastRmsItems.fsConV[0].fRMS2));
		IIR1_Filter2_IS(&INVCTRL.fltFsConV[1],
				sqrt(FastRmsItems.fsConV[1].fRMS2));
		IIR1_Filter2_IS(&INVCTRL.fltFsConV[2],
				sqrt(FastRmsItems.fsConV[2].fRMS2));

		IIR1_Filter2_IS(&INVCTRL.fltFsConVp[0],
				sqrt(FastRmsItems.fsConVp[0].fRMS2));
		IIR1_Filter2_IS(&INVCTRL.fltFsConVp[1],
				sqrt(FastRmsItems.fsConVp[1].fRMS2));
		IIR1_Filter2_IS(&INVCTRL.fltFsConVp[2],
				sqrt(FastRmsItems.fsConVp[2].fRMS2));
	}
	/*------------------------------------------------------------------------------------------------------------------*/
	/* [End Block] GRID, INV Voltage : Sampling -> Pos Seq -> abc2xy -> FS RMS -> IIR Filtering of FS RMS		 	    */
	/*------------------------------------------------------------------------------------------------------------------*/

	/*------------------------------------------------------------------------------------------------------------------*/
	/* [Start Block] GRID Current : Sampling -> abc2xy 																	*/
	/* 130622	    																									*/
	/*------------------------------------------------------------------------------------------------------------------*/
	// 150513 June. 오전 미팅에서 그리드 측 전류가 빠지는 현상이 있으니 일단 인버터 측을 이용하기로 함. V1.00.05
	// 150526 V1.00.06. 턴비 및 Gain을 곱함.
#if 1
	EXCTRL.trIa = INVCTRL.Ia * TR_RATIO * EADC.pAdcTrIa->fGain;
	EXCTRL.trIb = INVCTRL.Ib * TR_RATIO * EADC.pAdcTrIb->fGain;
	EXCTRL.trIc = INVCTRL.Ic * TR_RATIO * EADC.pAdcTrIc->fGain;
#else
	EADC_GetAnalogValue(EADC.pAdcOutIa, &EXCTRL.trIa);
	EADC_GetAnalogValue(EADC.pAdcOutIb, &EXCTRL.trIb);
	EADC_GetAnalogValue(EADC.pAdcOutIc, &EXCTRL.trIc);
#endif	// #ifndef STABLEEN

	EADC_GetAnalogValue(EADC.pAdcCapIa, &ACP.CAP.ia);
	EADC_GetAnalogValue(EADC.pAdcCapIb, &ACP.CAP.ib);
	EADC_GetAnalogValue(EADC.pAdcCapIc, &ACP.CAP.ic);

#ifdef JCNET_EMUL
    ACP.CAP.ia = 1234.0;
    ACP.CAP.ib = 2345.0;
    ACP.CAP.ic = 3456.0;
#endif
#if 0
	EADC_GetAnalogValue(EADC.pAdcBypIa, &EXCTRL.bypIa);
	EADC_GetAnalogValue(EADC.pAdcBypIb, &EXCTRL.bypIb);
	EADC_GetAnalogValue(EADC.pAdcBypIc, &EXCTRL.bypIc);
#else
	EXCTRL.bypIa = ACP.CAP.ia;
	EXCTRL.bypIb = ACP.CAP.ib;
	EXCTRL.bypIc = ACP.CAP.ic;
#endif

	/*
	 * Calculate RMS value for BYP Current
	 */
	FASTRMS_AddSample(&FastRmsItems.fsBypI[0], EXCTRL.bypIa * 0.1);
	FASTRMS_AddSample(&FastRmsItems.fsBypI[1], EXCTRL.bypIb * 0.1);
	FASTRMS_AddSample(&FastRmsItems.fsBypI[2], EXCTRL.bypIc * 0.1);

	/*
	 * Filtering BYP Current rms value
	 */
	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2( &EXCTRL.fltFsBypI[0],	10. * sqrt(FastRmsItems.fsBypI[0].fRMS2));
		IIR1_Filter2( &EXCTRL.fltFsBypI[1],	10. * sqrt(FastRmsItems.fsBypI[1].fRMS2));
		IIR1_Filter2( &EXCTRL.fltFsBypI[2],	10. * sqrt(FastRmsItems.fsBypI[2].fRMS2));
	}
	else
	{
		IIR1_Filter2_IS( &EXCTRL.fltFsBypI[0],	10. * sqrt(FastRmsItems.fsBypI[0].fRMS2));
		IIR1_Filter2_IS( &EXCTRL.fltFsBypI[1],	10. * sqrt(FastRmsItems.fsBypI[1].fRMS2));
		IIR1_Filter2_IS( &EXCTRL.fltFsBypI[2],	10. * sqrt(FastRmsItems.fsBypI[2].fRMS2));
	}

	TRANSFORM_abc_dq(EXCTRL.trIa, EXCTRL.trIb, EXCTRL.trIc, EXCTRL.trIds,
			EXCTRL.trIqs);

#ifdef BYP_EVT_OPERATION
	TRANSFORM_abc_dq(EXCTRL.bypIa, EXCTRL.bypIb, EXCTRL.bypIc, EXCTRL.bypIds,
				EXCTRL.bypIqs);

	TRANSFORM_ROTATE_THETA(EXCTRL.bypIds, EXCTRL.bypIqs, EXCTRL.CosTheta_Byp,
		EXCTRL.SinTheta_Byp, EXCTRL.bypIde, EXCTRL.bypIqe);

	ACP.BYP.Ia = EXCTRL.bypIa;
	ACP.BYP.Ib = EXCTRL.bypIb;
	ACP.BYP.Ic = EXCTRL.bypIc;

	y = (PRM_PCS[SCR_RATED_CURRENT].iValue * PRM_PCS[SCR_RATED_CURRENT].fIncDec) * (PRM_PCS[SCR_CURRENT_LIMIT].iValue * PRM_PCS[SCR_CURRENT_LIMIT].fIncDec) * 0.01;
	z = -y;

	CkeckOC(ACP.BYP.Ia, y, z);
	CkeckOC(ACP.BYP.Ib, y, z);
	CkeckOC(ACP.BYP.Ic, y, z);

	if (ACP.BYP.Ia_OC_Count >= BYP_OC_COUNT_MAX
			|| ACP.BYP.Ib_OC_Count >= BYP_OC_COUNT_MAX
			|| ACP.BYP.Ic_OC_Count >= BYP_OC_COUNT_MAX)	//	BYPASS 측 OC 판단
	{
		bSSW_CB2_Disable_Init = TRUE;

		bByp_Evt_Operating = TRUE;

		CTRL.INV.SEAMLESS.enb_for_BEO = FALSE;	//  GI 전환 비활성화

		bSSW_CB2_Disable = TRUE;

		GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);

		///XXX Check Point STATUS_CB2-->CB4로 수정
		if( !MC_GetStatus(STATUS_CB4) && !FLT_GetEachStatus(FLTH_PM_OVERLOAD) && CTRL_BYP_NormalOK())
		{
			MCB_UpdateCmd(M_CB4_SSW, CB_CMD_ON);
		}

		EVT_Store_NoDup(FLTL_SSW_OC);
	}
	else
	{
		if(bSSW_CB2_Disable == TRUE)
			bSSW_CB2_Disable_Init = FALSE;
	}
#endif

	/*------------------------------------------------------------------------------------------------------------------*/
	// [End Block]
	/*------------------------------------------------------------------------------------------------------------------*/

	/* BYPASS 전압 Freq */
	ThetaDetectRun(&GRID_BYP_THETA, EXCTRL.fltEde_byp.fOut); // R : 8us
	EXCTRL.SinTheta_Byp = sin(GRID_BYP_THETA.fRadian);
	EXCTRL.CosTheta_Byp = cos(GRID_BYP_THETA.fRadian);
	TRANSFORM_ROTATE_THETA(EXCTRL.bypEds, EXCTRL.bypEqs, EXCTRL.CosTheta_Byp,
	EXCTRL.SinTheta_Byp, EXCTRL.bypEde, EXCTRL.bypEqe);

#if 0
	// 16.05.11
	//timer_psec = timer_sec;
	Bool direction = 0; // 현재 0:radian < 0.0 1:else
	timer_sec_r = ZeroCrossingTimeDetect(&GRID_BYP_THETA, 0, &direction);
	/*
	if ( ((timer_psec - timer_sec) > 0.005) || (timer_psec - timer_sec) < -0.005 )
	{
		TIMER_SUB[TIMER_SUB_CNT] = timer_sec - timer_psec;
		TIMER_SUB_CNT +=1;
		if(TIMER_SUB_CNT >= TIMER_SUB_CNT_MAX)
			TIMER_SUB_CNT=0;
		for(i=0, i<TIMER_SUB_CNT_MAX, i++)
			TIMER_SUB_SUM += TIMER_SUB[i];
	}
	timer_sec2 =
	 */
	if(timer_sec_r < 0.0003)
	{
		if( !bTintOneshot_r )
		{
			timer_sec_shadow = timer_sec_r;
			if (GRID_BYP_THETA.fRadian < 0.0)
			{
				GRID_BYP_THETA.ThetaPolarity_r = FALSE;	// 반대로 생각
			}
			else
			{
				GRID_BYP_THETA.ThetaPolarity_r = TRUE;
			}
			bTintOneshot_r = 1;

			//PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
			//CpuTimer0Regs.TPR.all = 20;
			//CpuTimer0Regs.TCR.all = 0x4001;
			CpuTimer0Regs.PRD.all = 15*(timer_sec_r*500000);
			CpuTimer0Regs.TCR.bit.TRB = 1;
			CpuTimer0Regs.TCR.bit.TIE = 1;
		}
	}
	else
	{
		//-CpuTimer0Regs.TCR.bit.TRB = 1;
		bTintOneshot_r = 0;
	}

	timer_sec_s = ZeroCrossingTimeDetect(&GRID_BYP_THETA, 1, &direction);
	if(timer_sec_s < 0.0003)
	{
		if( !bTintOneshot_s )
		{
			if (direction == 0)
			{
				GRID_BYP_THETA.ThetaPolarity_s = FALSE;	// 반대로 생각
			}
			else
			{
				GRID_BYP_THETA.ThetaPolarity_s = TRUE;
			}
			bTintOneshot_s = 1;

			//PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
			//CpuTimer0Regs.TPR.all = 20;
			//CpuTimer0Regs.TCR.all = 0x4001;
			CpuTimer0Regs.PRD.all = 15*(timer_sec_s*500000);
			CpuTimer0Regs.TCR.bit.TRB = 1;
			CpuTimer0Regs.TCR.bit.TIE = 1;
		}
	}
	else
	{
		//-CpuTimer0Regs.TCR.bit.TRB = 1;
		bTintOneshot_s = 0;
	}

	timer_sec_t = ZeroCrossingTimeDetect(&GRID_BYP_THETA, 2, &direction);
	if(timer_sec_t < 0.0003)
	{
		if( !bTintOneshot_t )
		{
			if (direction == 0)
			{
				GRID_BYP_THETA.ThetaPolarity_t = FALSE;	// 반대로 생각
			}
			else
			{
				GRID_BYP_THETA.ThetaPolarity_t = TRUE;
			}
			bTintOneshot_t = 1;

			//PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
			//CpuTimer0Regs.TPR.all = 20;
			//CpuTimer0Regs.TCR.all = 0x4001;
			CpuTimer0Regs.PRD.all = 15*(timer_sec_t*500000);
			CpuTimer0Regs.TCR.bit.TRB = 1;
			CpuTimer0Regs.TCR.bit.TIE = 1;
		}
	}
	else
	{
		//-CpuTimer0Regs.TCR.bit.TRB = 1;
		bTintOneshot_t = 0;
	}
#endif
	// bypass freq W
	//---ThetaDetectRun_Omega(&GRID_BYP_THETA, EXCTRL.bypEde);
	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2( &EXCTRL.fltEde_byp, EXCTRL.bypEde);
		IIR1_Filter2( &EXCTRL.fltEqe_byp, EXCTRL.bypEqe);
		IIR1_Filter2( &EXCTRL.fltFreq_Byp, GRID_BYP_THETA.fFreq);
		IIR1_Filter2( &GRID_BYP_THETA.fltFreqW, GRID_BYP_THETA.fFreqW);
	}
	else
	{
		IIR1_Filter2_IS( &EXCTRL.fltEde_byp, EXCTRL.bypEde);
		IIR1_Filter2_IS( &EXCTRL.fltEqe_byp, EXCTRL.bypEqe);
		IIR1_Filter2_IS( &EXCTRL.fltFreq_Byp, GRID_BYP_THETA.fFreq);
		IIR1_Filter2_IS( &GRID_BYP_THETA.fltFreqW, GRID_BYP_THETA.fFreqW);
	}

	if( fBypOmega_Max < GRID_BYP_THETA.fltFreqW.fOut )
		fBypOmega_Max = GRID_BYP_THETA.fltFreqW.fOut;

	if( fBypOmega_Min > GRID_BYP_THETA.fltFreqW.fOut )
		fBypOmega_Min = GRID_BYP_THETA.fltFreqW.fOut;

	//141201 June
	if( INVERTER.uStatus == SYS_INV_RUN  && (PRM_PCS[CTRL_BYP_V_GRID_TEST_MODE].iValue  & 0x1 ) )
	{
#if 1 // 120 도 급변 시 고장 발생 - (참고: 인증 받을 때 20도 정도 급변 시 고장 발생하게 하려면 OC Level을 낮춰야 함.)
		if( GRID_BYP_THETA.fltFreqW.fOut >= 392 /* 394.0 */ )
		{
			FLT_Raise(FLTH_INV_PAHSE_JUMP);
		}
		else if( GRID_BYP_THETA.fltFreqW.fOut <= 361 /* 359.69 */ )
		{
			FLT_Raise(FLTH_INV_PAHSE_JUMP);
		}
#endif
	}

	/*------------------------------------------------------------------------------------------------------------------*/
	/* [Start Block] GRID, INV Voltage : Freq Detect(PLL) -> xy2dq	-> IIR Filtering of dq and Freq	-> IIR for INV fltq */
	/*------------------------------------------------------------------------------------------------------------------*/
	ThetaDetectRun(&GRIDTHETA, EXCTRL.fltEde.fOut); // R : 8us
	EXCTRL.SinTheta = sin(GRIDTHETA.fRadian);
	EXCTRL.CosTheta = cos(GRIDTHETA.fRadian);
	TRANSFORM_ROTATE_THETA(EXCTRL.pccEds, EXCTRL.pccEqs, EXCTRL.CosTheta,
			EXCTRL.SinTheta, EXCTRL.pccEde, EXCTRL.pccEqe);
	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2( &EXCTRL.fltEde, EXCTRL.pccEde);
		IIR1_Filter2( &EXCTRL.fltEqe, EXCTRL.pccEqe);
		IIR1_Filter2( &EXCTRL.fltFreq, GRIDTHETA.fFreq);
	}
	else
	{
		IIR1_Filter2_IS( &EXCTRL.fltEde, EXCTRL.pccEde);
		IIR1_Filter2_IS( &EXCTRL.fltEqe, EXCTRL.pccEqe);
		IIR1_Filter2_IS( &EXCTRL.fltFreq, GRIDTHETA.fFreq);
	}

	ThetaDetectRun(&CONTHETA, INVCTRL.fltEde.fOut); // R : 8us!
	INVCTRL.SinTheta = sin(CONTHETA.fRadian);
	INVCTRL.CosTheta = cos(CONTHETA.fRadian);

#if 0 //by JCNET 해당 부분 사용하는곳 없음..
	// 14.4.7 -------------------------------------------------------------------
	INVCTRL.RefTheta = CONTHETA.fRadian - PI_6; //by JCNET ?? 필요한것인가??
	if (INVCTRL.RefTheta > PI)
		INVCTRL.RefTheta -= TWO_PI;
	else if (INVCTRL.RefTheta < -PI)
		INVCTRL.RefTheta += TWO_PI;

	INVCTRL.RefSinTheta = sin(INVCTRL.RefTheta);
	INVCTRL.RefCosTheta = cos(INVCTRL.RefTheta);
	// ---------------------------------------------------------------------------
#endif

	TRANSFORM_ROTATE_THETA(INVCTRL.Eds, INVCTRL.Eqs, INVCTRL.CosTheta,
			INVCTRL.SinTheta, INVCTRL.Ede, INVCTRL.Eqe);
	ThetaDetectRun_Omega(&GRIDTHETA_AI, EXCTRL.pccEde);
	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2( &INVCTRL.fltEde, INVCTRL.Ede);
		IIR1_Filter2( &INVCTRL.fltEqe, INVCTRL.Eqe);
		IIR1_Filter2( &INVCTRL.fltFreq, CONTHETA.fFreq);
		IIR1_Filter2( &INVCTRL.fltEqe2nd, INVCTRL.fltEqe.fOut);
		IIR1_Filter2( &GRIDTHETA_AI.fltFreqW, GRIDTHETA_AI.fFreqW);
		// GRID 필터 거치지 않은 freq omega를 구하기 위함 13.10.29
	}
	else
	{
		IIR1_Filter2_IS( &INVCTRL.fltEde, INVCTRL.Ede);
		IIR1_Filter2_IS( &INVCTRL.fltEqe, INVCTRL.Eqe);
		IIR1_Filter2_IS( &INVCTRL.fltFreq, CONTHETA.fFreq);
		IIR1_Filter2_IS( &INVCTRL.fltEqe2nd, INVCTRL.fltEqe.fOut);
		IIR1_Filter2_IS( &GRIDTHETA_AI.fltFreqW, GRIDTHETA_AI.fFreqW);
		// GRID 필터 거치지 않은 freq omega를 구하기 위함 13.10.29
	}
	/*------------------------------------------------------------------------------------------------------------------*/
	/* [END Block] GRID, INV Voltage : Freq Detect(PLL) -> xy2dq	-> IIR Filtering of dq and Freq	-> IIR for GRID fltq*/
	/*------------------------------------------------------------------------------------------------------------------*/

	/*------------------------------------------------------------------------------------------------------------------*/
	/* [Start Block] GRID 필터 거치지 않은 freq omega를 구하기 위함 13.10.29														*/
	/*------------------------------------------------------------------------------------------------------------------*/
	//-ThetaDetectRun_Omega(&GRIDTHETA_AI, EXCTRL.pccEde); // R : 8us
	//-IIR1_Filter2( &GRIDTHETA_AI.fltFreqW , GRIDTHETA_AI.fFreqW);
	/*------------------------------------------------------------------------------------------------------------------*/
	/* [END Block] 																										*/
	/*------------------------------------------------------------------------------------------------------------------*/

	TRANSFORM_abc_dq(INVCTRL.Ia, INVCTRL.Ib, INVCTRL.Ic, INVCTRL.Ids, INVCTRL.Iqs);

//by JCNET	CTRL_INV_DROOP_Control();

//++JCNET
	{
	    extern void SagCompen_Run();
	    SagCompen_Run();
	}

//--JCNET

#if 0
	a = ACP.INV.RATE.Iph_pk * INV_RATED_VOLTAGE / (float) PRM_PCS[GRID_RATED_VOLT].iValue * (float) PRM_PCS[LOAD_OC_LEVEL].iValue * 0.01;
	b = -a;

	CkeckOC(ACP.CAP.ia, a, b);
	CkeckOC(ACP.CAP.ib, a, b);
	//Added
	CkeckOC(ACP.CAP.ic, a, b);

	//	16.07.29	Yang
	//CTRL_BYP_EVT_OPERATION_SCR_OC();

	if (ACP.CAP.ia_OC_Count >= CC_LOAD_OC_COUNT_MAX
			|| ACP.CAP.ib_OC_Count >= CC_LOAD_OC_COUNT_MAX
			|| ACP.CAP.ic_OC_Count >= CC_LOAD_OC_COUNT_MAX)
	{
		FLT_Raise(FLTH_LOAD_OC);
		ACP.CAP.ia_OC_Count = 0;
		ACP.CAP.ib_OC_Count = 0;
		ACP.CAP.ic_OC_Count = 0;
	}
#else
	if( PRM_PCS[LOAD_OC_LEVEL].iValue == 0 )
		PRM_PCS[LOAD_OC_LEVEL].iValue = 100;

	a = ACP.INV.RATE.Iph * INV_RATED_VOLTAGE / (float) PRM_PCS[GRID_RATED_VOLT].iValue * (float) PRM_PCS[LOAD_OC_LEVEL].iValue * 0.01;
	if( (EXCTRL.fltFsBypI[0].fOut > a) || (EXCTRL.fltFsBypI[1].fOut > a) || (EXCTRL.fltFsBypI[2].fOut > a) )
		FLT_Raise(FLTH_LOAD_OC);
#endif

	TRANSFORM_ROTATE_THETA(INVCTRL.Ids, INVCTRL.Iqs, INVCTRL.CosTheta,
			INVCTRL.SinTheta, INVCTRL.Ide, INVCTRL.Iqe);

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2( &INVCTRL.fltIqe, INVCTRL.Iqe);
	}
	else
	{
		IIR1_Filter2_IS( &INVCTRL.fltIqe, INVCTRL.Iqe);
	}
	/*------------------------------------------------------------------------------------------------------------------*/
	/* [Start Block] Detect Grid Fault :																				*/
	/*									Calculate GRID Volt in P.U by using fltVqe/RatedV ->						    */
	/*									Check FRT ENB -> ASYM FLT -> SYM FLT -> None ->								    */
	/*									Init IqeRefDelay or IqeRef Ramp according to detected FRT mode				    */
	/*------------------------------------------------------------------------------------------------------------------*/
#if USE_FASTRTS_LIB_DIV
	EXCTRL.VqPU = div(EXCTRL.fltEqe.fOut, ACP.PCC.RATE.Vph_pk);
#else
	EXCTRL.VqPU = EXCTRL.fltEqe.fOut / ACP.PCC.RATE.Vph_pk;
#endif

	a = PARAM_VAL(CTRL_FRT_ASYNC_LEVEL);

	if (PARAM_RAW_VAL(GRID_FRT_ENB)== 0)	// Ethos: Disable
	{
		INVCTRL.FrtMode = FRT_NORMAL;
		INVCTRL.FrtModeVote = 0;
	}
	else if (INVERTER.uStatus == SYS_INV_RUN)
	{
		b = FRT_MODE_VOTE_LIMIT;

		if ( ((a < INVCTRL.fltFsConVp[0].fOut && a < fabs(INVCTRL.fltFsConV[0].fOut - INVCTRL.fltFsConVp[0].fOut)) ||
				(a < INVCTRL.fltFsConVp[1].fOut && a < fabs(INVCTRL.fltFsConV[1].fOut - INVCTRL.fltFsConVp[1].fOut)) ||
				(a < INVCTRL.fltFsConVp[2].fOut && a < fabs(INVCTRL.fltFsConV[2].fOut - INVCTRL.fltFsConVp[2].fOut)))
		)
		{
			INVCTRL.FrtModeVote -= 1;
			if (INVCTRL.FrtModeVote < -b) INVCTRL.FrtModeVote = -b;
		}
		else if ( ((EXCTRL.VqPU < (float)PARAM_VAL(GRID_UV_LEVEL1)*0.01) || (((float)PARAM_VAL(GRID_OV_LEVEL1)*0.01 < EXCTRL.VqPU && PARAM_RAW_VAL(GRID_FRT_OV_ENB))))
		)
		{
			INVCTRL.FrtModeVote += 1;
			if (INVCTRL.FrtModeVote > b) INVCTRL.FrtModeVote = b;
		}
		else
		{
			if(INVCTRL.FrtModeVote > 0) INVCTRL.FrtModeVote -=1;
			else if(INVCTRL.FrtModeVote < 0) INVCTRL.FrtModeVote +=1;
		}
		/*
		 * Decide FRT MODE according to FrtModeVote count
		 */
		switch(INVCTRL.FrtMode)
		{
			case FRT_NORMAL:
			if (INVCTRL.FrtModeVote > FRT_MODE_VOTE_THRESHOLD) INVCTRL.FrtMode = FRT_SYMMETRIC;
			else if(INVCTRL.FrtModeVote < -FRT_MODE_VOTE_THRESHOLD) INVCTRL.FrtMode = FRT_ASYMMETRIC;
			break;
			case FRT_SYMMETRIC:
			case FRT_ASYMMETRIC:
			if (INVCTRL.FrtModeVote == 0) INVCTRL.FrtMode = FRT_NORMAL;
			break;
		}

	}
	else
	{
		INVCTRL.FrtMode = FRT_NORMAL;
		INVCTRL.FrtModeVote = 0;
	}

	if (PARAM_RAW_VAL(TRC_TRACE_MODE)== TRC_FRT_ENTRY_STOP && INVCTRL.FrtMode)
		TRC_StopTrace();
	/*------------------------------------------------------------------------------------------------------------------*/
	/* [End Block] Detect Grid Fault :																				*/
	/*									Calculate GRID Volt in P.U by using fltVqe/RatedV							    */
	/*									Check FRT ENB -> ASYM FLT -> SYM FLT -> None								    */
	/*									Init IqeRefDelay or IqeRef Ramp according to detected FRT mode				    */
	/*------------------------------------------------------------------------------------------------------------------*/

#if IIVC_ENB
	if (INVERTER.uStatus == SYS_INV_AC_GENERATE && INVERTER.bRequestInvThetaInit)
	{
		if( PRM_PCS[INV_TRANSFORMER].iValue == 0)
		{
			CONTHETA.fRadian = GRIDTHETA.fRadian;
		}
		else
		{
			CONTHETA.fRadian = GRIDTHETA.fRadian - PI_6;
		}
		INVERTER.bRequestInvThetaInit = FALSE;
	}
#endif

	/*if ((PRM_PCS[CTRL_TEST_MODE].iValue == 1 && BUTTON.bTestRun) ||
	 ((INVERTER.uStatus == SYS_INV_RUN || (PRM_PCS[CTRL_TEST_MODE].iValue == 2 && INVERTER.uStatus == SYS_INV_AC_GENERATE))&& INVERTER.bRun &&
	 !FLT_GetHeavyStatus() && !CCI_NoGating() && INVCTRL.FrtMode != FRT_ASYMMETRIC) 	)*/
    if ((PRM_PCS[CTRL_TEST_MODE].iValue == 1 && pButton->bTestRun)
#if 0 //by JCNET
    if ((PRM_PCS[CTRL_TEST_MODE].iValue == 1 && BUTTON.bTestRun)
			|| (INVERTER.uStatus == SYS_INV_ISLANDING && INVERTER.bRun)
			|| (INVERTER.uStatus == SYS_INV_RE_SYNC && INVERTER.bRun)
#endif
			|| (INVERTER.uStatus == SYS_INV_RUN && INVERTER.bRun))
	{
		//13.3.1 TEST 모드에서 Fault 발생 할 경우에도 PWM 출력 정지
		if (FLT_GetHeavyStatus())
			IVC.uStatus = CVC_DISABLE;
		else
		{
			IVC.uStatus = CVC_ENABLE;
#if 0 //by JCNET
			// 150128 June for 전기안전연구원
			if( CTRL.IRT.bPwmOff )
				IVC.uStatus = CVC_DISABLE;
#endif
		}
	}
	//		else if ( (INVERTER.uStatus == SYS_INV_START_SYNC || INVERTER.uStatus == SYS_INV_AC_GENERATE) && INVERTER.bInitInvOn
	else if ((
#if 0 //by JCNET
	        INVERTER.uStatus == SYS_INV_START_SYNC||
#endif
	                 INVERTER.uStatus == SYS_INV_AC_GENERATE))
	//				&& !CCI_NoGating() && INVCTRL.FrtMode != FRT_ASYMMETRIC )
	{
		IVC.uStatus = CVC_INIT_VOLTAGE_BUILD;
	}
	else
		IVC.uStatus = CVC_DISABLE;

#if 0 // by JCNET
#if USE_IDLE_GATE_OFF == 1	// PCC 에서 Idle시 Gate OFF 삭제.
	/*
	 * 130626 Power Control 모드에서 NOGATING 상태이면 CVC DISABLE
	 */
	if( BCState.ccMode == CC_MODE_CP && PCC.uState == GPC_NOGATING /*&& !INVERTER.bInitInvOn*/ && INVERTER.uStatus == SYS_INV_RUN && CTRL.INV.ctrl_mode != PARAM_OPERATION_MODE_IS)
	{
		IVC.uStatus = CVC_DISABLE;
	}
#endif
#endif //by JCNET

	if( IVC.uStatus == CVC_DISABLE )
	{
		if (PRM_PCS[TRC_TRACE_MODE].iValue == TRC_PWM_OFF_STOP)
			TRC_StopTrace();
	}

#if DBUG_MODE == 2
	ACP.INV.THETA.diff_rad = 0;
#else
	if (PRM_PCS[INV_TRANSFORMER].iValue == 0)
		ACP.INV.THETA.diff_rad = (CONTHETA.fRadian) - GRIDTHETA.fRadian;
	else
		ACP.INV.THETA.diff_rad = (CONTHETA.fRadian + PI_6) - GRIDTHETA.fRadian; //- PI_6; Transformer

/*	16.06.29 Re-sync 잘 안되는 부분 때문에 추가	*/
#ifdef RE_SYNC_DEBUG
	if (PRM_PCS[INV_TRANSFORMER].iValue == 0)
		ACP.BYP.THETA.diff_rad = (CONTHETA.fRadian) - GRID_BYP_THETA.fRadian;
		//ACP.BYP.THETA.diff_rad = (GRIDTHETA.fRadian) - GRID_BYP_THETA.fRadian;//GRID_BYP_THETA
	else
		//ACP.BYP.THETA.diff_rad = (CONTHETA.fRadian + PI_6) - GRIDTHETA.fRadian; //- PI_6; Transformer
		ACP.BYP.THETA.diff_rad = (GRIDTHETA.fRadian) - GRID_BYP_THETA.fRadian; //- PI_6; Transformer
#endif

	if (ACP.INV.THETA.diff_rad > PI)
		ACP.INV.THETA.diff_rad += -TWO_PI;
	else if (ACP.INV.THETA.diff_rad < -PI)
		ACP.INV.THETA.diff_rad += TWO_PI;
#endif
	ACP.INV.THETA.diff_time = ACP.INV.THETA.diff_rad * CC_fWeInverse * 1e6;

	CC_CalcInstantPower();
#if 0 //by JCNET
	CTRL_INV_SYNC_Proceed();
	CTRL_BYP_SYNC_Proceed();
	CTRL_GEN_Proceed();
	CTRL_INV_SEAMLESS_Proceed();
#endif


	PCC.fRef_limit = PRM_PCS[BATT_REF_VAL_MODE].iValue / 100.;
	if( PCC.fRef_limit >= 0.8)
		PCC.fRef_limit = 0.8;
	if( PCC.fRef_limit <= 0.1 )
		PCC.fRef_limit = 0.1;

//
//by JCNET
// 정류기 DC Link 전압제어기
//
	switch (IVC.uStatus)
	{
	case CVC_DISABLE:
		PWM_Disable(PWM_CON);
#if 0 // by JCNET
		if (CTRL.INV.operation_mode != PARAM_OPERATION_MODE_IS
				&& !FLT_GetEachStatus(FLTH_GRID_UV_LEVEL1)
				&& !FLT_GetEachStatus(FLTH_GRID_UV_LEVEL2)
				&& !FLT_GetEachStatus(FLTL_GRID_UV))
		{
			/*
			 * TODO
			 * 21.05.31
			 * 협의자 : 황동옥 이사, 신요섭
			 * HLMC 2단계 작업으로 아래와 같이 계통 측(사이리스터 전단)을 정지 상태일 때만 체크하도록 하고(기동시에도 PWM OFF를 하고 있어서 이 시퀀스를 타게 됨),
			 * 추후 3단계 작업은 황동옥 이사가 개발한 사이리스터 제어보드를 지원해야 하고, SCR전 후 단 전압을 보고 SCR고장 조건을 판단하여 중고장을 발생시켜야 함.
			 *
			 * [사이리스터 후단(PCS측)]  -> 	[사이리스터 전단(계통)]
			 * pccEa 				-> 	bypEa
			 */

			if (PRM_PCS[CTRL_TEST_MODE].iValue != 1) // 테스트 모드 1에서는 fault발생 X
			{
				// GRID CB가 ON 일 경우에 체크 하도록 수정.
				if( PRM_PCS[GRID_GATE_WAY_ENB].iValue == 1 )
				{
					if( CTRL_BYP_NormalOK() && INVERTER.uStatus == SYS_INV_STOP /*-210531 CTRL_INV_SEAMLESS_PccNormalOK()*/ )
						PHS_Check(&GPC, EXCTRL.bypEa, EXCTRL.bypEb, EXCTRL.bypEc, FLTH_INV_PHASE);
				}
				else
				{
					{
						if( CTRL_BYP_NormalOK() && INVERTER.uStatus == SYS_INV_STOP  /*-210531 CTRL_INV_SEAMLESS_PccNormalOK()*/ )
							PHS_Check(&GPC, EXCTRL.bypEa, EXCTRL.bypEb, EXCTRL.bypEc, FLTH_INV_PHASE);
					}
				}
			}

		}
#endif
		// DQ 제어기 초기화
		ConverterInitialize(); //R:12us

		// PR 제어기 초기화
		/* 계통 연계 Idle 일 때만 0로 만듬, AC GEN 단계에서 과도 발생. */
		if( INVERTER.uStatus == SYS_INV_RUN && ( CTRL.INV.operation_mode == PARAM_OPERATION_MODE_GC || CTRL.INV.operation_mode == PARAM_OPERATION_MODE_AUTO_IS) )
		{
			ACP.INV.ids = 0;
			ACP.INV.iqs = 0;
			CTRL_INV_PRC_Run();
		}
		else
		{
			CTRL_INV_PRC_Initialize(&CTRL.INV.PRC_Ids);
			CTRL_INV_PRC_Initialize(&CTRL.INV.PRC_Iqs);
			CTRL_INV_PRC_Initialize(&CTRL.INV.PRC_Vds);
			CTRL_INV_PRC_Initialize(&CTRL.INV.PRC_Vqs);

			CTRL.INV.vds_ref = 0.;
			CTRL.INV.vqs_ref = 0.;
			CTRL.INV.vds_ref_vi = 0.;
			CTRL.INV.vqs_ref_vi = 0.;
			CTRL.INV.vds_ref_vi_vuc = 0.;
			CTRL.INV.vqs_ref_vi_vuc = 0.;

			CTRL.INV.ids_ref_lim = 0.;
			CTRL.INV.iqs_ref_lim = 0.;

			CTRL.INV.va_ref = 0.;
			CTRL.INV.vb_ref = 0.;
			CTRL.INV.vc_ref = 0.;

			CTRL.INV.va_ref_ad = 0.;
			CTRL.INV.vb_ref_ad = 0.;
			CTRL.INV.vc_ref_ad = 0.;

			INVCTRL.VaRef = 0.;
			INVCTRL.VbRef = 0.;
			INVCTRL.VcRef = 0.;

			CTRL.INV.PQC.q_ref = 0.;
			CTRL.INV.PQC.p_ref = 0.;

			//+ 141105
			CTRL.INV.ids_ref = 0;
			CTRL.INV.iqs_ref = 0;

			PICON_Initialize(&CTRL.INV.PQC.PI_Q.CORE);
			PICON_Initialize(&CTRL.INV.PQC.PI_P.CORE);

			CTRL_INV_VI_Initialize();

		}


		//?XXX PCC.uIdleTime = 0;
		break;

	case CVC_ENABLE:
#if 1
		if (CTRL.INV.ctrl_mode != PARAM_VSI_PR)
		{
			if (BCState.srcOder == SRC_REMOTE)
				CC_BCRemotePowerFlowCheck();
			else
			{
#if 0 //by JCNET
				CC_BCLocalPowerFlowCheck();
#else
		        BCState.powerFlow = PFLOW_CHARGE;
#endif
			}

			if (BCState.bNeedInit)
			{
				BCState.bNeedInit = FALSE;
				// STANDBY 모드에서 깨어날 경우.
				//- 141023 CVC Disable 에서 이미 초기화 하므로 필요 없음. ConverterInitialize();
			}

			if( PRM_PCS[INV_TRANSFORMER].iValue == 1 )
				INVCTRL.fEqeCalcVoltage = PARAM_VAL(GRID_RATED_VOLT) * 0.81649 * TR_RATIO; // 0.81 = PCC_v / 1.732 * 1.414
			else
				INVCTRL.fEqeCalcVoltage = PARAM_VAL(GRID_RATED_VOLT) * 0.81649;

			//151125 Capacity 성분 보상. TR 모델에서만 해당(100kW는 C가 80이라 임시로 100kW장비만 적용)
			if( PRM_PCS[INV_TRANSFORMER].iValue == 1 && PRM_PCS[INV_CAPACITY].iValue <= 125 )
			{
				if( PRM_PCS[BYP_MODE].iValue == 20 || PRM_PCS[BYP_MODE].iValue == 30 )
					PCC.fRefQ -= 4.5 * ACP.INV.RATE.Omega * 80e-6 * (INVCTRL.fEqeCalcVoltage*INVCTRL.fEqeCalcVoltage);
				else
					PCC.fRefQ -= 4.5 * ACP.INV.RATE.Omega * 80e-6 * (INVCTRL.fltEqe.fOut*INVCTRL.fltEqe.fOut);
			}

			if (BCState.powerFlow == PFLOW_STANDBY)
			{
//++JCNET
//					INVCTRL.IqeRef = RAMP_Change(ICC.pRAMPOutput, 0);
					INVCTRL.IqeRef = RAMP_Change(IVC.pRAMPOutput,0); // JCNET RAMP 초기화 0으로..
                    INVCTRL.IdeRef = 0; // ?? ACP.INV.RATE.Iph_pk * INVCTRL.fCurrentLimit * 0.01; // d 축 전류지령으로 바꿈..
//--
			}
			else
			{
				switch (BCState.ccMode)
				{
				case CC_MODE_CV:
					CC_Mode_CV(); 		  		// ret IVC.fPIOut
//JCNET
#if 0
					INVCTRL.IqeRef = IVC.fPIOut;
					INVCTRL.IdeRef = 0;
#else
					INVCTRL.IqeRef = IVC.iqe_ref; // PI out을 RAMP 거친 값으로 변경
					INVCTRL.IdeRef = 0; // 뒤에 반영되어 있음..
#endif
					break;
				}
			}
#else
			c_int11_CCCV_before(); 		 // ret ICC.fErr
			INVCTRL.IqeRef = RAMP_Change(ICC.pRAMPOutput, IVC.fPIOut);
#endif



#if COSPHI_ENB == 1
			COSPHI_AdjustPQPower(&Cosphi, &INVCTRL);

#else
			/* 130305 CCOSPHI_AdjustPQPower가 사용되면 하단 코드는 삭제 해야 함
			 * 140521 PCC 에서 계산된 Ide 값에 더함. Anti-islanding 시험 시 사용할 수 있다고 함.
			 */
			//-140521 INVCTRL.IdeRef += (float)PRM_PCS[CTRL_REACTIVE_POWER].iValue* INVERTER.RATE.fCurrent * SQRT2 * 0.01;
			INVCTRL.IdeRef += (float) PRM_PCS[CTRL_REACTIVE_POWER].iValue * ACP.INV.RATE.Iph_pk * 0.01;

			//-}
#endif
		}/* if (CTRL.INV.ctrl_mode != PARAM_VSI_PR) */


#if 0 /* Apply Idq Ramp - deleted */
		if (INVCTRL.CCI_Status == CCI_DELAY)
			INVCTRL.IqeRef = RAMP_Change(INVCTRL.pRampIqeRefDelay, INVCTRL.IqeRef);
		else
			INVCTRL.IqeRef = RAMP_Change(INVCTRL.pRampIqeRef, INVCTRL.IqeRef);
#endif

//
//by JCNET
//
//  정류기 DQ 전류 제어기 (1/2)
		if (CTRL.INV.ctrl_mode == PARAM_CSI_DQ)
		{
			//PICON_SetActualOutM(CVC.pPI, CON.IqeRef);
			/* Synchronous -> Stationary */
			TRANSFORM_ROTATE_THETA_INVERSE(INVCTRL.IdeRef, INVCTRL.IqeRef,
					INVCTRL.CosTheta, INVCTRL.SinTheta, INVCTRL.IdsRef,
					INVCTRL.IqsRef);

			/* Stationary -> 3 Phases For display only */
			TRANSFORM_dq_abc(INVCTRL.IdsRef, INVCTRL.IqsRef, INVCTRL.IaRef,
					INVCTRL.IbRef, INVCTRL.IcRef);

			// 140516
			INVCTRL.Li = (float) PRM_PCS[CTRL_LI].iValue * CC_MICRO_UNIT;
			//-INVCTRL.Li = (float) FILTER_INDUCTANCE_LI * CC_MICRO_UNIT;

			/* D-Axis Synchronous PI Controller [AC Current Controller] (5)------------------------------*/
			INVCTRL.IdeErr = INVCTRL.IdeRef - INVCTRL.Ide;
			INVCTRL.VdeIntegOut += INVCTRL.KiTCC * INVCTRL.IdeErr;

			//		a = INVERTER.RATE.fCurrent;// * 1.;
			a = ACP.INV.RATE.Iph;

			if (INVCTRL.VdeIntegOut > a)
				INVCTRL.VdeIntegOut = a;
			if (INVCTRL.VdeIntegOut < -a)
				INVCTRL.VdeIntegOut = -a;

			INVCTRL.VdePIOut = INVCTRL.VdeIntegOut
					+ INVCTRL.KpCC * INVCTRL.IdeErr;
			INVCTRL.VdeEmfComp = CC_fWe * INVCTRL.Li * INVCTRL.IqeRef;
			INVCTRL.VdeRef = -INVCTRL.VdePIOut + INVCTRL.VdeEmfComp;

			/* Q-Axis Synchronous PI Controller [AC Current Controller] (5)-------------------------------*/
			INVCTRL.IqeErr = INVCTRL.IqeRef - INVCTRL.Iqe;
			INVCTRL.VqeIntegOut += INVCTRL.KiTCC * INVCTRL.IqeErr;

			if (INVCTRL.VqeIntegOut > a)
				INVCTRL.VqeIntegOut = a;
			if (INVCTRL.VqeIntegOut < -a)
				INVCTRL.VqeIntegOut = -a;
			INVCTRL.VqePIOut = INVCTRL.VqeIntegOut
					+ INVCTRL.KpCC * INVCTRL.IqeErr;
			INVCTRL.VqeEmfComp = CC_fWe * INVCTRL.Li * INVCTRL.IdeRef;

			//		if( PRM_PCS[SYS_REV_1].iValue == 2 )
			//			INVCTRL.VqeRef = -INVCTRL.VqePIOut - INVCTRL.VqeEmfComp + 302.0669;
			//		else
			INVCTRL.VqeRef = -INVCTRL.VqePIOut - INVCTRL.VqeEmfComp
					+ INVCTRL.fltEqe.fOut; // Original

#if 0 //by JCNET
			if (PRM_PCS[CTRL_TEST_MODE].iValue == 1 && BUTTON.bTestRun)
#else
	        if (PRM_PCS[CTRL_TEST_MODE].iValue == 1 && pButton->bTestRun)
#endif
			{
				INVCTRL.VdeRef = 0.;
				INVCTRL.VqeRef = PRM_PCS[CTRL_TEST_PWM_VQE_REF].iValue; // TEST PWM RUN
			}

			//#if INV_V_USE_LTL == 1
			//			TRANSFORM_ROTATE_THETA_INVERSE(INVCTRL.VdeRef, INVCTRL.VqeRef, INVCTRL.RefCosTheta, INVCTRL.RefSinTheta, INVCTRL.VdsRef, INVCTRL.VqsRef);
			//#else
			/* Synchronous -> Stationary */
			TRANSFORM_ROTATE_THETA_INVERSE(INVCTRL.VdeRef, INVCTRL.VqeRef,
					INVCTRL.CosTheta, INVCTRL.SinTheta, INVCTRL.VdsRef,
					INVCTRL.VqsRef);
			//#endif

			/* Stationary -> 3 Phases */
			TRANSFORM_dq_abc(INVCTRL.VdsRef, INVCTRL.VqsRef, INVCTRL.VaRef,
					INVCTRL.VbRef, INVCTRL.VcRef);

		}
		else if (CTRL.INV.ctrl_mode == PARAM_VSI_PR	|| CTRL.INV.ctrl_mode == PARAM_CSI_PR)
		{
			//		c_int11_CCCV_before();
			//		INVCTRL.IqeRef = RAMP_Change(ICC.pRAMPOutput, IVC.fPIOut);

			//		CC_Mode_CV(); 		  		// ret IVC.fPIOut
			//		INVCTRL.IqeRef = RAMP_Change(ICC.pRAMPOutput, IVC.fPIOut); // ICC 출력 RAMP 사용해야 함.

			CTRL_INV_PRC_Run();
		}

		CTRL_INV_PWM_Generate();
		PWM_SetOnTime(PWM_CON, &INVCTRL.PwmOnTime);

		if (PRM_PCS[TRC_TRACE_MODE].iValue == TRC_INV_CTRL_ENB_STOP)
			TRC_StopTrace();
		// Region CVC ENABLE
		break;

	case CVC_INIT_VOLTAGE_BUILD:

		if (INVERTER.uStatus == SYS_INV_AC_GENERATE
				|| INVERTER.uStatus == SYS_INV_START_SYNC) // AC가 생성된 후 체크 하도록 한다. AC State 진입 후 바로 Fault발생하는 횟수가 잦다.
		{
			PHS_Check(&IPC, INVCTRL.Ea, INVCTRL.Eb, INVCTRL.Ec, FLTH_INV_PHASE);
		}

		CTRL_INV_PRC_Run();
		CTRL_INV_PWM_Generate();
		PWM_SetOnTime(PWM_CON, &INVCTRL.PwmOnTime);

		if (PRM_PCS[TRC_TRACE_MODE].iValue == TRC_INIT_INV_START)
			TRC_StopTrace();
		break;
	}
//by JCNET
//SagCompensator(Inverter) PWM Generate
	{
	    extern void CTRL_SCC_PWM_Generate();
	    CTRL_SCC_PWM_Generate();
	    PWM_SetOnTime(PWM_SCC, &SCCTRL.PwmOnTime);
	}
	if (CONTHETA.bRMSChangeEnable)
	{
		RMS_Change(pRMSGridV);
		RMS_Change(pRMSConE);
		RMS_Change(pRMSConI);
		RMS_Change(pRMSGridEp);
		RMS_Change(pRMSGridI);
		RMS_Change(pRMSBypassV);
		RMS_Change(pRMSBypassI);
		CONTHETA.bRMSChangeEnable = FALSE;
	}

	RMS_GetM(pRMSConE, 0, INVCTRL.Ea);
	RMS_GetM(pRMSConE, 1, INVCTRL.Eb);
	RMS_GetM(pRMSConE, 2, INVCTRL.Ec);

	RMS_GetM(pRMSConI, 0, INVCTRL.Ia);
	RMS_GetM(pRMSConI, 1, INVCTRL.Ib);
	RMS_GetM(pRMSConI, 2, INVCTRL.Ic);

	RMS_GetM(pRMSGridV, 0, EXCTRL.pccEa);
	RMS_GetM(pRMSGridV, 1, EXCTRL.pccEb);
	RMS_GetM(pRMSGridV, 2, EXCTRL.pccEc);
	RMS_GetM(pRMSGridEp, 0, EXCTRL.Eap);
	RMS_GetM(pRMSGridEp, 1, EXCTRL.Ebp);
	RMS_GetM(pRMSGridEp, 2, EXCTRL.Ecp);
	//13.3.27--------------------------------------
	RMS_GetM(pRMSGridI, 0, EXCTRL.trIa);
	RMS_GetM(pRMSGridI, 1, EXCTRL.trIb);
	RMS_GetM(pRMSGridI, 2, EXCTRL.trIc);
	//----------------------------------------------

	RMS_GetM(pRMSBypassV, 0, EXCTRL.bypEa);
	RMS_GetM(pRMSBypassV, 1, EXCTRL.bypEb);
	RMS_GetM(pRMSBypassV, 2, EXCTRL.bypEc);

	RMS_GetM(pRMSBypassI, 0, EXCTRL.bypIa);
	RMS_GetM(pRMSBypassI, 1, EXCTRL.bypIb);
	RMS_GetM(pRMSBypassI, 2, EXCTRL.bypIc);

	// 150128 June
//	if( INVERTER.uStatus == SYS_INV_ISLANDING && CTRL.IRT.bPwmOff  )
//	{
//		// 아일랜딩 상황에서 OC로 인해 PWM OFF 상태일 경우에는 rms를 계산하지 않음.
//		// Inv UV 발생 막기 위함.
//	}
//	else
//	{
		RMS_AddSampleAll(pRMSConE);
//	}

	RMS_AddSampleAll(pRMSConI);
	RMS_AddSampleAll(pRMSGridV);
	RMS_AddSampleAll(pRMSGridEp);
	RMS_AddSampleAll(pRMSGridI); //13.3.27
	RMS_AddSampleAll(pRMSBypassV); //13.3.27
	RMS_AddSampleAll(pRMSBypassI);	//16.08.10
	//*/

	EADC_GetAnalogValue(EADC.pAdcBatI, &BATCTRL.I);
	INVCTRL.Pout = -INVCTRL.Iqe * INVCTRL.Eqe * 1.5;

#ifdef JCNET_EMUL
	BATCTRL.I = 1000.0;
	INVCTRL.Pout = 1500.0;
#endif
	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		IIR1_Filter2( &BATCTRL.fltI, BATCTRL.I);
		IIR1_Filter2( &BATCTRL.fltI2, BATCTRL.fltI.fOut);
		IIR1_Filter2( &INVCTRL.fltPout, INVCTRL.Pout);
		IIR1_Filter2( &INVCTRL.fltPout2nd, INVCTRL.fltPout.fOut);
	}
	else
	{
		IIR1_Filter2_IS( &BATCTRL.fltI, BATCTRL.I);
		IIR1_Filter2_IS( &BATCTRL.fltI2, BATCTRL.fltI.fOut);
		IIR1_Filter2_IS( &INVCTRL.fltPout, INVCTRL.Pout);
		IIR1_Filter2_IS( &INVCTRL.fltPout2nd, INVCTRL.fltPout.fOut);
	}

	if (fabs(BATCTRL.fltI.fOut) >= BATCTRL.OCLevel)
		FLT_Raise(FLTH_BATT_OC);

#if 0 //by JCNET
	if(INVERTER.uStatus == SYS_INV_TEST_MODE)
	{
		CTRL.INV.ctrl_mode = PARAM_CSI_DQ;

		if( CTRL.INV.filter_mode != FILTER_MODE_GC_SwF_1 || CC_uPeriod != PRM_PCS[CTRL_CC_PERIOD].iValue )
		{
			CTRL.INV.filter_mode = FILTER_MODE_GC_SwF_1;
			CTRL.INV.ctrl_mode_change_enb = ON;
			CTRL_INV_PRC_ApplyGC_GI_PI_Gain();
		}
	}
	else if(INVERTER.uStatus == SYS_INV_RUN)
	{
		if(CTRL.INV.SEAMLESS.pcc_blackout_enb )
		{
			CTRL.INV.ctrl_mode = PARAM_VSI_PR;

			if( CTRL.INV.filter_mode != FILTER_MODE_IS_SwF_2 || CC_uPeriod != PRM_PCS[IS_CCP].iValue  )
			{
				CTRL.INV.filter_mode = FILTER_MODE_IS_SwF_2;
				CTRL.INV.ctrl_mode_change_enb = ON;
				CTRL_INV_PRC_ApplyGC_GI_PI_Gain();
			}
		}
		else
		{
			CTRL.INV.ctrl_mode = PARAM_CSI_PR;

			if( CTRL.INV.filter_mode != FILTER_MODE_GC_SwF_1 || CC_uPeriod != PRM_PCS[CTRL_CC_PERIOD].iValue )
			{
				CTRL.INV.filter_mode = FILTER_MODE_GC_SwF_1;
				CTRL.INV.ctrl_mode_change_enb = ON;
				CTRL_INV_PRC_ApplyGC_GI_PI_Gain();
			}
		}
	}
	else if(INVERTER.uStatus == SYS_INV_ISLANDING)
	{
		CTRL.INV.ctrl_mode = PARAM_VSI_PR;

		if( CTRL.INV.filter_mode != FILTER_MODE_IS_SwF_2 || CC_uPeriod != PRM_PCS[IS_CCP].iValue  )
		{
			CTRL.INV.filter_mode = FILTER_MODE_IS_SwF_2;
			CTRL.INV.ctrl_mode_change_enb = ON;
			CTRL_INV_PRC_ApplyGC_GI_PI_Gain();
		}
	}
#endif

#if 1
	if (CTRL.INV.ctrl_mode_change_enb)
	{
		if (CTRL.INV.filter_mode == FILTER_MODE_IS_SwF_2)
		{
			CC_uPeriod = PRM_PCS[IS_CCP].iValue;
		}
		else
		{
			CC_uPeriod = PRM_PCS[CTRL_CC_PERIOD].iValue;
		}

		if (CC_uPeriod < CC_PERIOD_MIN)
			CC_uPeriod = CC_PERIOD_MIN;
		if (CC_uPeriod > CC_PERIOD_MAX)
			CC_uPeriod = CC_PERIOD_MAX;

		CC_tsCC = CC_uPeriod * CC_MICRO_UNIT;
		CC_tsSample = CC_tsCC;
#if DOUBLE_CONTROL == 1
		CC_tsSample = CC_tsCC * 0.5;
#endif
		CC_tsSampleInv = 1 / CC_tsSample;

		// LF28 PWM 에서 Up-Down Mdoe인 경우 200usec가 되려면...
		INVCTRL.PwmOnTime.uPeriod = (CC_uPeriod * PWM_CLK) >> 1; // div 2

		PWM_SetPeriod(PWM_CON, &INVCTRL.PwmOnTime);

		CC_fDTheta = CC_fWe * CC_tsSample;
		INVCTRL.KiTCC = INVCTRL.KiCC * CC_tsSample;

		PLL_K.fIT = (float) PRM_PCS[CTRL_PLL_I_GAIN].iValue
				* (float) PRM_PCS[CTRL_PLL_I_GAIN].fIncDec * CC_tsSample;

		CVC_UpdateParameter();

		IVC.K.fIT = (float) PRM_PCS[CTRL_VC_I_GAIN].iValue
				* (float) PRM_PCS[CTRL_VC_I_GAIN].fIncDec * CC_tsSample;

		ICC.K.fIT = (float) PRM_PCS[BATT_ICC_I_GAIN].iValue
				* (float) PRM_PCS[BATT_ICC_I_GAIN].fIncDec * CC_tsSample;

		PCC.K.fIT = (float) PRM_PCS[BATT_PCC_I_GAIN].iValue
				* (float) PRM_PCS[BATT_PCC_I_GAIN].fIncDec * CC_tsSample;
		PCC.K_Q.fIT = (float) PRM_PCS[BATT_PCC_I_GAIN].iValue
				* (float) PRM_PCS[BATT_PCC_I_GAIN].fIncDec * CC_tsSample;

		FASTRMS_UpdateParameter();

		CTRL_INV_DROOP_UpdateRamp();
		CTRL_INV_PQC_UpdateParameterRamp();
		CTRL_INV_PQC_UpdateParameterPIGain();
		CTRL_INV_VUC_UpdateParameterPIGain();

		CTRL.INV.ctrl_mode_change_enb = OFF;
	}
#endif

	//-140630 BATCTRL.fPowerFilterd_1st = BATCTRL.fltI2.fOut * BATCTRL.fltDCBattV_2nd.fOut;
#if 0 //by JCNET
	BATCTRL.fPowerFilterd_1st = BATCTRL.fltI.fOut * BATCTRL.fltDCBattV.fOut;
#else
    BATCTRL.fPowerFilterd_1st = BATCTRL.fltI.fOut * BATCTRL.fltDCLinkV.fOut;
#endif
	BATCTRL.fPower = BATCTRL.I * BATCTRL.DCLinkV;

	/*
	 * Termination routine
	 */
	INVCTRL.prevFrtMode = INVCTRL.FrtMode;

	//-PWM_SetOnTime(PWM_PEBTEMP, &pwmPEBTemp);

	if (iCallOrder == 0) // Double Control 시 2회 호출되지 않도록.
	{
		TRC_UpdateTraceChannel();
		//-141225
#if EXTRACE_USE_SPI_FLASH == 1
		TF_UpdateData();
#endif
	}

}


/*
 * call : MCU_APP_Create()
 */
#pragma CODE_SECTION (CC_AddInfoNode, "dp_ctrl")
void CC_AddInfoNode(void)
{
#if EXTRACE_USE_SPI_FLASH == 1 //-141225
	TF_Create();
	TF_DataHook(0, TRC_00_DCLINK_VDC, &BATCTRL.DCBattV);
	TF_DataHook(1, TRC_03_BATT_I_FLT, &BATCTRL.I);
	TF_DataHook(2, TRC_20_CON_EA, &INVCTRL.Ea);
	TF_DataHook(3, TRC_21_CON_EB, &INVCTRL.Eb);
	TF_DataHook(4, TRC_22_CON_EC, &INVCTRL.Ec);
	TF_DataHook(5, TRC_23_CON_IA, &INVCTRL.Ia);
	TF_DataHook(6, TRC_24_CON_IB, &INVCTRL.Ib);
	TF_DataHook(7, TRC_25_CON_IC, &INVCTRL.Ic);
#endif

	TRC_AddNode(TRC_00_DCLINK_VDC, TRC_FLOAT, &BATCTRL.DCLinkV);
	TRC_AddNode(TRC_01_DCLINK_VDC_FLT, TRC_FLOAT, &BATCTRL.fltDCLinkV2nd.fOut);
	TRC_AddNode(TRC_02_BATT_I, TRC_FLOAT, &BATCTRL.I);
	TRC_AddNode(TRC_03_BATT_I_FLT, TRC_FLOAT, &BATCTRL.fltI2.fOut);
	TRC_AddNode(TRC_04_BATT_VDC, TRC_FLOAT, &BATCTRL.DCBattV);
	TRC_AddNode(TRC_05_BATT_VDC_FLT, TRC_FLOAT, &BATCTRL.fltDCBattV_2nd.fOut);
	TRC_AddNode(TRC_06_, TRC_FLOAT, &BATCTRL.fltDCBattVForCC.fOut);

	TRC_AddNode(TRC_09_PWR_CONTROL_EQS, TRC_FLOAT, &EXCTRL.pccEqs);
	TRC_AddNode(TRC_10_PWR_CONTROL_EDS, TRC_FLOAT, &EXCTRL.pccEds);
	TRC_AddNode(TRC_11_TR_I_IQS, TRC_FLOAT, &EXCTRL.trIqs);
	TRC_AddNode(TRC_12_TR_I_IDS, TRC_FLOAT, &EXCTRL.trIds);
	//-TRC_AddNode(TRC_13_, TRC_FLOAT, &INVCTRL.fltPout2nd.fOut);
	//-TRC_AddNode(TRC_14_, TRC_FLOAT, &EXCTRL.fltAcPowerP_2nd.fOut);

	TRC_AddNode(TRC_13_, TRC_FLOAT, &(CTRL.INV.DROOP.w_ref));
	TRC_AddNode(TRC_14_, TRC_FLOAT, &(CTRL.INV.DROOP.v_ref));
	TRC_AddNode(TRC_15_, TRC_FLOAT, &(ACP.INV.p_f_lim));
	TRC_AddNode(TRC_16_, TRC_FLOAT, &(ACP.INV.q_f_lim));

	TRC_AddNode(TRC_17_IVC_REF, TRC_FLOAT, &IVC.fRef_command);
	TRC_AddNode(TRC_18_ICC_REF, TRC_FLOAT, &ICC.fRef);

	TRC_AddNode(TRC_20_CON_EA, TRC_FLOAT, &INVCTRL.Ea);
	TRC_AddNode(TRC_21_CON_EB, TRC_FLOAT, &INVCTRL.Eb);
	TRC_AddNode(TRC_22_CON_EC, TRC_FLOAT, &INVCTRL.Ec);

	TRC_AddNode(TRC_23_CON_IA, TRC_FLOAT, &INVCTRL.Ia);
	TRC_AddNode(TRC_24_CON_IB, TRC_FLOAT, &INVCTRL.Ib);
	TRC_AddNode(TRC_25_CON_IC, TRC_FLOAT, &INVCTRL.Ic);

	TRC_AddNode(TRC_26_CON_EA_OFFSET, TRC_FLOAT, &(INVCTRL.Eab));
	TRC_AddNode(TRC_27_CON_EB_OFFSET, TRC_FLOAT, &(INVCTRL.Ebc));
	TRC_AddNode(TRC_28_CON_EC_OFFSET, TRC_FLOAT, &(INVCTRL.Eca));

	TRC_AddNode(TRC_29_CON_EDE_FLT, TRC_FLOAT, &INVCTRL.fltEde.fOut);
	TRC_AddNode(TRC_30_CON_EQE_FLT, TRC_FLOAT, &INVCTRL.fltEqe.fOut);
#if FREQ_MODE_ENB
#else
	TRC_AddNode(TRC_31_CON_FREQ, TRC_FLOAT, &CONTHETA.fFreq);
#endif
	TRC_AddNode(TRC_32_CON_THETA, TRC_FLOAT, &CONTHETA.fRadian);

	/*
	 TRC_AddNode( TRC_33_CON_POSITIVE_RMS_VA, TRC_FLOAT, &GRID_ARG.fVoltPositive[0]);
	 TRC_AddNode( TRC_34_CON_POSITIVE_RMS_VB, TRC_FLOAT, &GRID_ARG.fVoltPositive[1]);
	 TRC_AddNode( TRC_35_CON_POSITIVE_RMS_VC, TRC_FLOAT, &GRID_ARG.fVoltPositive[2]);
	 */
	TRC_AddNode(TRC_33_CON_POSITIVE_RMS_VA, TRC_FLOAT,
			&(INVCTRL.fltFsConVp[0].fOut));
	TRC_AddNode(TRC_34_CON_POSITIVE_RMS_VB, TRC_FLOAT,
			&(INVCTRL.fltFsConVp[1].fOut));
	TRC_AddNode(TRC_35_CON_POSITIVE_RMS_VC, TRC_FLOAT,
			&(INVCTRL.fltFsConVp[2].fOut));

	/*
	TRC_AddNode(TRC_36_CON_POSITIVE_INSTANT_VA, TRC_FLOAT, &(ACP.INV.va_p));
	TRC_AddNode(TRC_37_CON_POSITIVE_INSTANT_VB, TRC_FLOAT, &(ACP.INV.vb_p));
	TRC_AddNode(TRC_38_CON_POSITIVE_INSTANT_VC, TRC_FLOAT, &(ACP.INV.vc_p));
	*/

	TRC_AddNode(TRC_36_CON_POSITIVE_INSTANT_VA, TRC_FLOAT, &(GenBlock.pcsDecreasePowerP));
	TRC_AddNode(TRC_37_CON_POSITIVE_INSTANT_VB, TRC_FLOAT, &(GenBlock.pcsDecreasePowerQ));
	TRC_AddNode(TRC_38_CON_POSITIVE_INSTANT_VC, TRC_FLOAT, &(GenBlock.diffThetaRadian));

	/*
	 TRC_AddNode( TRC_36_GRID_ALLPASS_VA, TRC_FLOAT, &GRID.EaDelay90);
	 TRC_AddNode( TRC_37_GRID_ALLPASS_VB, TRC_FLOAT, &GRID.EbDelay90);
	 TRC_AddNode( TRC_38_GRID_ALLPASS_VC, TRC_FLOAT, &GRID.EcDelay90);
	 */

	TRC_AddNode(TRC_40_GRID_EA, TRC_FLOAT, &EXCTRL.pccEa);
	TRC_AddNode(TRC_41_GRID_EB, TRC_FLOAT, &EXCTRL.pccEb);
	TRC_AddNode(TRC_42_GRID_EC, TRC_FLOAT, &EXCTRL.pccEc);
	TRC_AddNode(TRC_43_GRID_EDE, TRC_FLOAT, &EXCTRL.pccEde);
	TRC_AddNode(TRC_44_GRID_EQE, TRC_FLOAT, &EXCTRL.pccEqe);
	TRC_AddNode(TRC_45_GRID_EDE_FLT, TRC_FLOAT, &EXCTRL.fltEde.fOut);
	TRC_AddNode(TRC_46_GRID_EQE_FLT, TRC_FLOAT, &EXCTRL.fltEqe.fOut);
	TRC_AddNode(TRC_47_GRID_THETA, TRC_FLOAT, &GRIDTHETA.fRadian);

	//TRC_AddNode(TRC_48_GRID_PA, TRC_FLOAT, &GRIDTHETA_AI.fFreqW);

	TRC_AddNode(TRC_48_GRID_PA, TRC_FLOAT, &PCC.fRefP);
	TRC_AddNode(TRC_49_GRID_PB, TRC_FLOAT, &PCC.fRefQ);
	TRC_AddNode(TRC_50_GRID_PC, TRC_FLOAT, &PCC.fPowerQ);
	TRC_AddNode(TRC_51_GRID_EA_OFFSET, TRC_FLOAT, &EXCTRL.fltAcPowerP.fOut);
	TRC_AddNode(TRC_52_GRID_EB_OFFSET, TRC_FLOAT, &EXCTRL.fltAcPowerQ.fOut);

	TRC_AddNode(TRC_54_GRID_FREQ, TRC_FLOAT, &GRIDTHETA.fFreq);
#if 0
	TRC_AddNode(TRC_55_GRID_FAST_RMS_A, TRC_FLOAT, &(FastRmsItems.fsGridV[0].fRMS2));
	TRC_AddNode(TRC_56_GRID_FAST_RMS_B, TRC_FLOAT, &(FastRmsItems.fsGridV[1].fRMS2));
	TRC_AddNode(TRC_57_GRID_FAST_RMS_C, TRC_FLOAT, &(FastRmsItems.fsGridV[2].fRMS2));
#else
	TRC_AddNode(TRC_55_GRID_FAST_RMS_A, TRC_FLOAT, &(EXCTRL.trIa));
	TRC_AddNode(TRC_56_GRID_FAST_RMS_B, TRC_FLOAT, &(EXCTRL.trIb));
	TRC_AddNode(TRC_57_GRID_FAST_RMS_C, TRC_FLOAT, &(EXCTRL.trIc));
#endif
	TRC_AddNode(TRC_58_GRID_Vq_PU, TRC_FLOAT, &(EXCTRL.VqPU));

	TRC_AddNode(TRC_60_CON_IDE_REF, TRC_FLOAT, &INVCTRL.IdeRef);
	TRC_AddNode(TRC_61_CON_IDE, TRC_FLOAT, &INVCTRL.Ide);
	TRC_AddNode(TRC_62_CON_IQE_REF, TRC_FLOAT, &INVCTRL.IqeRef);
	TRC_AddNode(TRC_63_CON_IQE, TRC_FLOAT, &INVCTRL.Iqe);
	TRC_AddNode(TRC_64_CON_IA_REF, TRC_FLOAT, &INVCTRL.IaRef);
	TRC_AddNode(TRC_65_CON_IB_REF, TRC_FLOAT, &INVCTRL.IbRef);
	TRC_AddNode(TRC_66_CON_IC_REF, TRC_FLOAT, &INVCTRL.IcRef);
	TRC_AddNode(TRC_67_CON_VDEREF, TRC_FLOAT, &INVCTRL.VdeRef);
	TRC_AddNode(TRC_68_CON_VQEREF, TRC_FLOAT, &INVCTRL.VqeRef);
	TRC_AddNode(TRC_69_CON_EA_REF, TRC_FLOAT, &INVCTRL.VaRef);
	TRC_AddNode(TRC_70_CON_EB_REF, TRC_FLOAT, &INVCTRL.VbRef);
	TRC_AddNode(TRC_71_CON_EC_REF, TRC_FLOAT, &INVCTRL.VcRef);
	TRC_AddNode(TRC_72_CON_PWM_A, TRC_FLOAT, &Con.uPhA);
	TRC_AddNode(TRC_73_CON_PWM_B, TRC_FLOAT, &Con.uPhB);
	TRC_AddNode(TRC_74_CON_PWM_C, TRC_FLOAT, &Con.uPhC);
	TRC_AddNode(TRC_75_THETA_DIFF, TRC_FLOAT, &ACP.INV.THETA.diff_time);
#if IIVC_ENB
	TRC_AddNode( TRC_76_IIVCQ_REF, TRC_FLOAT, &InitInvVControl_q.fRef);
	TRC_AddNode( TRC_77_IINV_VDE, TRC_FLOAT, &InitINV.fltEdeCtrl.fOut);
	TRC_AddNode( TRC_78_IINV_VQE, TRC_FLOAT, &InitINV.fltEqeCtrl.fOut);
#endif
	TRC_AddNode(TRC_80_CON_VC_REF, TRC_FLOAT, &ACP.INV.THETA.diff_rad);

	//	TRC_AddNode( TRC_85_GRID_RMS_FILTER_VA, TRC_FLOAT, &(IIRGridV[0].fOut));
	//	TRC_AddNode( TRC_86_GRID_RMS_FILTER_VB, TRC_FLOAT, &(IIRGridV[1].fOut));
	//	TRC_AddNode( TRC_87_GRID_RMS_FILTER_VC, TRC_FLOAT, &(IIRGridV[2].fOut));

	//	TRC_AddNode(TRC_85_GRID_RMS_FILTER_VA, TRC_FLOAT, &(EXCTRL.fltFsGridV[0].fOut));
	//	TRC_AddNode(TRC_86_GRID_RMS_FILTER_VB, TRC_FLOAT, &(EXCTRL.fltFsGridV[1].fOut));
	//	TRC_AddNode(TRC_87_GRID_RMS_FILTER_VC, TRC_FLOAT, &(EXCTRL.fltFsGridV[2].fOut));

	TRC_AddNode(TRC_85_GRID_RMS_FILTER_VA, TRC_FLOAT, &(INVCTRL.VsnRef));

#if CC_EXECUTION_PERIOD
	TRC_AddNode(TRC_89_CC_PERIOD_TIME2, TRC_UNS, (void*)&CC_uExecutionTime2);
#endif

#if CC_EXECUTION_TIME
	TRC_AddNode(TRC_90_CC_EXECUTION_TIME, TRC_UNS, (void*)&CC_uExecutionTime);
#endif

	TRC_AddNode(TRC_91_CC_PERIOD, TRC_UNS, (void*) &CC_uPeriod);
	TRC_AddNode(TRC_93_FLT_IQE, TRC_FLOAT, &(INVCTRL.fltIqe.fOut));

	TRC_AddNode(TRC_100_, TRC_FLOAT, &(CTRL.INV.vds_ref));
	TRC_AddNode(TRC_101_, TRC_FLOAT, &(CTRL.INV.vqs_ref));
	TRC_AddNode(TRC_102_, TRC_FLOAT, &(CTRL.INV.ids_ref_lim));
	TRC_AddNode(TRC_103_, TRC_FLOAT, &(CTRL.INV.iqs_ref_lim));
	TRC_AddNode(TRC_104_, TRC_FLOAT, &(CTRL.INV.va_ref));
	TRC_AddNode(TRC_105_, TRC_FLOAT, &(CTRL.INV.vb_ref));
	TRC_AddNode(TRC_106_, TRC_FLOAT, &(CTRL.INV.vc_ref));
	TRC_AddNode(TRC_107_, TRC_FLOAT, &(CTRL.INV.va_ref_ad));
	TRC_AddNode(TRC_108_, TRC_FLOAT, &(CTRL.INV.vb_ref_ad));
	TRC_AddNode(TRC_109_, TRC_FLOAT, &(CTRL.INV.vc_ref_ad));
	TRC_AddNode(TRC_110_, TRC_FLOAT, &(ACP.CAP.ia));
	TRC_AddNode(TRC_111_, TRC_FLOAT, &(ACP.CAP.ib));
	TRC_AddNode(TRC_112_, TRC_FLOAT, &(ACP.CAP.ic));
	TRC_AddNode(TRC_113_, TRC_FLOAT, &(CTRL.INV.theta_ref));
	TRC_AddNode(TRC_114_, TRC_FLOAT, &(ACP.INV.vds));
	TRC_AddNode(TRC_115_, TRC_FLOAT, &(ACP.INV.vqs));
	TRC_AddNode(TRC_116_, TRC_FLOAT, &(ACP.INV.ids));
	TRC_AddNode(TRC_117_, TRC_FLOAT, &(ACP.INV.iqs));
	TRC_AddNode(TRC_118_, TRC_FLOAT, &(CTRL.INV.PRC_Ids.FILTER.har_1st.y));
	TRC_AddNode(TRC_119_, TRC_FLOAT, &(CTRL.INV.PRC_Ids.FILTER.har_5th.y));
	TRC_AddNode(TRC_120_, TRC_FLOAT, &(CTRL.INV.PRC_Ids.FILTER.har_7th.y));
	TRC_AddNode(TRC_121_, TRC_FLOAT, &(CTRL.INV.PRC_Ids.FILTER.har_11th.y));
	TRC_AddNode(TRC_122_, TRC_FLOAT, &(CTRL.INV.PRC_Iqs.FILTER.har_1st.y));
	TRC_AddNode(TRC_123_, TRC_FLOAT, &(CTRL.INV.PRC_Iqs.FILTER.har_5th.y));
	TRC_AddNode(TRC_124_, TRC_FLOAT, &(CTRL.INV.PRC_Iqs.FILTER.har_7th.y));
	TRC_AddNode(TRC_125_, TRC_FLOAT, &(CTRL.INV.PRC_Iqs.FILTER.har_11th.y));
	TRC_AddNode(TRC_126_, TRC_FLOAT, &(CTRL.INV.PRC_Vds.FILTER.har_1st.y));
	TRC_AddNode(TRC_127_, TRC_FLOAT, &(CTRL.INV.PRC_Vds.FILTER.har_5th.y));
	TRC_AddNode(TRC_128_, TRC_FLOAT, &(CTRL.INV.PRC_Vds.FILTER.har_7th.y));
	TRC_AddNode(TRC_129_, TRC_FLOAT, &(CTRL.INV.PRC_Vds.FILTER.har_11th.y));
	TRC_AddNode(TRC_130_, TRC_FLOAT, &(CTRL.INV.PRC_Vqs.FILTER.har_1st.y));
	TRC_AddNode(TRC_131_, TRC_FLOAT, &(CTRL.INV.PRC_Vqs.FILTER.har_5th.y));
	TRC_AddNode(TRC_132_, TRC_FLOAT, &(CTRL.INV.PRC_Vqs.FILTER.har_7th.y));
	TRC_AddNode(TRC_133_, TRC_FLOAT, &(CTRL.INV.PRC_Vqs.FILTER.har_11th.y));
	TRC_AddNode(TRC_134_, TRC_FLOAT, &(CTRL.INV.SYNC.w));
	TRC_AddNode(TRC_135_, TRC_FLOAT, &(CTRL.INV.SYNC.v));
	TRC_AddNode(TRC_136_, TRC_FLOAT, &(CTRL.INV.SYNC.FILTER.iir_w.fOut));
	TRC_AddNode(TRC_137_, TRC_FLOAT, &(ACP.INV.vds_ll));
	TRC_AddNode(TRC_138_, TRC_FLOAT, &(ACP.INV.vqs_ll));
	TRC_AddNode(TRC_139_, TRC_FLOAT, &(ACP.PCC.vds));
	TRC_AddNode(TRC_140_, TRC_FLOAT, &(ACP.PCC.vqs));
	TRC_AddNode(TRC_141_, TRC_FLOAT, &(CTRL.INV.DROOP.w_sec));
	TRC_AddNode(TRC_142_, TRC_FLOAT, &(CTRL.INV.DROOP.v_sec));
	TRC_AddNode(TRC_143_, TRC_FLOAT, &(CTRL.INV.DROOP.w_ref_sec));
	TRC_AddNode(TRC_144_, TRC_FLOAT, &(CTRL.INV.DROOP.v_ref_sec));
	TRC_AddNode(TRC_145_, TRC_FLOAT, &(CTRL.INV.DROOP.v_ref_ramp));
	TRC_AddNode(TRC_146_, TRC_FLOAT, &(CTRL.INV.PQC.w_ref));
	TRC_AddNode(TRC_147_, TRC_FLOAT, &(CTRL.INV.PQC.v_ref));
	TRC_AddNode(TRC_148_, TRC_FLOAT, &(CTRL.INV.PQC.q_ref));
	TRC_AddNode(TRC_149_, TRC_FLOAT, &(CTRL.INV.ids_ref));
	TRC_AddNode(TRC_150_, TRC_FLOAT, &(CTRL.INV.iqs_ref));

	TRC_AddNode(TRC_151_, TRC_FLOAT, &(CTRL.INV.PRC_Ids.output));
	//TRC_AddNode(TRC_151_, TRC_FLOAT, &(ACP.INV.va_n));
	TRC_AddNode(TRC_152_, TRC_FLOAT, &(ACP.INV.vb_n));
	TRC_AddNode(TRC_153_, TRC_FLOAT, &(ACP.INV.vc_n));
	TRC_AddNode(TRC_154_, TRC_FLOAT, &(CTRL.INV.VUC.vuf));
	TRC_AddNode(TRC_155_, TRC_FLOAT, &(CTRL.INV.VI.vds_compen));
	TRC_AddNode(TRC_156_, TRC_FLOAT, &(CTRL.INV.VI.vqs_compen));
	TRC_AddNode(TRC_157_, TRC_FLOAT, &(CTRL.INV.vds_ref_vi));
	TRC_AddNode(TRC_158_, TRC_FLOAT, &(CTRL.INV.vqs_ref_vi));
	TRC_AddNode(TRC_159_, TRC_FLOAT, &(CTRL.INV.vds_ref_vi_vuc));
	TRC_AddNode(TRC_160_, TRC_FLOAT, &(CTRL.INV.vqs_ref_vi_vuc));
	TRC_AddNode(TRC_161_, TRC_FLOAT, &(CTRL.INV.VI.r));
	TRC_AddNode(TRC_162_, TRC_FLOAT, &(CTRL.INV.VI.l));
	TRC_AddNode(TRC_163_, TRC_FLOAT, &(ACP.BYP.va));
	TRC_AddNode(TRC_164_, TRC_FLOAT, &(ACP.BYP.vb));
	TRC_AddNode(TRC_165_, TRC_FLOAT, &(ACP.BYP.vc));
	TRC_AddNode(TRC_166_, TRC_FLOAT, &(ACP.BYP.v_pk));
	TRC_AddNode(TRC_167_, TRC_FLOAT, &(CTRL.BYP.SYNC.FILTER.iir_v.fOut));
	TRC_AddNode(TRC_168_, TRC_FLOAT, &(CTRL.BYP.SYNC.w));
	TRC_AddNode(TRC_169_, TRC_FLOAT, &(EXCTRL.fltFsBypV[0].fOut));
	TRC_AddNode(TRC_170_, TRC_FLOAT, &(EXCTRL.fltFsBypV[1].fOut));
	TRC_AddNode(TRC_171_, TRC_FLOAT, &(EXCTRL.fltFsBypV[2].fOut));
	TRC_AddNode(TRC_172_, TRC_FLOAT, &(ACP.PCC.v_pk));
	TRC_AddNode(TRC_173_, TRC_FLOAT, &(CTRL.INV.VI.iqs_delay90));
	TRC_AddNode(TRC_174_, TRC_FLOAT, &(INVCTRL.EaDelay90));
	TRC_AddNode(TRC_175_, TRC_FLOAT, &(EXCTRL.fltFreq_Byp.fOut));
	TRC_AddNode(TRC_175_, TRC_FLOAT, &(GRID_BYP_THETA.fltFreqW.fOut));
	TRC_AddNode(TRC_176_, TRC_FLOAT, &(fBypOmega_Max));
	TRC_AddNode(TRC_177_, TRC_FLOAT, &(fBypOmega_Min));
	TRC_AddNode(TRC_178_, TRC_FLOAT, &(CTRL.INV.vi_dc));
	TRC_AddNode(TRC_179_, TRC_FLOAT, &(ACP.BYP.vds));
	TRC_AddNode(TRC_180_, TRC_FLOAT, &(ACP.BYP.vqs));

	TRC_AddNode(TRC_181_, TRC_FLOAT, &(EXCTRL.fltFsBypV[0].fPrevIn));
	TRC_AddNode(TRC_182_, TRC_FLOAT, &(EXCTRL.fltFsBypV[1].fPrevIn));
	TRC_AddNode(TRC_183_, TRC_FLOAT, &(EXCTRL.fltFsBypV[2].fPrevIn));
	TRC_AddNode(TRC_184_, TRC_FLOAT, &(ACP.GEN.vds));
	TRC_AddNode(TRC_185_, TRC_FLOAT, &(ACP.GEN.vqs));
	TRC_AddNode(TRC_186_, TRC_FLOAT, &(EXCTRL.genEds));
	TRC_AddNode(TRC_187_, TRC_FLOAT, &(EXCTRL.genEqs));

	TRC_AddNode(TRC_188_, TRC_FLOAT, &(GenBlock.pcsDecreasePowerP));
	TRC_AddNode(TRC_189_, TRC_FLOAT, &(GenBlock.pcsDecreasePowerQ));

	TRC_AddNode(TRC_190_, TRC_FLOAT, &(GRID_BYP_THETA.fPIOut));
//	TRC_AddNode(TRC_190_, TRC_FLOAT, &(CTRL.GEN.SYNC.FILTER.iir_v.fOut));
	TRC_AddNode(TRC_191_, TRC_FLOAT, &(CTRL.GEN.SYNC.w));

	TRC_AddNode(TRC_192_, TRC_FLOAT, &(INVCTRL.SinTheta));
	TRC_AddNode(TRC_193_, TRC_FLOAT, &(CTRL.INV.SYNC.FILTER.iir_inv_v.fOut));

	TRC_AddNode(TRC_194_, TRC_FLOAT, &(CTRL.INV.SYNC.pi_mag_input));

	TRC_AddNode(TRC_195_, TRC_FLOAT, &(EXCTRL.bypEqe));
	TRC_AddNode(TRC_197_, TRC_FLOAT, &(EXCTRL.bypEde));

	//TRC_AddNode(TRC_196_, TRC_FLOAT, &(timer_sec_r));
	//TRC_AddNode(TRC_197_, TRC_FLOAT, &(timer_sec_shadow));
	//TRC_AddNode(TRC_198_, TRC_FLOAT, &(GRID_BYP_THETA.fRadian));

	TRC_AddNode(TRC_196_, TRC_FLOAT, &(aaa));
}

/*
 * call: 없음.
 */
float CVC_GetReference(void)
{
	return IVC.fRef;
}

/*
 * call: ACI_ResponseDIOSTATUS()]
 * call: Process1sec()
 */
Uns CVC_GetStatus(void)
{
	return IVC.uStatus;
}

/*
 * call: idlLoop()
 */
void CVC_UpdateConEPeakInverse(void)
{
	INVCTRL.EPeakInverse = 1. / INVCTRL.fltEqe.fOut;
}

/*
 * call: cint11()
 * when: CC_PERIOD == 0 또는 VPRD.uRef != CC_uPeriod
 * call: CC_Create()
 * call: PARAM_UpdateALL()
 * when: CTRL_CC_DI_DT 파라미터 변경 되었을 경우.
 */
void CVC_UpdateParameter(void)
{
	float delta;
	float fRampTime;

	if (PRM_PCS[CTRL_RAMP].iValue < 1)
		fRampTime = 1e-3;
	else
		fRampTime = (float) PRM_PCS[CTRL_RAMP].iValue * 1e-3;

	//-delta = (float) PRM_PCS[CTRL_CC_DI_DT].iValue * INVERTER.RATE.fPeakCurrent * INVCTRL.fCurrentLimit * 0.01 * CC_tsSample;
	delta = (float) PRM_PCS[CTRL_CC_DI_DT].iValue * ACP.INV.RATE.Iph_pk
			* CC_tsSample;
	RAMP_SetDelta(ICC.pRAMPOutput, delta);

	delta = (float) PARAM_VAL(BATT_IVC_VDC_RAMP) * CC_tsSample;
	RAMP_SetDelta(IVC.pRAMPOutput, delta);

	//130620
	//-delta = (float) PRM_PCS[CTRL_CC_DI_DT].iValue * INVERTER.RATE.fPeakCurrent * INVCTRL.fCurrentLimit * 0.01 * CC_tsSample;
	delta = (float) PRM_PCS[CTRL_CC_DI_DT].iValue * ACP.INV.RATE.Iph_pk
			* CC_tsSample;
	RAMP_SetDelta(PCC.pRAMPOutput, delta);
	RAMP_SetDelta(PCC.pRAMPOutput_Q, delta);

	// Vdc Ref를 100Vdc변화시키는데 필요한 시간
#if USE_FASTRTS_LIB_DIV
	delta = div((100. * CC_tsSample), fRampTime);
#else
	delta = 100. * CC_tsSample / fRampTime;
#endif
	RAMP_SetDelta(IVC.pRAMPInput, delta);

}

/*
 * call: CC_Create()
 * call: PARAM_UpdateALL()
 * when: CTRL_VC_P_GAIN 파라미터 변경 되었을 경우.
 */
void CVC_UpdateGains(void)
{
	IVC.K.fP = (float) PRM_PCS[CTRL_VC_P_GAIN].iValue
			* (float) PRM_PCS[CTRL_VC_P_GAIN].fIncDec;
	IVC.K.fIT = (float) PRM_PCS[CTRL_VC_I_GAIN].iValue
			* (float) PRM_PCS[CTRL_VC_I_GAIN].fIncDec * CC_tsSample;
#if USE_FASTRTS_LIB_DIV
	IVC.K.fA = div(1., IVC.K.fP);
#else
	IVC.K.fA = 1. / IVC.K.fP;
#endif

	//13.3.23
	ICC.K.fP = (float) PRM_PCS[BATT_ICC_P_GAIN].iValue
			* (float) PRM_PCS[BATT_ICC_P_GAIN].fIncDec;
	ICC.K.fIT = (float) PRM_PCS[BATT_ICC_I_GAIN].iValue
			* (float) PRM_PCS[BATT_ICC_I_GAIN].fIncDec * CC_tsSample;

#if USE_FASTRTS_LIB_DIV
	ICC.K.fA = div(1., ICC.K.fP);
#else
	ICC.K.fA = 1. / ICC.K.fP;
#endif



	//130622
	PCC.K.fP = (float) PRM_PCS[BATT_PCC_P_GAIN].iValue
			* (float) PRM_PCS[BATT_PCC_P_GAIN].fIncDec;
	PCC.K.fIT = (float) PRM_PCS[BATT_PCC_I_GAIN].iValue
			* (float) PRM_PCS[BATT_PCC_I_GAIN].fIncDec * CC_tsSample;

#if USE_FASTRTS_LIB_DIV
	PCC.K.fA = div(1., PCC.K.fP);
#else
	PCC.K.fA = 1. / PCC.K.fP; // fixed bug.
#endif

	PCC.K_Q.fP = (float) PRM_PCS[BATT_PCC_P_GAIN].iValue
			* (float) PRM_PCS[BATT_PCC_P_GAIN].fIncDec;
	PCC.K_Q.fIT = (float) PRM_PCS[BATT_PCC_I_GAIN].iValue
			* (float) PRM_PCS[BATT_PCC_I_GAIN].fIncDec * CC_tsSample;

#if USE_FASTRTS_LIB_DIV
	PCC.K_Q.fA = div(1., PCC.K_Q.fP);
#else
	PCC.K_Q.fA = 1. / PCC.K_Q.fP;
#endif


}

//by JCNET
void SCC_UpdateGains(void)
{
    //++JCNET
    // TODO Sag Compensation PI Gain!!
        SCCTRL.K[0].fP = (float) PRM_PCS[CTRL_SCC_D_P_GAIN].iValue * (float) PRM_PCS[CTRL_SCC_D_P_GAIN].fIncDec;
        SCCTRL.K[0].fIT = (float) PRM_PCS[CTRL_SCC_D_I_GAIN].iValue* (float) PRM_PCS[CTRL_SCC_D_I_GAIN].fIncDec * CC_tsSample;
        SCCTRL.K[0].fA = 1. / SCCTRL.K[0].fP;

        SCCTRL.K[1].fP = (float) PRM_PCS[CTRL_SCC_Q_P_GAIN].iValue * (float) PRM_PCS[CTRL_SCC_Q_P_GAIN].fIncDec;
        SCCTRL.K[1].fIT = (float) PRM_PCS[CTRL_SCC_Q_I_GAIN].iValue* (float) PRM_PCS[CTRL_SCC_Q_I_GAIN].fIncDec * CC_tsSample;
        SCCTRL.K[1].fA = 1. / SCCTRL.K[1].fP;

        SCCTRL.K[2].fP = (float) PRM_PCS[CTRL_SCC_N_P_GAIN].iValue * (float) PRM_PCS[CTRL_SCC_N_P_GAIN].fIncDec;
        SCCTRL.K[2].fIT = (float) PRM_PCS[CTRL_SCC_N_I_GAIN].iValue* (float) PRM_PCS[CTRL_SCC_N_I_GAIN].fIncDec * CC_tsSample;
        SCCTRL.K[2].fA = 1. / SCCTRL.K[2].fP;
    //--
}
/*
 * call: 없음.
 */
void CVC_ChangeController(float alpha)
{
	IVC.pARG->fAlpha = alpha;
}

/*
 * call: 없음.
 */
float CC_GetWe(void)
{
	return CC_fWe;
}

float CC_GetPowerRefP(void)
{
	return PCC.fRefP;
}
float CC_GetPowerRefQ(void)
{
	return PCC.fRefQ;
}

/*
 * call: CC_Create()
 * call: PARAM_UpdateALL()
 * when: CTRL_CC_I_GAIN 파라미터 변경 되었을 경우. SYS_UpdateLiViaParam()
 */
void CC_UpdateGains(void)
{
	//SYS_UpdateKpccKiccLi(); 외부에서 이 함수를 호출 하기전에 호출 하도록 하였음.

	INVCTRL.KpCC = (float) PRM_PCS[CTRL_CC_P_GAIN].iValue
			* (float) PRM_PCS[CTRL_CC_P_GAIN].fIncDec;
	INVCTRL.KiCC = (float) PRM_PCS[CTRL_CC_I_GAIN].iValue
			* (float) PRM_PCS[CTRL_CC_I_GAIN].fIncDec;
	INVCTRL.KiTCC = INVCTRL.KiCC * CC_tsSample;

	PLL_K.fP = (float) PRM_PCS[CTRL_PLL_P_GAIN].iValue
			* (float) PRM_PCS[CTRL_PLL_P_GAIN].fIncDec;
	PLL_K.fIT = (float) PRM_PCS[CTRL_PLL_I_GAIN].iValue
			* (float) PRM_PCS[CTRL_PLL_I_GAIN].fIncDec * CC_tsSample;
#if USE_FASTRTS_LIB_DIV
	PLL_K.fA = div(1., PLL_K.fP);
#else
	PLL_K.fA = 1. / PLL_K.fP;
#endif
}

/*
 * call: CC_Create()
 * call: FLT_Initialize()
 * call: PARAM_UpdateALL()
 * when: CTRL_CC_PERIOD 파라미터 변경 되었을 경우.
 */
void CC_UpdateParameter(void)
{
	int GridFreq;
#if DBUG_MODE == 2
	float fGridFreq;
#endif

	GridFreq = PRM_PCS[GRID_RATED_FREQ].iValue;
	if (GridFreq < 50 || GridFreq > 60)
		GridFreq = 60;

#if DBUG_MODE == 2
	//-fGridFreq = GridFreq + (float)PRM_PCS[ANL_AI3_GAIN].iValue * (float)PRM_PCS[ANL_AI3_GAIN].fIncDec;
	fGridFreq = GridFreq;
#endif

	if (PRM_PCS[CTRL_CC_PERIOD].iValue != 0)
	{
		CC_uPeriod = PRM_PCS[CTRL_CC_PERIOD].iValue; // n => CC 주기
		if (CC_uPeriod < CC_PERIOD_MIN)
			CC_uPeriod = CC_PERIOD_MIN;
		if (CC_uPeriod > CC_PERIOD_MAX)
			CC_uPeriod = CC_PERIOD_MAX;
	}
	else
	{
		CC_uPeriod = 285;
	}

	CC_tsCC = CC_uPeriod * CC_MICRO_UNIT;
	CC_tsSample = CC_tsCC;
#if DOUBLE_CONTROL == 1
	CC_tsSample = CC_tsCC * 0.5;
#endif
	CC_tsSampleInv = 1 / CC_tsSample;


	// LF28 PWM 에서 Up-Down Mdoe인 경우 200usec가 되려면...
	INVCTRL.PwmOnTime.uPeriod = (CC_uPeriod * PWM_CLK) >> 1;

	PWM_SetPeriod(PWM_CON, &INVCTRL.PwmOnTime);

#if DBUG_MODE == 2
	CC_fWe = 2.0 * PI * fGridFreq; /* rad/sec, SEC_2_RAD */
#else
	CC_fWe = 2.0 * PI * GridFreq; /* rad/sec, SEC_2_RAD */
#endif
#if USE_FASTRTS_LIB_DIV
	CC_fWeInverse = div(1., CC_fWe);
#else
	CC_fWeInverse = 1. / CC_fWe; /* RAD_2_SEC */
#endif
	CC_fDTheta = CC_fWe * CC_tsSample;

	// Radian을 시간으로 변경할 때는 2*PI*f를 나누고
	// 시간을 Radian으로 변경할 때는 2*PI*f를 곱한다.

	CC_UpdateIdqRamp();
	CC_UpdateIdqDelayRamp();
}

#if IIVC_ENB
/*
 * call: CC_Create()
 * call: PARAM_UpdateALL()
 * when: CTRL_IINV_VOLTAGE_BUILDUP_TIME 파라미터 변경 되었을 경우.
 */
void CC_UpdateIINVParameter( void )
{
	float a, b, delta;

	//-b = PRM_PCS[GRID_RATED_VOLT].iValue * Model_getAttribute(TRRatio) * INV_SQRT3 * SQRT2;

	// Seth, 2013-06-08
	//--b = (float)PRM_PCS[GRID_RATED_VOLT].iValue * svModel.modelData.TRRatio * INV_SQRT3 * SQRT2;
	b = INV_RATED_VOLTAGE * INV_SQRT3 * SQRT2;

	a = b * 1.25;

	InitInvVControl_d.pARG->fMax = a;
	InitInvVControl_d.pARG->fMin = -a;

	InitInvVControl_q.pARG->fMax = a;
	InitInvVControl_q.pARG->fMin = -a;

	// IINV의 입력 Ref를 0에서 정격까지 변화시키는데 필요한 시간 {BUILDUP_TIME}
#if USE_FASTRTS_LIB_DIV
	//-delta = div(b*CC_tsSample, PARAM_VAL(CTRL_IINV_VOLTAGE_BUILDUP_TIME) * 0.001); // unit is msec
#else
	delta = ( b * CC_tsSample ) / ( (float)PARAM_VAL(CTRL_IINV_VOLTAGE_BUILDUP_TIME) * 0.001); // unit is msec
#endif

	RAMP_SetDelta(InitInvVControl_d.pRAMPInput, delta);
	RAMP_SetDelta(InitInvVControl_q.pRAMPInput, delta);

	// IINV의 제어기출력을 0에서 정격까지 변화시키는데 필요한 시간. BUILDUP_TIME * 5 p.u
	//delta = (b * CC_tsSample) * 5;
	delta *= 5.0;
	RAMP_SetDelta(InitInvVControl_d.pRAMPOutput, delta);
	RAMP_SetDelta(InitInvVControl_q.pRAMPOutput, delta);
}

/*
 * call: CC_Create()
 * call: PARAM_UpdateALL()
 * when: CTRL_IIVCD_P&I&_GAIN 파라미터 변경 되었을 경우.
 * when: CTRL_IIVCQ_P&I&_GAIN 파라미터 변경 되었을 경우.
 */
void CC_UpdateIINVGains(void)
{
	InitInvVControl_d.K.fP = (float)PRM_PCS[CTRL_IIVCD_P_GAIN].iValue * (float)PRM_PCS[CTRL_IIVCD_P_GAIN].fIncDec;
	InitInvVControl_d.K.fIT = (float)PRM_PCS[CTRL_IIVCD_I_GAIN].iValue * (float)PRM_PCS[CTRL_IIVCD_I_GAIN].fIncDec * CC_tsSample;
	InitInvVControl_d.K.fA = 1. / InitInvVControl_d.K.fP;

	InitInvVControl_q.K.fP = (float)PRM_PCS[CTRL_IIVCQ_P_GAIN].iValue * (float)PRM_PCS[CTRL_IIVCQ_P_GAIN].fIncDec;
	InitInvVControl_q.K.fIT = (float)PRM_PCS[CTRL_IIVCQ_I_GAIN].iValue * (float)PRM_PCS[CTRL_IIVCQ_I_GAIN].fIncDec * CC_tsSample;
	InitInvVControl_q.K.fA = 1. / InitInvVControl_q.K.fP;
}

/*
 * call: CC_Create()
 */
static void IIVCCreate( InitInvVoltCtrl *this, PICon *pPI, PIArg *pPIArg, Ramp *pRampIn, Ramp *pRampOut )
{
	float delta = 0.01;

	this->pARG = PIArg_( pPIArg, &this->K );

	this->pPI = PICon_(pPI);

	this->pRAMPInput = Ramp_(pRampIn, delta);
	this->pRAMPOutput = Ramp_(pRampOut, delta);

	IIVC_Initialize(this);
}

#endif
/*
 * call: CC_Create()
 * call: PARAM_UpdateALL()
 * when: CTRL_IINV_SYNC_TOLERANCE_THETA 파라미터 변경 되었을 경우.
 */
void CC_UpdateSyncToleranceTheta(void)
{
	/*
	 * The unit of parameter {IINV_SYNC_TOLERANCE_THETA} is degree
	 * So change that to radian to compare to the differences between two PLLs.
	 */
	ACP.INV.THETA.sync_accept_rad =
			(float) PARAM_VAL(CTRL_IINV_SYNC_TOLERANCE_THETA) * 2 * PI / 360.0;
}

/*
 * call: SYS_UpdateInverterStatus()
 * when: SYS_INV_GRIDMC_ON state AND SYS_SUB_GRIDMC_ON_BUILD_UP AND flgOdt == ODT_FINISH
 */
Bool CC_SyncOK(void)
{
	if ((fabs(ACP.INV.THETA.diff_rad) < ACP.INV.THETA.sync_accept_rad))
		return TRUE;
	else
		return FALSE;
}

Bool CC_VoltageGenerateOK(void)
{
	float compare;

	//compare = CTRL.INV.DROOP.v_ref_ramp;
	compare = INVCTRL.Eqe; // 2016.03.30

	//if (compare < ACP.INV.RATE.Vph_pk * 0.9)
	if (compare < (ACP.INV.RATE.Vph_pk * (float)PRM_PCS[IS_UV_LEVEL1].iValue * 0.01))
		return FALSE;

	return TRUE;
}
/*
 * call: SYS_UpdateInverterStatus()
 * when: always
 */
void CC_UpdatePLLGains(Uns uMode)
{
	if (uMode == CC_PLL_NORMAL)
	{
		PLL_K.fP = (float) PRM_PCS[CTRL_PLL_P_GAIN].iValue
				* (float) PRM_PCS[CTRL_PLL_P_GAIN].fIncDec;
		PLL_K.fIT = (float) PRM_PCS[CTRL_PLL_I_GAIN].iValue
				* (float) PRM_PCS[CTRL_PLL_I_GAIN].fIncDec * CC_tsSample;
	}
	else /* Initial Voltage Build-up */
	{
		PLL_K.fP = (float) PRM_PCS[CTRL_IINV_PLL_P_GAIN].iValue
				* (float) PRM_PCS[CTRL_IINV_PLL_P_GAIN].fIncDec;
		PLL_K.fIT = (float) PRM_PCS[CTRL_IINV_PLL_I_GAIN].iValue
				* (float) PRM_PCS[CTRL_IINV_PLL_I_GAIN].fIncDec * CC_tsSample;
	}

#if USE_FASTRTS_LIB_DIV
	PLL_K.fA = div(1., PLL_K.fP);
#else
	PLL_K.fA = 1. / PLL_K.fP;
#endif
}
/*
 * call: CC_Create()
 * call: PARAM_UpdateALL()
 * when: BATT_OC_LEVEL && INV_CAPACITY 파라미터 변경 되었을 경우.
 */
void BATT_UpdateParameter()
{
	//전류 허용 범위 5kW / 48V = 104A
	BATCTRL.fIMaxPermissible =
			((float) PRM_PCS[INV_CAPACITY].iValue * (float) 1000 * (float) 1.05)
					/ (float) PARAM_VAL(BATT_V_RANGE_MIN);
	BATCTRL.OCLevel = BATCTRL.fIMaxPermissible
			* (float) PRM_PCS[BATT_OC_LEVEL].iValue * 0.01;
}

/*
 * call: CC_Create()
 * call: PARAM_UpdateALL()
 * when: CTRL_OPTION 파라미터 변경 되었을 경우.
 */
void CTRL_UpdateOptions()
{
	switch (PRM_PCS[CTRL_OPTION].iValue)
	{
	case 0:
	case 96:
		OPTION.uPWM = 0;
		break;
	case 32:
		OPTION.uPWM = 32;
		break;
	case 64:
		OPTION.uPWM = 64;
		break;
	default:
		OPTION.uPWM = 0;
	}
}
#if 0
/*
 * call: cint11()
 * when: PRM_PCS[CTRL_CC_PERIOD].iValue == 0 && VPRD.uRef != CC_uPeriod AND CC_bVoltageFilterShadowing == 1
 * call: PARAM_UpdateALL()
 * when: CTRL_VOLT_LPF 파라미터 변경시.
 *
 */
void CC_UpdateVoltageFilterCoefficient(void)
{
	IIR1_ChangeCoefficient(&INVCTRL.fltEde);
	IIR1_ChangeCoefficient(&INVCTRL.fltEqe);
	IIR1_ChangeCoefficient(&EXCTRL.fltEde);
	IIR1_ChangeCoefficient(&EXCTRL.fltEqe);
	CC_bVoltageFilterShadowing = TRUE;// => maybe should be located in CC_CalcVoltageFilterShadow()
}
#endif
/*
 * call: PARAM_UpdateALL()
 * when: CTRL_CC_PERIOD 파라미터 변경시
 */
void CC_CalcVoltageFilterShadow(void)
{
	/*
	 float t_sample_main, t_sample_is;
	 t_sample_main = CTRL_FILTER_GetSampleTime(CTRL_CC_PERIOD);
	 t_sample_is = CTRL_FILTER_GetSampleTime(IS_CCP);
	 */

	IIR1_UpdateCoeff(&INVCTRL.fltEde, CC_tsSample,
			PRM_PCS[CTRL_VOLT_LPF].iValue);
	IIR1_UpdateCoeff(&INVCTRL.fltEqe, CC_tsSample,
			PRM_PCS[CTRL_VOLT_LPF].iValue);
	IIR1_UpdateCoeff(&EXCTRL.fltEde, CC_tsSample,
			PRM_PCS[CTRL_VOLT_LPF].iValue);
	IIR1_UpdateCoeff(&EXCTRL.fltEqe, CC_tsSample,
			PRM_PCS[CTRL_VOLT_LPF].iValue);
//++JCNET
    IIR1_UpdateCoeff(&SCCTRL.fltEde, CC_tsSample,
            PRM_PCS[CTRL_VOLT_LPF].iValue);
    IIR1_UpdateCoeff(&SCCTRL.fltEqe, CC_tsSample,
            PRM_PCS[CTRL_VOLT_LPF].iValue);
    IIR1_UpdateCoeff(&SCCTRL.fltEne, CC_tsSample,
            PRM_PCS[CTRL_VOLT_LPF].iValue);
//--
}
#if 0
/*
 * call: cint11()
 * when: PRM_PCS[CTRL_CC_PERIOD].iValue == 0 && VPRD.uRef != CC_uPeriod AND CC_bVoltageFilterShadowing == 1
 * CC_bFilterShadowing
 *
 */
void CC_UpdateFilterCoefficient(void)
{
	int i;

	IIR1_ChangeCoefficient(&BATCTRL.fltDCBattVForCC);
	IIR1_ChangeCoefficient(&BATCTRL.fltDCLinkV);
	IIR1_ChangeCoefficient(&BATCTRL.fltDCLinkV2nd);

	IIR1_ChangeCoefficient(&BATCTRL.fltDCBattV);
	IIR1_ChangeCoefficient(&BATCTRL.fltDCBattV_2nd);

	IIR1_ChangeCoefficient(&BATCTRL.fltI);
	IIR1_ChangeCoefficient(&BATCTRL.fltI2);
	IIR1_ChangeCoefficient(&INVCTRL.fltPout);
	IIR1_ChangeCoefficient(&INVCTRL.fltPout2nd);

	IIR1_ChangeCoefficient(&INVCTRL.fltEqe2nd);
	IIR1_ChangeCoefficient(&INVCTRL.fltIqe);

	//130622
	IIR1_ChangeCoefficient(&EXCTRL.fltAcPowerP);
	IIR1_ChangeCoefficient(&EXCTRL.fltAcPowerQ);
}
#endif
/*
 * call: PARAM_UpdateALL()
 * when: CTRL_CC_PERIOD 파라미터 변경시
 */
void CC_CalcFilterShadow(void)
{
	//int i;
	//float a;

	IIR1_UpdateCoeff(&BATCTRL.fltDCLinkV, CC_tsSample,
			CC_DC_V_FLT_CUTOFF_HZ);
	IIR1_UpdateCoeff(&BATCTRL.fltDCLinkV2nd, CC_tsSample,
			CC_DC_V_FLT2ND_CUTOFF_HZ);

	IIR1_UpdateCoeff(&BATCTRL.fltDCBattVForCC, CC_tsSample,
			CC_DC_V_FOR_CC_FLT_CUTOFF_HZ);
	IIR1_UpdateCoeff(&BATCTRL.fltDCBattV, CC_tsSample, CC_DC_V_FLT_CUTOFF_HZ);
	IIR1_UpdateCoeff(&BATCTRL.fltDCBattV_2nd, CC_tsSample,
			CC_DC_V_FLT2ND_CUTOFF_HZ);

	IIR1_UpdateCoeff(&BATCTRL.fltI, CC_tsSample, CC_PV_I_FLT_CUTOFF_HZ);
	IIR1_UpdateCoeff(&BATCTRL.fltI2, CC_tsSample, CC_PV_I_FLT2_CUTOFF_HZ);

	IIR1_UpdateCoeff(&INVCTRL.fltPout, CC_tsSample, CC_POUT_FLT_CUTOFF_HZ);
	IIR1_UpdateCoeff(&INVCTRL.fltPout2nd, CC_tsSample, CC_POUT_FLT_CUTOFF_HZ);

	IIR1_UpdateCoeff(&INVCTRL.fltEqe2nd, CC_tsSample, CC_EQE_2ND_FIL_CUTOFF_HZ);
	IIR1_UpdateCoeff(&INVCTRL.fltIqe, CC_tsSample, CC_IQE_FLT_CUTOFF_HZ);

	//130622
	IIR1_UpdateCoeff(&EXCTRL.fltAcPowerP, CC_tsSample, CC_ACPOWER_CUTOFF_HZ);
	IIR1_UpdateCoeff(&EXCTRL.fltAcPowerQ, CC_tsSample, CC_ACPOWER_CUTOFF_HZ);

	IIR1_UpdateCoeff(&EXCTRL.fltAcPowerP_2nd, CC_tsSample, CC_ACPOWER_CUTOFF_HZ_2ND);
	IIR1_UpdateCoeff(&EXCTRL.fltAcPowerQ_2nd, CC_tsSample, CC_ACPOWER_CUTOFF_HZ_2ND);

	//	CC_bFilterShadowing = TRUE;
}

/*
 * call: MCU_APP_Create()
 */
#pragma CODE_SECTION (CC_Create, "dp_ctrl")
void CC_Create(void)
{
	ScaleGating = PWM_CLK * 1.0e6 * 0.5; /* Continuous Up-down 모드를 이용하므로 1/2해야함*/

	//	memset(&BATCTRL, 0, sizeof(BATCTRL));
	memset(&IVC, 0, sizeof(IVC));
	memset(&IVC_PI, 0, sizeof(IVC_PI));
	memset(&IVC_PIArg, 0, sizeof(IVC_PIArg));
	memset(&IVC_RAMP_OUTPUT, 0, sizeof(IVC_RAMP_OUTPUT));
	memset(&IVC_RAMP_INPUT, 0, sizeof(IVC_RAMP_INPUT));
	//13.3.23
	memset(&ICC, 0, sizeof(ICC));
	memset(&ICC_PI, 0, sizeof(ICC_PI));
	memset(&ICC_PIArg, 0, sizeof(ICC_PIArg));
	memset(&ICC_RAMP_OUTPUT, 0, sizeof(ICC_RAMP_OUTPUT));


	//130621
	memset(&PCC, 0, sizeof(PCC));
	memset(&PCC_PI, 0, sizeof(PCC_PI));
	memset(&PCC_PIArg, 0, sizeof(PCC_PIArg));
	memset(&PCC_PI_Q, 0, sizeof(PCC_PI_Q));
	memset(&PCC_PIArg_Q, 0, sizeof(PCC_PIArg_Q));

	memset(&PCC_RAMP_OUTPUT, 0, sizeof(PCC_RAMP_OUTPUT));
	memset(&PCC_RAMP_OUTPUT_Q, 0, sizeof(PCC_RAMP_OUTPUT_Q));

	//	memset(&GRIDTHETA, 0, sizeof(GRIDTHETA));
	memset(&GRIDTHETA_AI, 0, sizeof(GRIDTHETA_AI));	//13.10.29
	//	memset(&CONTHETA, 0, sizeof(CONTHETA));

	memset(&PLL_K, 0, sizeof(PLL_K));
	memset(&CONTHETA_PIArg, 0, sizeof(CONTHETA_PIArg));
	memset(&CONTHETA_PI, 0, sizeof(CONTHETA_PI));
	memset(&GRIDTHETA_PI, 0, sizeof(GRIDTHETA_PI));
	memset(&GRIDTHETA_PIArg, 0, sizeof(GRIDTHETA_PIArg));
	memset(&GRIDTHETA_PI_AI, 0, sizeof(GRIDTHETA_PI_AI));
	memset(&GRIDTHETA_PIArg_AI, 0, sizeof(GRIDTHETA_PIArg_AI));
/*by JCNET */
    memset(&SCCTHETA_PIArg, 0, sizeof(SCCTHETA_PIArg));
    memset(&SCCTHETA_PI, 0, sizeof(SCCTHETA_PI));
/**/
	//	memset(&EXCTRL, 0, sizeof(EXCTRL));
	//	memset(&INVCTRL, 0, sizeof(INVCTRL));
	//	memset(&OPTION, 0, sizeof(OPTION));
	//	memset(&CTRL, 0, sizeof(CTRL));
	//	memset(&ACP, 0, sizeof(ACP));

#if IIVC_ENB
	memset(&InitInvVControl_d, 0, sizeof(InitInvVControl_d));
	memset(&InitInvVControl_q, 0, sizeof(InitInvVControl_q));
#endif

	//	memset(&BCState, 0, sizeof(BCState));

	CC_BCStateUpdate();
	CC_BCTransIdle();

	CC_InitLoadInfo(); // 13.12.6
	CTRL_UpdateOptions();

	ThetaDetectCreate(&CONTHETA, &CONTHETA_PI, &CONTHETA_PIArg, &PLL_K);
	a = TWO_PI * 10; /* 10Hz */
	CC_SetThetaDetectLimit(&CONTHETA, -a, a);
	ThetaDetectCreate(&GRIDTHETA, &GRIDTHETA_PI, &GRIDTHETA_PIArg, &PLL_K);
	CC_SetThetaDetectLimit(&GRIDTHETA, -a, a);

	ThetaDetectCreate(&GRIDTHETA_AI, &GRIDTHETA_PI_AI, &GRIDTHETA_PIArg_AI,
			&PLL_K); //13.10.29
	CC_SetThetaDetectLimit(&GRIDTHETA_AI, -a, a);

	ThetaDetectCreate(&GRID_BYP_THETA, &GRID_BYP_THETA_PI, &GRID_BYP_THETA_PIArg_AI, &PLL_K); //14.11.28
	CC_SetThetaDetectLimit(&GRID_BYP_THETA, -a, a);

	ThetaDetectCreate(&GEN_THETA, &GEN_THETA_PI, &GEN_THETA_PIArg, &PLL_K);
	CC_SetThetaDetectLimit(&GEN_THETA, -a, a);

/*by JCNET */
	ThetaDetectCreate(&SCCTHETA, &SCCTHETA_PI, &SCCTHETA_PIArg, &PLL_K);
	CC_SetThetaDetectLimit(&SCCTHETA, -a, a);
/* */
	INVCTRL.pRampIdeRef = Ramp_(&RAMPIdeRef, 0.01);
	INVCTRL.pRampIdeQV = Ramp_(&RAMPIdeQV, 0.01);		// Set dummy value..
	INVCTRL.pRampIdePPN = Ramp_(&RAMPIdePPN, 0.01);		// Set dummy value..
	INVCTRL.pRampIqeRef = Ramp_(&RAMPIqeRef, 0.01);	// Set dummy value..
	INVCTRL.pRampIqeRefDelay = Ramp_(&RAMPIqeRefDelay, 0.01);// Set dummy value..
	CC_UpdateParameter();

	BATT_UpdateParameter();

	SYS_UpdateKpccKiccLi(); // + CTRL_CC_P_GAIN이 변경되어도, 자동 계산되도록 함.
	CC_UpdateGains();

#if 0
	/*
	 * Filter creation
	 */
	IIR1_Creation(&INVCTRL.fltEde, CC_tsSample, PRM_PCS[CTRL_VOLT_LPF].iValue);
	IIR1_Creation(&INVCTRL.fltEqe, CC_tsSample, PRM_PCS[CTRL_VOLT_LPF].iValue);
	IIR1_Creation(&INVCTRL.fltEqe2nd, CC_tsSample, CC_EQE_2ND_FIL_CUTOFF_HZ);
	IIR1_Creation(&INVCTRL.fltIqe, CC_tsSample, CC_IQE_FLT_CUTOFF_HZ);

	IIR1_Creation(&EXCTRL.fltEde, CC_tsSample, PRM_PCS[CTRL_VOLT_LPF].iValue);
	IIR1_Creation(&EXCTRL.fltEqe, CC_tsSample, PRM_PCS[CTRL_VOLT_LPF].iValue);

	IIR1_Creation(&BATCTRL.fltDCLinkV, CC_tsSample, CC_DCLINK_V_FLT_CUTOFF_HZ);
	IIR1_Creation(&BATCTRL.fltDCLinkV2nd, CC_tsSample, CC_DCLINK_V_FLT2ND_CUTOFF_HZ);

	IIR1_Creation(&BATCTRL.fltDCBattVForCC, CC_tsSample, CC_BATT_V_FOR_CC_FLT_CUTOFF_HZ);
	IIR1_Creation(&BATCTRL.fltDCBattV, CC_tsSample, CC_DCLINK_V_FLT_CUTOFF_HZ);
	IIR1_Creation(&BATCTRL.fltDCBattV_2nd, CC_tsSample, CC_DCLINK_V_FLT2ND_CUTOFF_HZ);

	IIR1_Creation(&BATCTRL.fltI, CC_tsSample, CC_PV_I_FLT_CUTOFF_HZ);
	IIR1_Creation(&BATCTRL.fltI2, CC_tsSample, CC_PV_I_FLT2_CUTOFF_HZ);

	IIR1_Creation(&INVCTRL.fltPout, CC_tsSample, CC_POUT_FLT_CUTOFF_HZ);
	IIR1_Creation(&INVCTRL.fltPout2nd, CC_tsSample, CC_POUT_FLT_CUTOFF_HZ);

	/*
	 * Filter line item creation
	 */
	cutoff = PARAM_VAL(CTRL_GRID_FS_RMS_CUTOFF_HZ);
	for (i = 0; i < CC_LINE_NUM; i++)
	{
		IIR1_Creation(&EXCTRL.fltFsGridV[i], CC_tsSample, cutoff);
		IIR1_Creation(&EXCTRL.fltFsBypV[i], CC_tsSample, cutoff);
	}

	cutoff = PARAM_VAL(CTRL_INV_FS_RMS_CUTOFF_HZ);
	for (i = 0; i < CC_LINE_NUM; i++)
	{
		IIR1_Creation(&INVCTRL.fltFsConV[i], CC_tsSample, cutoff);
		IIR1_Creation(&INVCTRL.fltFsConVp[i], CC_tsSample, cutoff);
	}
	cutoff = PARAM_VAL(CTRL_FREQ_CUTOFF_HZ);
	IIR1_Creation(&INVCTRL.fltFreq, CC_tsSample, cutoff);
	IIR1_Creation(&EXCTRL.fltFreq, CC_tsSample, cutoff);

	IIR1_Creation(&GRIDTHETA_AI.fltFreqW, CC_tsSample, CC_GRID_FREQ_OMEGA_CUTOFF);

	//130622
	IIR1_Creation(&EXCTRL.fltAcPowerP, CC_tsSample, PARAM_VAL(BATT_PCC_PQ_CUTOFF_HZ));
	IIR1_Creation(&EXCTRL.fltAcPowerQ, CC_tsSample, PARAM_VAL(BATT_PCC_PQ_CUTOFF_HZ));
#endif

	CTRL_FILTER_Create();

	INVCTRL.Ia_OC_Count = 0;
	INVCTRL.Ib_OC_Count = 0;
	INVCTRL.Ic_OC_Count = 0;

	ACP.CAP.ia_OC_Count = 0;
	ACP.CAP.ib_OC_Count = 0;
	ACP.CAP.ic_OC_Count = 0;

	INVCTRL.CCI_Status = CCI_OCCHECK;
	INVCTRL.CCI_2ndOC_Status = CCI_OCCHECK;
	INVCTRL.CCI_OCCheckCnt = 0;

	INVCTRL.FrtMode = FRT_NORMAL;
	INVCTRL.FrtModeVote = 0;

	INVCTRL.fCurrentLimit = (float) PRM_PCS[INV_CURRENT_LMT].iValue
			* (float) PRM_PCS[INV_CURRENT_LMT].fIncDec;

	CTRL_FILTER_UpdateAPS();

	IVC.pPI = PICon_(&IVC_PI);
	IVC.pARG = PIArg_(&IVC_PIArg, &IVC.K);
	IVC.pARG->fAlpha = 1; // 1 ==> PI 제어, 0 ==> IP 제어
	IVC.pRAMPOutput = Ramp_(&IVC_RAMP_OUTPUT, 0.01);
	IVC.pRAMPInput = Ramp_(&IVC_RAMP_INPUT, 0.01);
	//13.3.23
	ICC.pPI = PICon_(&ICC_PI);
	ICC.pARG = PIArg_(&ICC_PIArg, &ICC.K);
	ICC.pARG->fAlpha = 1; // 1 ==> PI 제어, 0 ==> IP 제어
	ICC.pRAMPOutput = Ramp_(&ICC_RAMP_OUTPUT, 0.01);


	//130621
	PCC.pPI = PICon_(&PCC_PI);
	PCC.pARG = PIArg_(&PCC_PIArg, &PCC.K);
	PCC.pARG->fAlpha = 1; // 1 ==> PI 제어, 0 ==> IP 제어

	PCC.pPI_Q = PICon_(&PCC_PI_Q);
	PCC.pARG_Q = PIArg_(&PCC_PIArg_Q, &PCC.K_Q);
	PCC.pARG_Q->fAlpha = 1; // 1 ==> PI 제어, 0 ==> IP 제어

	PCC.pRAMPOutput = Ramp_(&PCC_RAMP_OUTPUT, 0.01);
	PCC.pRAMPOutput_Q = Ramp_(&PCC_RAMP_OUTPUT_Q, 0.01);
	PCC.uState = GPC_GATING;

//++JCNET
	{
	    int i;
	    for( i = 0 ; i < 3  ; i ++)
	    {
	        PICon_(&SCCTRL.PICon[i]);
	        PIArg_(&SCCTRL.PIArg[i], &SCCTRL.K[i]);
	        SCCTRL.PIArg[i].fAlpha = 1;
	    }
	}
//--
	CVC_UpdateParameter();
	CVC_UpdateGains();
//by JCNET
	SCC_UpdateGains();
//
	//-130806 BC_FSM_init(&BC_fsm);

#if IIVC_ENB
	IIR1_Creation(&InitINV.fltEdeCtrl, CC_tsSample, PRM_PCS[CTRL_VOLT_LPF].iValue);
	IIR1_Creation(&InitINV.fltEqeCtrl, CC_tsSample, PRM_PCS[CTRL_VOLT_LPF].iValue);

	IIVCCreate(&InitInvVControl_d, &InitInvVControl_d.PICTRL, &InitInvVControl_d.ARG, &InitInvVControl_d.RAMPInput, &InitInvVControl_d.RAMPOutput);
	IIVCCreate(&InitInvVControl_q, &InitInvVControl_q.PICTRL, &InitInvVControl_q.ARG, &InitInvVControl_q.RAMPInput, &InitInvVControl_q.RAMPOutput);

	CC_UpdateIINVGains();
	CC_UpdateIINVParameter();
#endif

	//IINV.SyncAcceptTime = 50. * CC_MICRO_UNIT;
	//IINV.SyncAcceptRadian = IINV.SyncAcceptTime * CC_fWe;
	CC_UpdateSyncToleranceTheta();

	CTRL_INV_DROOP_Create();
	CTRL_INV_PRC_Create();
	CTRL_INV_SYNC_Create();
	CTRL_BYP_SYNC_Create();
	CTRL_GEN_Create();
	CTRL_INV_PQC_Create();
	CTRL.INV.theta_ref = 0.;
	CTRL_INV_VUC_Create();
	CTRL_INV_VI_Create();
	CTRL_INV_SEAMLESS_Create();

#if DBUG_MODE == 1 || DBUG_MODE == 2
	//	DBUG.Erms = INVERTER.RATE.fPhase;
	DBUG.Theta = 0;
#endif
}

/*
 * call: PARAM_UpdateAll()
 * when: CC_PERIOD 파라미터 변경 시
 * when: PRM_PCS[CTRL_GRID_FS_RMS_CUTOFF_HZ].bChange || PRM_PCS[CTRL_INV_FS_RMS_CUTOFF_HZ].bChange) 파라미터 변경 시
 */
void CC_CalcFsRmsFilterShadow(void)
{
	float cutoff = PARAM_VAL(CTRL_INV_FS_RMS_CUTOFF_HZ);

	/*
	 * INV FASTSUM RMS
	 */
	IIR1_UpdateCoeff(&INVCTRL.fltFsConV[0], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&INVCTRL.fltFsConV[1], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&INVCTRL.fltFsConV[2], CC_tsSample, cutoff);

	/*
	 * INV Positive sequence FASTSUM RMS
	 */
	IIR1_UpdateCoeff(&INVCTRL.fltFsConVp[0], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&INVCTRL.fltFsConVp[1], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&INVCTRL.fltFsConVp[2], CC_tsSample, cutoff);

	cutoff = PARAM_VAL(CTRL_GRID_FS_RMS_CUTOFF_HZ);
	/*
	 * GRID FASTSUM RMS
	 */
	IIR1_UpdateCoeff(&EXCTRL.fltFsGridV[0], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&EXCTRL.fltFsGridV[1], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&EXCTRL.fltFsGridV[2], CC_tsSample, cutoff);

	/*
	 * GRID-BYP FASTSUM RMS
	 */
	IIR1_UpdateCoeff(&EXCTRL.fltFsBypV[0], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&EXCTRL.fltFsBypV[1], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&EXCTRL.fltFsBypV[2], CC_tsSample, cutoff);

	/*
	 * GEN FASTSUM RMS
	 */
	IIR1_UpdateCoeff(&EXCTRL.fltFsGenV[0], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&EXCTRL.fltFsGenV[1], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&EXCTRL.fltFsGenV[2], CC_tsSample, cutoff);

	/*
	 * BYP I FASTSUM RMS
	 */
	IIR1_UpdateCoeff(&EXCTRL.fltFsBypI[0], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&EXCTRL.fltFsBypI[1], CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&EXCTRL.fltFsBypI[2], CC_tsSample, cutoff);

	//	CC_bFsRmsFilterShadowing = TRUE;
}
#if 0
/*
 * call: c_int11()
 * when: CC_bFsRmsFilterShadowing is TRUE
 */
void CC_UpdateFsRmsFilterCoefficient(void)
{
	/*
	 * INV FASTSUM RMS
	 */
	IIR1_ChangeCoefficient(&INVCTRL.fltFsConV[0]);
	IIR1_ChangeCoefficient(&INVCTRL.fltFsConV[1]);
	IIR1_ChangeCoefficient(&INVCTRL.fltFsConV[2]);

	/*
	 * INV Positive sequence FASTSUM RMS
	 */
	IIR1_ChangeCoefficient(&INVCTRL.fltFsConVp[0]);
	IIR1_ChangeCoefficient(&INVCTRL.fltFsConVp[1]);
	IIR1_ChangeCoefficient(&INVCTRL.fltFsConVp[2]);

	/*
	 * GRID FASTSUM RMS
	 */
	IIR1_ChangeCoefficient(&EXCTRL.fltFsGridV[0]);
	IIR1_ChangeCoefficient(&EXCTRL.fltFsGridV[1]);
	IIR1_ChangeCoefficient(&EXCTRL.fltFsGridV[2]);

	/*
	 * GRID FASTSUM RMS
	 */
	IIR1_ChangeCoefficient(&EXCTRL.fltFsBypV[0]);
	IIR1_ChangeCoefficient(&EXCTRL.fltFsBypV[1]);
	IIR1_ChangeCoefficient(&EXCTRL.fltFsBypV[2]);

}
#endif


/*
 * call: PARAM_UpdateAll()
 * when: CC_PERIOD OR CTRL_FREQ_CUTOFF_HZ 파라미터 변경 시
 */
void CC_CalcFreqFilterShadow(void)
{
	float cutoff = PARAM_VAL(CTRL_FREQ_CUTOFF_HZ);
	IIR1_UpdateCoeff(&INVCTRL.fltFreq, CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&EXCTRL.fltFreq, CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&GRIDTHETA_AI.fltFreqW, CC_tsSample, cutoff);

	IIR1_UpdateCoeff(&EXCTRL.fltFreq_Byp, CC_tsSample, cutoff);
	IIR1_UpdateCoeff(&EXCTRL.fltFreqW_Byp, CC_tsSample, cutoff);

	//	CC_bFreqFilterShadowing = TRUE;
}
#if 0
/*
 * call: cint11()
 * when: CC_bFreqFilterShadowing is TRUE
 */
void CC_UpdateFreqFilterCoefficient(void)
{
	IIR1_ChangeCoefficient(&INVCTRL.fltFreq);
	IIR1_ChangeCoefficient(&EXCTRL.fltFreq);
	IIR1_ChangeCoefficient(&GRIDTHETA_AI.fltFreqW);

}
#endif
/*
 * call: PARAM_UpdateAll()
 * when: BATT_CONST_PWR_DCHG(파워제어 CutOffHz) 파라미터 변경시
 */
void CC_CalcPowerControlFilterShadow(void)
{
	IIR1_UpdateCoeff(&EXCTRL.fltAcPowerP, CC_tsSample,	PARAM_VAL(BATT_PCC_PQ_CUTOFF_HZ));
	IIR1_UpdateCoeff(&EXCTRL.fltAcPowerQ, CC_tsSample,	PARAM_VAL(BATT_PCC_PQ_CUTOFF_HZ));

	//	CC_bPowerControlFilterShadowing = TRUE;
}
#if 0
/*
 * call: cint11()
 * when: CC_bPowerControlFilterShadowing is TRUE
 */
void CC_UpdatePowerControlFilterCoefficient(void)
{
	IIR1_ChangeCoefficient(&EXCTRL.fltAcPowerP);
	IIR1_ChangeCoefficient(&EXCTRL.fltAcPowerQ);
}
#endif

/*
 * call: CC_UpdateParameter()
 * call: PARAM_UpdateAll()
 * when: INV_CAPACITY 또는 GRID_RATED_VOLT 파라미터 변경 시
 */
void CC_UpdateIdqRamp(void)
{
#if USE_FASTRTS_LIB_DIV
	RAMP_SetDelta(INVCTRL.pRampIdeRef, div(ACP.INV.RATE.Iph_pk*CC_tsCC, 10.));
	RAMP_SetDelta(INVCTRL.pRampIqeRef, div(ACP.INV.RATE.Iph_pk*PARAM_RAW_VAL(CTRL_CC_PERIOD), CC_IREF_RAMP_US));
	RAMP_SetDelta(INVCTRL.pRampIdeRef, div(ACP.INV.RATE.Iph_pk*PARAM_RAW_VAL(CTRL_CC_PERIOD), CC_IdREF_RAMP_US));
#if DOUBLE_CONTROL == 1
#else
	RAMP_SetDelta(INVCTRL.pRampIdeRef, div(ACP.INV.RATE.Iph_pk*CC_tsCC, 10.));
	RAMP_SetDelta(INVCTRL.pRampIqeRef, div(ACP.INV.RATE.Iph_pk*PARAM_RAW_VAL(CTRL_CC_PERIOD), CC_IREF_RAMP_US));
	RAMP_SetDelta(INVCTRL.pRampIdeRef, div(ACP.INV.RATE.Iph_pk*PARAM_RAW_VAL(CTRL_CC_PERIOD), CC_IdREF_RAMP_US));
#endif
#else

#if DOUBLE_CONTROL == 1
	//-RAMP_SetDelta(INVCTRL.pRampIdeRef, (ACP.INV.RATE.Iph_pk * CC_tsSample / 10.));
	RAMP_SetDelta(INVCTRL.pRampIqeRef,
			(ACP.INV.RATE.Iph_pk * (PARAM_RAW_VAL(CTRL_CC_PERIOD)* 0.5) /CC_IREF_RAMP_US));
	RAMP_SetDelta(INVCTRL.pRampIdeRef, (ACP.INV.RATE.Iph_pk * (PARAM_RAW_VAL(CTRL_CC_PERIOD) * 0.5) / CC_IdREF_RAMP_US));
#else
	RAMP_SetDelta(INVCTRL.pRampIdeRef, (ACP.INV.RATE.Iph_pk * CC_tsCC / 10.));
	RAMP_SetDelta(INVCTRL.pRampIqeRef, (ACP.INV.RATE.Iph_pk * PARAM_RAW_VAL(CTRL_CC_PERIOD) / CC_IREF_RAMP_US));
	RAMP_SetDelta(INVCTRL.pRampIdeRef, (ACP.INV.RATE.Iph_pk * PARAM_RAW_VAL(CTRL_CC_PERIOD) / CC_IdREF_RAMP_US));
#endif
#endif
}

/*
 * call: CC_UpdateParameter()
 * when: INV_CAPACITY 또는 GRID_RATED_VOLT 또는
 * 		 CTRL_QV_CTRL_RAMP_TIME || CTRL_FRT_IQ_RAMP_TIME || CTRL_COSPHI_P_PN_RAMP_TIME 파라미터 변경 시
 */
void CC_UpdateIdqDelayRamp(void)
{
#if USE_FASTRTS_LIB_DIV
	/*
	 * Change 10% of the rated power for {CTRL_QV_CTRL_RAMP_TIME} sec
	 * (Because QV power limit is 10 %) => Now 31%
	 */
	RAMP_SetDelta(INVCTRL.pRampIdeQV, div(ACP.INV.RATE.Iph_pk*0.3122 * PARAM_VAL(CTRL_CC_PERIOD), PARAM_VAL(CTRL_QV_CTRL_RAMP_TIME) * 1000 * 1000));

	/*
	 * Change 0.95 PF for {CTRL_COSPHI_P_PN_RAMP_TIME} sec
	 */
	RAMP_SetDelta(INVCTRL.pRampIdePPN, div(ACP.INV.RATE.Iph_pk*0.3122 * PARAM_VAL(CTRL_CC_PERIOD), PARAM_VAL(CTRL_COSPHI_P_PN_RAMP_TIME) * 1000 * 1000));

	/*
	 * Change rated current for {CTRL_COSPHI_P_PN_RAMP_TIME} msec
	 */
	RAMP_SetDelta(INVCTRL.pRampIqeRefDelay, (ACP.INV.RATE.Iph_pk * PARAM_VAL(CTRL_CC_PERIOD)/ PARAM_VAL(CTRL_FRT_IQ_RAMP_TIME) * 1000));
#else

#if DOUBLE_CONTROL == 1
	RAMP_SetDelta(INVCTRL.pRampIdeQV,
			(ACP.INV.RATE.Iph_pk * 0.3122
					* ((float) PARAM_VAL(CTRL_CC_PERIOD)) * 0.5)/(float)PARAM_VAL(CTRL_QV_CTRL_RAMP_TIME) * 1000 * 1000);
			RAMP_SetDelta(INVCTRL.pRampIdePPN, (ACP.INV.RATE.Iph_pk * 0.3122 * ((float)PARAM_VAL(CTRL_CC_PERIOD)) * 0.5) / (float)PARAM_VAL(CTRL_COSPHI_P_PN_RAMP_TIME) * 1000 * 1000);
			RAMP_SetDelta(INVCTRL.pRampIqeRefDelay, (ACP.INV.RATE.Iph_pk * ((float)PARAM_VAL(CTRL_CC_PERIOD)) * 0.5) / (float)PARAM_VAL(CTRL_FRT_IQ_RAMP_TIME) * 1000);
#else
					RAMP_SetDelta(INVCTRL.pRampIdeQV, (ACP.INV.RATE.Iph_pk * 0.3122 * (float) PARAM_VAL(CTRL_CC_PERIOD))/ (float)PARAM_VAL(CTRL_QV_CTRL_RAMP_TIME) * 1000 * 1000);
					RAMP_SetDelta(INVCTRL.pRampIdePPN, (ACP.INV.RATE.Iph_pk*0.3122 * (float)PARAM_VAL(CTRL_CC_PERIOD)) / (float)PARAM_VAL(CTRL_COSPHI_P_PN_RAMP_TIME) * 1000 * 1000);
					RAMP_SetDelta(INVCTRL.pRampIqeRefDelay, (ACP.INV.RATE.Iph_pk * (float)PARAM_VAL(CTRL_CC_PERIOD)) / (float)PARAM_VAL(CTRL_FRT_IQ_RAMP_TIME) * 1000);
#endif

#endif
}

Bool CC_GetCVCEnable()
{
	if( IVC.uStatus == CVC_ENABLE )
		return TRUE;
	else
		return FALSE;
}
