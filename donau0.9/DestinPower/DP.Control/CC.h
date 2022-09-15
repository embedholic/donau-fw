/*
 * CC.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef CC_H_
#define CC_H_

#include <xdc/runtime/Assert.h>
#include "LGS_Common.h"
#include "MathConst.h"
#include "PI.h"
#include "FILTER.h"
#include "PWM.h"
#include "RAMP.h"
#include "FastRms.h"
#include "EADC.h"
#include "SYSTEM.h"
#include "CTRL_INV_PRC.h"
#include "CTRL_INV_DROOP.h"
#include "CTRL_INV_PQC.h"
#include "CTRL_INV_SYNC.h"
#include "CTRL_BYP_SYNC.h"
#include "CTRL_INV_VUC.h"
#include "CTRL_INV_VI.h"
#include "CTRL_INV_SEAMLESS.h"
#include "Prm_pcs.h"
//#include "SagEventHistory.h"
//#include "CTRL_BYP_EVT_OPERATION.h"



#if DBUG_MODE != 0
#define CC_EXECUTION_TIME 0
#define CC_EXECUTION_PERIOD	0
#endif

#define CC_DEAD_TIME 2.97e-6

// 0 ==> Macro
// 1 ==> Lookup Table
#define CC_SINE_TABLE_USE	0

// 0 ==> EOC에 의한 DPRAM 인터럽트가 한번 발생
// 1 ==> EOC에 의한 DPRAM 인터럽트가 두번 발생
#define CC_INTERRUPT_DOUBLE	0

#define CC_LINE_NUM 3

//-150529 #define CC_PERIOD_MIN 200 // : 5KHz
#define CC_PERIOD_MIN 80  // : 12.5KHz
#define CC_PERIOD_MAX 333 // : 3KHz

#define CC_FASTSUM_PRECISION 		1000
#define CC_FASTSUM_PRECISION_INV 0.001

#define CC_MICRO_UNIT 1.0e-6
#define CC_PV_I_FLT_CUTOFF_HZ 50.
#define CC_PV_I_FLT2_CUTOFF_HZ 0.3

#define CC_DC_V_FOR_CC_FLT_CUTOFF_HZ 50.
#define CC_DC_V_FLT_CUTOFF_HZ 5.
#define CC_DC_V_FLT2ND_CUTOFF_HZ 0.2

#define CC_EQE_2ND_FIL_CUTOFF_HZ 1.
#define CC_IQE_FLT_CUTOFF_HZ 100.
#define CC_POUT_FLT_CUTOFF_HZ 0.3

#define CC_GRID_FREQ_OMEGA_CUTOFF 10
#define CC_ACPOWER_CUTOFF_HZ 100
#define CC_ACPOWER_CUTOFF_HZ_2ND 0.5

#define CC_BYP_I_CUTOFF_HZ 1

enum { CVC_DISABLE = 0, CVC_ENABLE, CVC_INIT_VOLTAGE_BUILD };
enum { CC_PLL_NORMAL = 0, CC_PLL_VOLTAGE_BUILD };

enum CCI_Status_e {CCI_OCCHECK = 0, CCI_NOGATING, CCI_DELAY};
enum FRT_Status_e {FRT_NORMAL = 0, FRT_ASYMMETRIC = 1, FRT_SYMMETRIC = 2};
enum GPC_Status_e {GPC_GATING = 0, GPC_NOGATING = 1};

#define CC_MODE_DBUG_ENB 1

typedef struct _BattBox
{
	Bool bActive;
	Int16 batteryV;
	Int16 timeOutCnt_sec;
	// 5초 이상 통신이 안될 경우 bActive -> false.
}BatteryBox;

typedef struct _ThetaDetect
{
	float fFbk;			// Feedback value
	PICon *pPI;			// Internal : PI Controller
	PIArg *pARG;		// Internal : PI Controller Arguments
	float fPIOut;		// Internal : PI Controller Output
	float fFreqW;		// Freq. w ==> 2*PI*f
	iir1st fltFreqW; 	// omega
	float fFreq;		// Freq. ==> f
	float pfRadian;
	float fRadian;
	Bool bRMSChangeEnable;
	int ThetaPolarity_r;
	int ThetaPolarity_s;
	int ThetaPolarity_t;
} ThetaDetect;

typedef struct _ExternalLineCtrl
{
	float pccEa;
	float pccEb;
	float pccEc;
	float pccEds;
	float pccEqs;
	float pccEde;
	float pccEqe;

	/*+Added 130622@for Instantaneous power calculation */
	float trIa;
	float trIb;
	float trIc;
	float trIds;
	float trIqs;

	//+Added 130826
	float bypEa;
	float bypEb;
	float bypEc;
	float bypEds;
	float bypEqs;
	float bypEde;
	float bypEqe;
	float bypIa;
	float bypIb;
	float bypIc;
	float bypIds;
	float bypIqs;
	float bypIde;
	float bypIqe;

	// 150828
	float genEa;
	float genEb;
	float genEc;
	float genEds;
	float genEqs;
	float genEde;
	float genEqe;

	float GridPowerP;
	float LoadPowerP;

	iir1st fltAcPowerP; // 유효전력 Lowpass filter
	iir1st fltAcPowerQ; // 무효전력 Lowpass filter
	iir1st fltAcPowerP_2nd; // 유효전력 Lowpass filter
	iir1st fltAcPowerQ_2nd; // 무효전력 Lowpass filter
	/*+=================================================*/
	iir1st fltEde;
	iir1st fltEqe;

	/* 14120 Power reference -> Current ref */
	float fltPccEqeToCurrentReference;
	float fltPccEdeToCurrentReference;

	iir1st fltEde_byp;
	iir1st fltEqe_byp;

	iir1st fltEde_gen;
	iir1st fltEqe_gen;

	float SinTheta;
	float CosTheta;

	float SinTheta_Byp;//
	float CosTheta_Byp;//

	float SinTheta_Gen;//
	float CosTheta_Gen;//

	AllPassShiftFilter fltEaDelay90; // Positive Seq. 관련
	AllPassShiftFilter fltEbDelay90; // Positive Seq. 관련
	AllPassShiftFilter fltEcDelay90; // Positive Seq. 관련

	float Eap; // 파지티브 시퀀스 계산용??
	float Ebp;
	float Ecp;
	float enp_mean;

	float EaDelay90;
	float EbDelay90;
	float EcDelay90;

	iir1st	fltFsGridV[CC_LINE_NUM];
	iir1st	fltFsBypV[CC_LINE_NUM];//+130826
	iir1st	fltFsGenV[CC_LINE_NUM];//+150828

	//	16.07.01
	iir1st	fltFsBypI[CC_LINE_NUM];

	float VqPU;//?

	iir1st fltFreq;
	iir1st fltFreqW; // omega

	iir1st fltFreq_Byp;
	iir1st fltFreqW_Byp; // omega

	iir1st fltFreq_Gen;

} ExternalCtrl;

typedef struct _ConCtrl
{
	float Ea;
	float Eb;
	float Ec;

	float Eab;
	float Ebc;
	float Eca;

	float Eds;
	float Eqs;
	float Ede;
	float Eqe;
	float EPeakInverse;
	iir1st fltEde;
	iir1st fltEqe;
	iir1st fltEqe2nd;

	float Ia;
	float Ib;
	float Ic;

	float Ids;
	float Iqs;

	float Ide;
	float Iqe;			// 유효전력
	//<--- values page
	float IdeRef;//d축전류 레퍼런스 페이지 11.
	float IqeRef;//q축전류 ""
	float IdeRef_prev;

	iir1st fltIde;
	iir1st fltIqe;
	iir1st fltIa;
	iir1st fltIb;
	iir1st fltIc;
	float IdsRef;
	float IqsRef;
	float InsRef;
	float VdeIntegOut;
	float VqeIntegOut;

	float VdePIOut;
	float VqePIOut;

	float VdeEmfComp;
	float VqeEmfComp;

	float IdeErr;
	float IqeErr;

	float IaRef;
	float IbRef;
	float IcRef;
	float KpCC;
	float KiCC;
	float KiTCC;
	float VdeRef;
	float VqeRef;

	float VdsRef;
	float VqsRef;

	float VaRef;
	float VbRef;
	float VcRef;
	float VanRef;
	float VbnRef;
	float VcnRef;
	float VsnRef;
	PwmOnTime PwmOnTime;
	float SinTheta;
	float CosTheta;

#if 0 //by JCNET
	// 14.2.7 for Line To Line V 30도 위상차 보상
	float RefTheta;
	float RefSinTheta;
	float RefCosTheta;
#endif

	float Li; //values page 2.
	UInt16 Ia_OC_Count;
	UInt16 Ib_OC_Count;
	UInt16 Ic_OC_Count;

	float Pout;
	iir1st fltPout;
	iir1st fltPout2nd;

	Ramp* 	pRampIdeRef;

	/*
	 * To support BDEW
	 */
	AllPassShiftFilter	fltEaDelay90;
	AllPassShiftFilter	fltEbDelay90;
	AllPassShiftFilter	fltEcDelay90;

//	float	Eap; //파지티브 시퀀스용??
//	float	Ebp;
//	float	Ecp;
	float	EaDelay90;
	float	EbDelay90;
	float	EcDelay90;

	iir1st	fltFsConV[CC_LINE_NUM];
	iir1st	fltFsConVp[CC_LINE_NUM];

	enum FRT_Status_e FrtMode;
	enum FRT_Status_e prevFrtMode;
	Ramp*	pRampIdeQV;
	Ramp*	pRampIdePPN;

	enum CCI_Status_e CCI_Status;
	enum CCI_Status_e CCI_2ndOC_Status;
	unsigned int CCI_DelayCnt;
	unsigned int CCI_OCCheckCnt;
	unsigned int CCI_NoGatingCnt;
	unsigned int CCI_2ndOC_NoGatingCnt;

	Ramp* pRampIqeRef;
	Ramp* pRampIqeRefDelay;

	iir1st	fltFreq;
	int		FrtModeVote;

	float fCurrentLimit; //
	float fPowerFactor;

	unsigned int Batt_OVCheckCnt;
	unsigned int Batt_UVCheckCnt;
	unsigned int DcSideAbnormalCnt;

	// 151109 EXCTRL.fltEqe.fOut = 310.23094 = byp30
	float fEqeCalcVoltage;
} InverterCtrl;

typedef struct _ConVoltCtrl
{
    float iqe_ref; // by JCNET
	float fRef;
	float fRef_command;
	PICon *pPI;
	PIArg *pARG;
	Gain  K;
	float fPIOut;
	Uns	  uStatus;
	Bool  bRAMPInputInitEnb;
	Ramp  *pRAMPInput;
	Ramp  *pRAMPOutput;
} InvVoltCtrl;

//13.3.23 added
typedef struct _ConCurrentCtrl
{
	float fRef;
	float fRefCommand;
	PICon *pPI;
	PIArg *pARG;
	Gain  K;
	float fPIOut;

	Ramp  *pRAMPOutput;

} InvCurrentCtrl;

#if 0
//JCNET added 2022/05/31
typedef struct _ConverterCurrentCtrl
{
    float ide_ref;
    float iqe_ref;
    float vde_ref;
    float vqe_ref;

    PICon PI_D;
    PIArg PIArg_D;
    PICon PI_Q;
    PIArg PIArg_Q;

    float fRef_D;
    float fRefCommand_D;
    PICon *pPI_D;
    PIArg *pARG_D;
    Gain  K_D;
    float fPI_DOut_D;

    float fRef_Q;
    float fRefCommand_Q;
    PICon *pPI_Q;
    PIArg *pARG_Q;
    Gain  K_Q;
    float fPI_DOut_Q;
} ConCurrentCtrl;
#endif

typedef struct _InvPowerCtrl
{
	float fRefP;
	float fRefQ;
	float fRefBefore;

	float fRef_limit;

	float fPowerP; // 유효전력
	float fPowerQ; // 무효전력

	// P PI Controller
	PICon *pPI;
	PIArg *pARG;
	Gain  K;
	float fPIOut;
	Ramp  *pRAMPOutput;

	// Q PI Controller
	PICon *pPI_Q;
	PIArg *pARG_Q;
	Gain  K_Q;
	float fPIOut_Q;
	Ramp  *pRAMPOutput_Q;

	short int bTrig_softLevel;
	short int bTrig_vdcLimit;
	unsigned int uIdleTime;
	unsigned int uTriggerTime;
	int uState;


} InvPowerCtrl;

typedef struct _BattCtrl
{
	float DCLinkV; 			//\CB  이후 전압
	iir1st fltDCLinkV;		//values page2.
	iir1st fltDCLinkV2nd;	//values page2.
	float I;
	float rawI;
	iir1st fltI;			//values page2.
	iir1st fltI2;			//values page2.
	float DCBattV; 			//\CB 이전 전압
	iir1st fltDCBattV;
	iir1st fltDCBattV_2nd;    //+13.3.5 for BC
	iir1st fltDCBattVForCC;
	float OCLevel;
	float fIMaxPermissible;
	float fPower;
	float fPowerFilterd_1st; //values page2.
	//-float fPowerFilterd_2nd;
	float RefCell;
	UInt16 uBattIFaultCount;

	BatteryBox bmsInfo;
} BattCtrl;

typedef struct _CtrlOption
{
	Bool bIsometer;
	Bool bPvFuse;
	Bool bPvTempSensor;
	Bool bPvRefCell;
	Bool bPT1000Sensor;
	Bool bWindSensor;
	Uns uPWM;
} CtrlOption;

#if IIVC_ENB
typedef struct _InitInvVoltCtrl
{
	float fRef;
	PIArg *pARG;
	PIArg ARG;
	Gain  K;
	PICon *pPI;
	PICon PICTRL;
	float fPIOut;
	float fOut;
	Ramp  *pRAMPInput;
	Ramp  RAMPInput;
	Ramp  *pRAMPOutput;
	Ramp  RAMPOutput;
} InitInvVoltCtrl;

typedef struct _InitInverter
{
	float VdeRef;
	float VqeRef;
	float VdsRef;
	float VqsRef;
	float VaRef;
	float VbRef;
	float VcRef;
	float EdeCtrl;
	float EqeCtrl;
	float SyncAcceptTime;
	float SyncAcceptRadian;
	float DiffThetaTime; 	// values page 2.
	float DiffThetaRadian;
	iir1st fltEdeCtrl;
	iir1st fltEqeCtrl;
	float RefTheta;
	float SinRefTheta;
	float CosRefTheta;
} InitInverter;
#endif

typedef struct _LoadInfo
{
	float fIQE_PEAK;
	float fIQE_PEAK_INVERSE;
}LoadInfo;

typedef enum { SRC_LOCAL=1, SRC_REMOTE } SRC_ORDER;
typedef enum { PFLOW_STANDBY=0, PFLOW_CHARGE, PFLOW_DISCHARGE } POWER_FLOW;
typedef enum { CC_MODE_CP = 1, CC_MODE_CCCV, CC_MODE_CC, CC_MODE_CV } CC_MODE;

typedef struct _ControllInfo
{
	POWER_FLOW powerFlow;
	POWER_FLOW prevPowerFlow;
	Bool bNeedInit;
	CC_MODE ccMode;	// 제어 모드
	SRC_ORDER srcOder;
}BC_STATE;

typedef struct _InverterRateBlock
{
	float		P3;
	float		Q3;
	float		Freq;
	float		Omega;
	float		Vll;
	float		Vph;
	float		Vph_pk;
	float		Vph_pk_gi;
	float		Iph;
	float		Iph_pk;
	float		Z;
} InverterRateBlock;

typedef struct _InverterFilterUnit
{
	iir1st iir1st_p;
	iir1st iir1st_q;
} InverterFilterUnit;

typedef struct _InverterThetaBlock
{
	float		rad;
	float		sync_accept_rad;
	float		diff_rad;
	float		diff_time;
} InverterThetaBlock;

typedef struct _InverterSystem
{
	InverterRateBlock RATE;
	InverterFilterUnit	FILTER;
	InverterThetaBlock	THETA;
	float		vab;
	float		vbc;
	float		vca;
	float		va;
	float		vb;
	float		vc;
	float		va_p;
	float		vb_p;
	float		vc_p;
	float		va_n;
	float		vb_n;
	float		vc_n;
	float		ia;
	float		ib;
	float		ic;
	float		vds;
	float		vqs;
	float		vds_p;
	float		vqs_p;
	float		vds_n;
	float		vqs_n;
	float		vds_ll;
	float		vqs_ll;
	float		ids;
	float		iqs;
	float		p;
	float		p_f;
	float		p_f_lim;
	float		q;
	float		q_f;
	float		q_f_lim;
	float 		tsw_div_vdc;
} InverterSystem;

typedef struct _TransformerRateBlock
{
	float		Z;
	float		RATIO;
} TransformerRateBlock;

typedef struct _CouplingTransformer
{
	TransformerRateBlock RATE;
} CouplingTransformer;

typedef struct _InverterCapacitor
{
	float		ia;
	float		ib;
	float		ic;
	UInt16 		ia_OC_Count;
	UInt16 		ib_OC_Count;
	UInt16 		ic_OC_Count;
} InverterCapacitor;

typedef struct _PCCRateBlock
{
	float		Freq;
	float		Omega;
	float		Vll;
	float		Vph;
	float		Vph_pk;
	float		Iph;
	float		Iph_pk;
	float		Z;
} PCCRateBlock;

typedef struct _PCCSystem
{
	PCCRateBlock RATE;
	float		va;
	float		vb;
	float		vc;
	float		ia;
	float		ib;
	float		ic;
	float		vds;
	float		vqs;
	float		ids;
	float		iqs;
	float		v_pk;
} PCCSystem;

typedef struct _BYPRateBlock
{
	float		Freq;
	float		Omega;
	float		Vll;
	float		Vph;
	float		Vph_pk;
} BYPRateBlock;

typedef struct _BypassSystem
{
	BYPRateBlock RATE;
	float		va;
	float		vb;
	float		vc;
	float		vds;
	float		vqs;
	float		v_pk;

	float		Ia;
	float		Ib;
	float		Ic;

	InverterThetaBlock	THETA;

	UInt16 Ia_OC_Count;
	UInt16 Ib_OC_Count;
	UInt16 Ic_OC_Count;

} BypassSystem;

typedef struct _GeneratorSystem
{
	float		va;
	float		vb;
	float		vc;
	float		vds;
	float		vqs;
	float		v_pk;
} GeneratorSystem;

typedef struct _AC_Panel
{
	InverterSystem INV;
	CouplingTransformer TR;
	InverterCapacitor CAP;
	PCCSystem PCC;
	BypassSystem BYP;
	GeneratorSystem GEN;
} AC_Panel;

typedef struct _ProtectionLevelInverter
{
	float	Idqs_MAX;
	float	Idqs_MIN;

} ProtectionLevelInverter;

typedef struct _InverterSeamlessBloc
{
	Bool g_enb;
	Bool pcc_blackout_enb;
	float pcc_ov_level_instant;
	float pcc_uv_level_instant;

	float pcc_ov_level2_instant;
	float pcc_uv_level2_instant;

	Odt		*odt;
	Odt		ODT;

	Bool enb_for_BEO;
} InverterSeamlessBloc;

typedef struct _InverterControl
{
	DroopBlock DROOP;
	PResonatBlock PRC_Ids;
	PResonatBlock PRC_Iqs;
	PResonatBlock PRC_Vds;
	PResonatBlock PRC_Vqs;
	ProtectionLevelInverter	PROTECTION;
	SyncBlock SYNC;
	PQControlBlock PQC;
	VucBlock VUC;
	ViBlock VI;

	InverterSeamlessBlock SEAMLESS;
	Uns	ctrl_mode;
	Bool ctrl_mode_change_enb;
	CMODE	operation_mode;
	Bool grid_connected_enb;
	Uns	filter_mode;

	float theta_ref;
	float sin_theta_ref;
	float cos_theta_ref;
	float vds_ref;
	float vqs_ref;
	float vds_ref_vi;
	float vqs_ref_vi;
	float vds_ref_vi_vuc;
	float vqs_ref_vi_vuc;
	float ids_ref_lim;
	float iqs_ref_lim;
	float ids_ref;
	float iqs_ref;
	float va_ref;
	float vb_ref;
	float vc_ref;
	float va_ref_ad;
	float vb_ref_ad;
	float vc_ref_ad;
	float va_cap_current_compen;
	float vb_cap_current_compen;
	float vc_cap_current_compen;
	float vi_dc; /*+ 150209 June */
} InverterControl;

typedef struct _BypassControl
{
	BypassSyncBlock SYNC;
} BypassControl;

typedef struct
{
	UInt16 uOCSkipCnt;
	UInt16 uOCSkipCntMax;
	UInt16 bPwmOff;

//	float uVSI_PR_keep_ms;
	float csi_to_vsi_oneShot;
	float csi_to_vsi_oneShot_keep_ms;
}IslandingRightThrow;

typedef struct
{
	Bool bEnable;
	UInt16 iSkipFreqFaultCnt;
	float skip_ms;
}GridReconnectRightThrow;

typedef struct _Controller
{
	InverterControl INV;
	BypassControl BYP;
	BypassControl GEN;
	IslandingRightThrow IRT;
	GridReconnectRightThrow GRT;  // 160912
} Controller;

//++JCNET
typedef struct _SagCompenController_tag {
    float Ea,Eb,Ec;
    float Ia,Ib,Ic;

    float SinTheta, CosTheta;
    float RefTheta;

    float Vqe_ref;
    float RefSinTheta, RefCosTheta;

    float Eds,Eqs, Ens;
    float Ede,Eqe, Ene;

    iir1st fltEde, fltEqe, fltEne, fltFreq;
    iir1st fltEqe2nd;

    PICon  PICon[3]; // D,Q,N
    PIArg  PIArg[3];
    Gain   K[3];
    float  fPIOut[3];
    float  I_fact_max, O_fact_max, I_lmt;
    float  Prate;
    float  Vdc_max;
    float  factor_prate;
    float  factor_vdc;
    float  Vd_comp0;
    float  Vq_comp0;
    float  Vn_comp0;
    float  Vd_comp1;
    float  Vq_comp1;
    float  Vn_comp1;
    float  Vd_comp2;
    float  Vq_comp2;
    float  Vn_comp2;
    float  Vds_comp2;
    float  Vqs_comp2;
    float  Vns_comp2;
    float  VaRef, VbRef, VcRef;
    PwmOnTime PwmOnTime;

} SagCompenCtrl;
//--
float CC_GetWe(void);
void CC_Create( void );
void CC_UpdateGains( void );
void CC_UpdateParameter( void );

#if IIVC_ENB
void CC_UpdateIINVParameter( void );
void CC_UpdateIINVGains(void);
#endif
Bool CC_SyncOK( void );
Bool CC_VoltageGenerateOK(void);
void CC_UpdateSyncToleranceTheta(void);
void CC_UpdatePLLGains( Uns uMode );

void CC_UpdateVoltageFilterCoefficient( void );
void CC_CalcVoltageFilterShadow();
void CC_UpdateFilterCoefficient( void );
void CC_CalcFilterShadow( void );
void CC_UpdateULParameters( void );
void CC_CalcFsRmsFilterShadow(void);
void CC_CalcFreqFilterShadow(void);
void CC_UpdateFsRmsFilterCoefficient(void);
void CC_UpdateFreqFilterCoefficient(void);
void CC_CalcPowerControlFilterShadow(void);
void CC_UpdatePowerControlFilterCoefficient(void);

float CVC_GetReference(void);
Uns CVC_GetStatus(void);
void CVC_UpdateConEPeakInverse(void);
void CVC_UpdateParameter(void);
void CVC_UpdateGains(void);
void CVC_ChangeController( float alpha );


float CC_GetFreq( ThetaDetect *this );
void CC_InitLoadInfo();
float CC_GetCurrentLoadI();
void CC_PCC_Gating();
void BATT_UpdateParameter();
void CTRL_UpdateOptions();

void CC_CalcGridFilterShadow(void);
void CC_UpdateGridFilterCoefficient(void);
//void CC_UpdateAPS(void);
void CC_UpdateIdqRamp(void);
void CC_UpdateIdqDelayRamp(void);
void CC_AddInfoNode();
void CC_IVC_OutRampInit();
void CC_BypOmegaReset();
extern void c_int11(int iCallOrder);
void CC_BCStateUpdate();
Bool CC_GetSoftLevel();
void CC_BCTransIdle();
float CC_GetPowerRefP(void);
float CC_GetPowerRefQ(void);
float CC_GetActualPowerP(void);
float CC_GetActualPowerQ(void);
float CC_GetActualPowerP_2nd(void);
float CC_GetActualPowerQ_2nd(void);
float CC_GetPowerFactor(void);
Bool CC_GetCVCEnable();

extern ThetaDetect GRIDTHETA, CONTHETA, GRIDTHETA_AI/*for AntiIslanding*/, GRID_BYP_THETA, GEN_THETA;
extern ThetaDetect SCCTHETA; // by JCNET
extern ExternalCtrl EXCTRL;
extern InverterCtrl INVCTRL;
extern BattCtrl BATCTRL;
extern CtrlOption OPTION;
extern BC_STATE BCState;
extern AC_Panel	ACP;
extern Controller	CTRL;
extern InvVoltCtrl IVC;

extern SagCompenCtrl SCCTRL;//by JCNET
extern float CC_tsCC;
extern float CC_tsSample;
extern float fPCCPowerRef;

extern InvPowerCtrl PCC;

extern Bool bSagEvent;

#endif /* CC_H_ */
