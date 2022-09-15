/******************************************************************************
 *
 * 파일명    : EADC.c
 * 작성자    : 신요섭
 * 목적      : External ADC
 * 사용방식  : Crate() -> public functions
 * 사용파일  :
 * 제한사항  :
 * 오류처리  :
 * 이력사항
 * 			2012. 11/05 신요섭
 * 			1. 최초 생성
 */

#include <xdc/runtime/Assert.h>
#include <string.h>

#include "EADC.h"
#include "XINTF_ADC.h"
#include "LGS_Common.h"
#include "MathConst.h"
#include "Prm_pcs.h"
#include "DBUG.h"
#include "CC.h"
#include "CTRL_GEN.h"
#include "MCCB.h"

//#define STABLEEN

/* --------------------- DEFINE --------------------- */
#define ADC_MASKBIT 0xFFFF
#define ADC_USEBIT_HALF 0x7FFF
#define ADC_USEBIT 0xFFFF
#define ADC_USEBIT_INVERSE (1.5259021896696421759365224689097e-5)
#define ADC_SIZE 16

public EAdc EADC;
static AdcItem adcInvEa;//
static AdcItem adcInvEb;//
static AdcItem adcInvEc;//
static AdcItem adcOutEa;//
static AdcItem adcOutEb;//
static AdcItem adcOutEc;//
static AdcItem adcGenEa; 			// IN V, GEN V MUX Control (Rev.G)
static AdcItem adcGenEb;
static AdcItem adcGenEc;
static AdcItem adcDCLinkV;//
static AdcItem adcBatV; //
static AdcItem adcInEa;//
static AdcItem adcInEb;//
static AdcItem adcInEc;//
static AdcItem adcOutIa;//		    // DCV,BATV,BATI Mux Control (Rev.G)
static AdcItem adcOutIb;//
static AdcItem adcOutIc;//
static AdcItem adcBatI;//
static AdcItem adcRevB_a;//
static AdcItem adcRevC_1; // Icap a
static AdcItem adcRevC_2; //
static AdcItem adcRevC_3; //
static AdcItem adcInvIa;//
static AdcItem adcInvIb;//
static AdcItem adcInvIc;//
static AdcItem adcBypIa;//
static AdcItem adcBypIb;//
static AdcItem adcBypIc;//

/*
 * Funtions Definition
 */
private void EADC_UpdateGain(AdcItem *_this, float Gain);
private void EADC_ADCItem_Init(AdcItem *_this, float Max, float Min, float Offset, float Gain);

/*
 *	ADCItem 초기화
 */
private void EADC_ADCItem_Init(AdcItem *_this, float Max, float Min, float Offset, float Gain)
{
	//아래는 범위가 MIX MAX가 -1100~1100 일 경우.
	//ADC 16비트의 분해능은 65535. 표현 범위는 총 2200이고 65535로 나누어 보면 value 1 당 얼마의 tangent를 갖는지 알수 있음.
	//2200 / 65535하면 ADC값 1은 0.0335라는 것을 알 수 있음.
	//ADC값이 -0x8000 이면 -0x8000 x 0.0335 하면 약 -1100이 나옴.

	_this->fTangent = (Max - Min) * ADC_USEBIT_INVERSE;
	_this->fOffset = Offset;
	_this->fGain = Gain;
}

/*
 * Voltage ADC 아이템 초기화.
 * 직류 Voltage Adc 아이템을 초기화 한다.
 * 외부 호출 : parameter module: 인버터의 Capacity가 변경되었을 때
 */
public void EADC_VoltageAdcItemInit(void)
{
	float fOffset, fGain;
	float fOffsetVarient, fGainVarient;
	float fVMax, fVMin;

	/*
	 * DC Link V Item Create
	 */
	fVMax = EADC_DCLINK_V_MAX;
	fVMin = -EADC_DCLINK_V_MAX;

	fOffsetVarient = PRM_PCS[OFS_DCLINK_VOLTAGE].fIncDec;
	fGainVarient = PRM_PCS[GN_DCLINK_VOLTAGE].fIncDec;
	fOffset = fOffsetVarient * PRM_PCS[OFS_DCLINK_VOLTAGE].iValue;
	fGain = fGainVarient * PRM_PCS[GN_DCLINK_VOLTAGE].iValue;
	EADC_ADCItem_Init(EADC.pAdcDCLinkV, fVMax, fVMin, fOffset, fGain);

	/*
	 * Battery V Item Create
	 */
	fVMax = EADC_BAT_V_MAX;
	fVMin = -EADC_BAT_V_MAX;
	fOffsetVarient = PRM_PCS[OFS_BATT_VOLTAGE].fIncDec;
	fGainVarient = PRM_PCS[GN_BAT_VOLTAGE].fIncDec;
	fOffset = fOffsetVarient * PRM_PCS[OFS_BATT_VOLTAGE].iValue;
	fGain = fGainVarient * PRM_PCS[GN_BAT_VOLTAGE].iValue;
	EADC_ADCItem_Init(EADC.pAdcDCBattV, fVMax, fVMin, fOffset, fGain);

}

/*
 * Current ADC 아이템 초기화
 * 모든 전류 Adc 아이템을 초기화 한다.
 * 외부 호출 : parameter module: 인버터의 Capacity가 변경되었을 때
 */
public void EADC_CurrentAdcItemInit(void)
{
	// 출력 전류
	float fOffsetOutIa, fOffsetOutIb, fOffsetOutIc;
	float fGainOutIa, fGainOutIb, fGainOutIc;
	// 배터리 전류
	float fOffsetBatI;
	float fGainBatI;
	// 인버터 전류
	float fOffsetInvIa, fOffsetInvIb, fOffsetInvIc;
	float fGainInvIa, fGainInvIb, fGainInvIc;
	// 부하 전류
	float fOffsetLoadIa, fOffsetLoadIb, fOffsetLoadIc;
	float fGainLoadIa, fGainLoadIb, fGainLoadIc;
	// etc.
	float fOffsetVarient;
	float fGainVarient;
	// Byp 전류
	float fOffsetBypIa, fOffsetBypIb, fOffsetBypIc;
	float fGainBypIa, fGainBypIb, fGainBypIc;

	/*
	 * Calc Offset
	 */
	fOffsetVarient = PRM_PCS[OFS_DCLINK_VOLTAGE].fIncDec;

	// Grid 전류에는 offset을 두지 않는다.
	fOffsetOutIa = fOffsetOutIb = fOffsetOutIc = 0.;
	fOffsetBypIa = fOffsetBypIb = fOffsetBypIc = 0.;
	fOffsetBatI =  fOffsetVarient * PRM_PCS[OFS_BAT_CURRENT].iValue;
	fOffsetInvIa = fOffsetVarient * PRM_PCS[OFS_INV_I_L1].iValue;
	fOffsetInvIb = fOffsetVarient * PRM_PCS[OFS_INV_I_L2].iValue;
	fOffsetInvIc = fOffsetVarient * PRM_PCS[OFS_INV_I_L3].iValue;
	fOffsetLoadIa = 0.;
	fOffsetLoadIb = 0.;
	fOffsetLoadIc = 0.;

	/*
	 * Calc Gain
	 */
	fGainVarient = PRM_PCS[GN_INV_I_L1].fIncDec; // 모든 게인의 최소증감 단위는 같다.

	fGainOutIa = PRM_PCS[GN_GRID_I_L1].iValue * fGainVarient; // FIXME 파라미터 생성 할 것.
	fGainOutIb = PRM_PCS[GN_GRID_I_L2].iValue * fGainVarient;
	fGainOutIc = PRM_PCS[GN_GRID_I_L3].iValue * fGainVarient;
	fGainBatI = fGainVarient * PRM_PCS[GN_BAT_CURRENT].iValue;
	fGainInvIa = fGainVarient * PRM_PCS[GN_INV_I_L1].iValue;
	fGainInvIb = fGainVarient * PRM_PCS[GN_INV_I_L2].iValue;
	fGainInvIc = fGainVarient * PRM_PCS[GN_INV_I_L3].iValue;
	fGainLoadIa = fGainVarient * PRM_PCS[GN_LOAD_I_L1].iValue;
	fGainLoadIb = fGainVarient * PRM_PCS[GN_LOAD_I_L2].iValue;
	fGainLoadIc = fGainVarient * PRM_PCS[GN_LOAD_I_L3].iValue;
	fGainBypIa = fGainBypIb = fGainBypIc = 10000. * fGainVarient;

	EADC_ADCItem_Init(EADC.pAdcTrIa, EADC_OUTPUT_I_MAX, -EADC_OUTPUT_I_MAX, fOffsetOutIa, fGainOutIa );
	EADC_ADCItem_Init(EADC.pAdcTrIb, EADC_OUTPUT_I_MAX, -EADC_OUTPUT_I_MAX, fOffsetOutIb, fGainOutIb );
	EADC_ADCItem_Init(EADC.pAdcTrIc, EADC_OUTPUT_I_MAX, -EADC_OUTPUT_I_MAX, fOffsetOutIc, fGainOutIc );

	EADC_ADCItem_Init(EADC.pAdcBatI, EADC_BAT_I_MAX, -EADC_BAT_I_MAX, fOffsetBatI, fGainBatI );

	EADC_ADCItem_Init(EADC.pAdcInvIa, EADC_INV_I_MAX, -EADC_INV_I_MAX, fOffsetInvIa, fGainInvIa );
	EADC_ADCItem_Init(EADC.pAdcInvIb, EADC_INV_I_MAX, -EADC_INV_I_MAX, fOffsetInvIb, fGainInvIb );
	EADC_ADCItem_Init(EADC.pAdcInvIc, EADC_INV_I_MAX, -EADC_INV_I_MAX, fOffsetInvIc, fGainInvIc );

	//TODO CHECK 일단 Icap에도 offset, gain을 두지 않는다.
	EADC_ADCItem_Init(EADC.pAdcCapIa, EADC_ICAP_I_MAX, -EADC_ICAP_I_MAX, fOffsetLoadIa, fGainLoadIa );
	EADC_ADCItem_Init(EADC.pAdcCapIb, EADC_ICAP_I_MAX, -EADC_ICAP_I_MAX, fOffsetLoadIb, fGainLoadIb );
	EADC_ADCItem_Init(EADC.pAdcCapIc, EADC_ICAP_I_MAX, -EADC_ICAP_I_MAX, fOffsetLoadIc, fGainLoadIc );

#if HILL_CAPI_TO_BYPV == 1
	// HILL 모드에서 Cap Ia 를 바이패스 모드로 사용한다.
		EADC_ADCItem_Init(EADC.pAdcCapIa, EADC_INPUT_V_MAX+30, -EADC_INPUT_V_MAX-30, 0, fGainOutIa );
		EADC_ADCItem_Init(EADC.pAdcCapIb, EADC_INPUT_V_MAX+30, -EADC_INPUT_V_MAX-30, 0, fGainOutIa );
		EADC_ADCItem_Init(EADC.pAdcCapIc, EADC_INPUT_V_MAX+30, -EADC_INPUT_V_MAX-30, 0, fGainOutIa );
#endif

#ifdef STABLEEN
	EADC_ADCItem_Init(EADC.pAdcBypIa, EADC_BYP_I_MAX, -EADC_BYP_I_MAX, fOffsetBypIa, fGainBypIa );
	EADC_ADCItem_Init(EADC.pAdcBypIb, EADC_BYP_I_MAX, -EADC_BYP_I_MAX, fOffsetBypIb, fGainBypIb );
	EADC_ADCItem_Init(EADC.pAdcBypIc, EADC_BYP_I_MAX, -EADC_BYP_I_MAX, fOffsetBypIc, fGainBypIc );
#endif
}

/*
 * 부하전압, 인버터 전압, 바이패스 전압 Adc Item의 Tangent 초기화
 * Tanget뿐만 아니라 Offset Gain도 갱신 한다.
 * 외부 호출 :  parameter module: 인버터의 TR Type이 변경되었을 때
 */
public void EADC_VoltageAdcItemTangentInit(void)
{
	volatile float Max, Min;
	volatile float fOffset, fOffIncDec;
	volatile float fGain, fGainIncDec;
	volatile float fGainVarient;

	fOffset = 0;

	/*
	 * InvE
	 */
	Max = EADC_INV_V_MAX;
	Min = -EADC_INV_V_MAX;

	// Update tangent
	EADC_UpdateTangent(EADC.pAdcInvEa,Max,Min);
	EADC_UpdateTangent(EADC.pAdcInvEb,Max,Min);
	EADC_UpdateTangent(EADC.pAdcInvEc,Max,Min);

	// Update Gain/Offset
	fGainVarient = PRM_PCS[GN_INV_V_L1].fIncDec; // 모든 옵셋의 최소증감치는 동일

	fGain = fGainVarient * PRM_PCS[GN_INV_V_L1].iValue;
	EADC_UpdateGain(EADC.pAdcInvEa, fGain);
	EADC_UpdateOffset(EADC.pAdcInvEa, fOffset);

	fGain = fGainVarient * PRM_PCS[GN_INV_V_L2].iValue;
	EADC_UpdateGain(EADC.pAdcInvEb, fGain);
	EADC_UpdateOffset(EADC.pAdcInvEb, fOffset);

	fGain = fGainVarient * PRM_PCS[GN_INV_V_L3].iValue;
	EADC_UpdateGain(EADC.pAdcInvEc, fGain);
	EADC_UpdateOffset(EADC.pAdcInvEc, fOffset);


	/*
	 * LOAD E
	 */
	Max = EADC_OUTPUT_V_MAX;
	Min = -EADC_OUTPUT_V_MAX;

	// Update tangent
	EADC_UpdateTangent(EADC.pAdcOutEa,Max,Min);
	EADC_UpdateTangent(EADC.pAdcOutEb,Max,Min);
	EADC_UpdateTangent(EADC.pAdcOutEc,Max,Min);

	// Update Gain/Offset
	fGainVarient = PRM_PCS[GN_GRID_V_L1].fIncDec;

	fGain = fGainVarient * PRM_PCS[GN_GRID_V_L1].iValue;
	EADC_UpdateGain(EADC.pAdcOutEa, fGain);
	EADC_UpdateOffset(EADC.pAdcOutEa, fOffset);

	fGain = fGainVarient * PRM_PCS[GN_GRID_V_L2].iValue;
	EADC_UpdateGain(EADC.pAdcOutEb, fGain);
	EADC_UpdateOffset(EADC.pAdcOutEb, fOffset);

	fGain = fGainVarient * PRM_PCS[GN_GRID_V_L3].iValue;
	EADC_UpdateGain(EADC.pAdcOutEc, fGain);
	EADC_UpdateOffset(EADC.pAdcOutEc, fOffset);

	/*
	 * GEN E
	 */
	Max = EADC_GEN_V_MAX;
	Min = -EADC_GEN_V_MAX;
	// Update tangent
	EADC_UpdateTangent(EADC.pAdcGenEa,Max,Min);
	EADC_UpdateTangent(EADC.pAdcGenEb,Max,Min);
	EADC_UpdateTangent(EADC.pAdcGenEc,Max,Min);

	// Update Gain/Offset
	fGainVarient = PRM_PCS[GN_GEN_V_L1].fIncDec;

	fGain = fGainVarient * PRM_PCS[GN_GEN_V_L1].iValue;
	EADC_UpdateGain(EADC.pAdcGenEa, fGain);
	EADC_UpdateOffset(EADC.pAdcGenEa, fOffset);

	fGain = fGainVarient * PRM_PCS[GN_GEN_V_L2].iValue;
	EADC_UpdateGain(EADC.pAdcGenEb, fGain);
	EADC_UpdateOffset(EADC.pAdcGenEb, fOffset);

	fGain = fGainVarient * PRM_PCS[GN_GEN_V_L3].iValue;
	EADC_UpdateGain(EADC.pAdcGenEc, fGain);
	EADC_UpdateOffset(EADC.pAdcGenEc, fOffset);


	/*
	 * Bypass E
	 */
	Max = EADC_INPUT_V_MAX;
	Min = -EADC_INPUT_V_MAX;

	// Update tangent
	EADC_UpdateTangent(EADC.pAdcBypEa,Max,Min);
	EADC_UpdateTangent(EADC.pAdcBypEb,Max,Min);
	EADC_UpdateTangent(EADC.pAdcBypEc,Max,Min);

#if PARA_VERSION >= 0x1003
	// Update Gain/Offset
	fGainVarient = PRM_PCS[GN_BYP_V_L1].fIncDec; // 모든 옵셋의 최소증감치는 동일

	fGain = fGainVarient * PRM_PCS[GN_BYP_V_L1].iValue;
	EADC_UpdateGain(EADC.pAdcBypEa, fGain);
	EADC_UpdateOffset(EADC.pAdcBypEa, fOffset);

	fGain = fGainVarient * PRM_PCS[GN_BYP_V_L2].iValue;
	EADC_UpdateGain(EADC.pAdcBypEb, fGain);
	EADC_UpdateOffset(EADC.pAdcBypEb, fOffset);

	fGain = fGainVarient * PRM_PCS[GN_BYP_V_L3].iValue;
	EADC_UpdateGain(EADC.pAdcBypEc, fGain);
	EADC_UpdateOffset(EADC.pAdcBypEc, fOffset);
#else
	// Update Gain/Offset
	fGainVarient = PRM_PCS[ANL_AI1_GAIN].fIncDec; // 모든 옵셋의 최소증감치는 동일

	fGain = fGainVarient * PRM_PCS[ANL_AI1_GAIN].iValue;
	EADC_UpdateGain(EADC.pAdcBypEa, fGain);
	EADC_UpdateOffset(EADC.pAdcBypEa, fOffset);

	fGain = fGainVarient * PRM_PCS[ANL_AI2_GAIN].iValue;
	EADC_UpdateGain(EADC.pAdcBypEb, fGain);
	EADC_UpdateOffset(EADC.pAdcBypEb, fOffset);

	fGain = fGainVarient * PRM_PCS[ANL_AI3_GAIN].iValue;
	EADC_UpdateGain(EADC.pAdcBypEc, fGain);
	EADC_UpdateOffset(EADC.pAdcBypEc, fOffset);
#endif

	/*
	 * Bypass I
	 */
	Max = EADC_BYP_I_MAX;
	Min = -EADC_BYP_I_MAX;

	// Update tangent
	EADC_UpdateTangent(EADC.pAdcBypIa,Max,Min);
	EADC_UpdateTangent(EADC.pAdcBypIb,Max,Min);
	EADC_UpdateTangent(EADC.pAdcBypIc,Max,Min);

	fGain = fGainVarient * 10000. /* PRM_PCS[GN_BYP_V_L1].iValue*/;
	EADC_UpdateGain(EADC.pAdcBypIa, fGain);
	EADC_UpdateOffset(EADC.pAdcBypIa, fOffset);

	fGain = fGainVarient * 10000. /* PRM_PCS[GN_BYP_V_L2].iValue*/;
	EADC_UpdateGain(EADC.pAdcBypIb, fGain);
	EADC_UpdateOffset(EADC.pAdcBypIb, fOffset);

	fGain = fGainVarient * 10000. /* PRM_PCS[GN_BYP_V_L3].iValue*/;
	EADC_UpdateGain(EADC.pAdcBypIc, fGain);
	EADC_UpdateOffset(EADC.pAdcBypIc, fOffset);
}

/*
 * External ADC Create
 *
 * 외부 호출 :
 * 파라미터 모듈이 먼저 생성 되어야 함. Create 에서 파라미터를 사용함
 */
public void EADC_Create(void)
{
	volatile float Offset, Gain;
	volatile float fGainVarient;

	memset(&adcInvEa, 0, sizeof(adcInvEa));
	memset(&adcInvEb, 0, sizeof(adcInvEb));
	memset(&adcInvEc, 0, sizeof(adcInvEc));
	memset(&adcOutEa, 0, sizeof(adcOutEa));
	memset(&adcOutEb, 0, sizeof(adcOutEb));
	memset(&adcOutEc, 0, sizeof(adcOutEc));
	memset(&adcGenEa, 0, sizeof(adcGenEa)); // Added
	memset(&adcGenEb, 0, sizeof(adcGenEa));
	memset(&adcGenEc, 0, sizeof(adcGenEa));
	memset(&adcDCLinkV, 0, sizeof(adcDCLinkV));
	memset(&adcBatV, 0, sizeof(adcBatV));
	memset(&adcInEa, 0, sizeof(adcInEa));
	memset(&adcInEb, 0, sizeof(adcInEb));
	memset(&adcInEc, 0, sizeof(adcInEc));
	memset(&adcOutIa, 0, sizeof(adcOutIa));
	memset(&adcOutIb, 0, sizeof(adcOutIb));
	memset(&adcOutIc, 0, sizeof(adcOutIc));
	memset(&adcBatI, 0, sizeof(adcBatI));
	memset(&adcRevB_a, 0, sizeof(adcRevB_a));
	memset(&adcRevC_1, 0, sizeof(adcRevC_1));
	memset(&adcRevC_2, 0, sizeof(adcRevC_2));
	memset(&adcRevC_3, 0, sizeof(adcRevC_3));
	memset(&adcInvIa, 0, sizeof(adcInvIa));
	memset(&adcInvIb, 0, sizeof(adcInvIb));
	memset(&adcInvIc, 0, sizeof(adcInvIc));
	memset(&adcBypIa, 0, sizeof(adcBypIa));
	memset(&adcBypIb, 0, sizeof(adcBypIb));
	memset(&adcBypIc, 0, sizeof(adcBypIc));

	EADC.pAdcInvEa = &adcInvEa;
	EADC.pAdcInvEb = &adcInvEb;
	EADC.pAdcInvEc = &adcInvEc;
	EADC.pAdcOutEa = &adcOutEa;
	EADC.pAdcOutEb = &adcOutEb;
	EADC.pAdcOutEc = &adcOutEc;
	EADC.pAdcGenEa = &adcGenEa;
	EADC.pAdcGenEb = &adcGenEb;
	EADC.pAdcGenEc = &adcGenEc;
	EADC.pAdcDCLinkV = &adcDCLinkV;
	EADC.pAdcDCBattV = &adcBatV;
	EADC.pAdcBypEa = &adcInEa;
	EADC.pAdcBypEb = &adcInEb;
	EADC.pAdcBypEc = &adcInEc;
	EADC.pAdcTrIa = &adcOutIa;
	EADC.pAdcTrIb = &adcOutIb;
	EADC.pAdcTrIc = &adcOutIc;
	EADC.pAdcBatI = &adcBatI;
	EADC.pAdcRevB_a = &adcRevB_a;
	EADC.pAdcCapIa = &adcRevC_1;
	EADC.pAdcCapIb = &adcRevC_2;
	EADC.pAdcCapIc = &adcRevC_3;
	EADC.pAdcInvIa = &adcInvIa;
	EADC.pAdcInvIb = &adcInvIb;
	EADC.pAdcInvIc = &adcInvIc;
	EADC.pAdcBypIa = &adcBypIa;
	EADC.pAdcBypIb = &adcBypIb;
	EADC.pAdcBypIc = &adcBypIc;

	EADC_CurrentAdcItemInit();
	EADC_VoltageAdcItemInit();
	EADC_VoltageAdcItemTangentInit(); // TR 타입이 변경되었을 경우 바뀌어야 하는 것들만.
}

public void EADC_UpdateTangent(AdcItem *_this, float Max, float Min)
{
	_this->fTangent = (Max - Min) * ADC_USEBIT_INVERSE;
}

public void EADC_UpdateOffset(AdcItem *_this, float Offset)
{
	_this->fOffset = Offset;
}

private void EADC_UpdateGain(AdcItem *_this, float Gain)
{
	_this->fGain = Gain;
}


/*
 * DMA 반환 결과를 반영한다.
 */
public void EADC_ADC_Result(void)
{
#if MCU_PCB_REV_VER >= MCU_PCB_REV_I_I
	EADC.pAdcInvEc->i16Val = (ADC_A_buf[0] & ADC_MASKBIT);
	EADC.pAdcInvEb->i16Val = (ADC_A_buf[1] & ADC_MASKBIT);
	EADC.pAdcInvEa->i16Val = (ADC_A_buf[2] & ADC_MASKBIT);
	EADC.pAdcOutEc->i16Val = (ADC_A_buf[3] & ADC_MASKBIT);
	EADC.pAdcOutEb->i16Val = (ADC_A_buf[4] & ADC_MASKBIT);
	EADC.pAdcOutEa->i16Val = (ADC_A_buf[5] & ADC_MASKBIT);

	EADC.pAdcCapIa->i16Val = (ADC_B_buf[0] & ADC_MASKBIT); // R3-1
	EADC.pAdcCapIb->i16Val = (ADC_B_buf[1] & ADC_MASKBIT); //
	EADC.pAdcCapIc->i16Val = (ADC_B_buf[2] & ADC_MASKBIT); //

	if( XINTF_ADC_MUX_STATE() == 1 )
	{
		//Active High
		EADC.pAdcBypEb->i16Val = (ADC_B_buf[3] & ADC_MASKBIT);
		EADC.pAdcBypEc->i16Val = (ADC_B_buf[4] & ADC_MASKBIT);
		EADC.pAdcBypEa->i16Val = (ADC_B_buf[5] & ADC_MASKBIT);
#ifndef STABLEEN
		EADC.pAdcTrIb->i16Val = (ADC_C_buf[3] & ADC_MASKBIT);
		EADC.pAdcTrIa->i16Val = (ADC_C_buf[4] & ADC_MASKBIT);
		EADC.pAdcTrIc->i16Val = (ADC_C_buf[5] & ADC_MASKBIT);
#else
		EADC.pAdcBypIb->i16Val = (ADC_C_buf[3] & ADC_MASKBIT);
		EADC.pAdcBypIa->i16Val = (ADC_C_buf[4] & ADC_MASKBIT);
		EADC.pAdcBypIc->i16Val = (ADC_C_buf[5] & ADC_MASKBIT);
#endif
	}
	else
	{
		//Active Low
		EADC.pAdcGenEb->i16Val = (ADC_B_buf[3] & ADC_MASKBIT);
		EADC.pAdcGenEc->i16Val = (ADC_B_buf[4] & ADC_MASKBIT);
		EADC.pAdcGenEa->i16Val = (ADC_B_buf[5] & ADC_MASKBIT);

		EADC.pAdcDCBattV->i16Val = (ADC_C_buf[3] & ADC_MASKBIT);
		EADC.pAdcDCLinkV->i16Val = (ADC_C_buf[4] & ADC_MASKBIT);
		EADC.pAdcBatI->i16Val = (ADC_C_buf[5] & ADC_MASKBIT);
	}

	//AD7656(1)
	EADC.pAdcInvIc->i16Val = (ADC_C_buf[0] & ADC_MASKBIT);
	EADC.pAdcInvIb->i16Val = (ADC_C_buf[1] & ADC_MASKBIT);
	EADC.pAdcInvIa->i16Val = (ADC_C_buf[2] & ADC_MASKBIT);


#elif MCU_PCB_REV_VER == MCU_PCB_REV_B_G

	EADC.pAdcInvEc->i16Val = (ADC_A_buf[0] & ADC_MASKBIT);
	EADC.pAdcInvEb->i16Val = (ADC_A_buf[1] & ADC_MASKBIT);
	EADC.pAdcInvEa->i16Val = (ADC_A_buf[2] & ADC_MASKBIT);
	EADC.pAdcOutEc->i16Val = (ADC_A_buf[3] & ADC_MASKBIT);
	EADC.pAdcOutEb->i16Val = (ADC_A_buf[4] & ADC_MASKBIT);
	EADC.pAdcOutEa->i16Val = (ADC_A_buf[5] & ADC_MASKBIT);

	if( XINTF_ADC_MUX_STATE() == 1 )
	{
		//Active High
		//IN VT, IN VS, IN VR    OUT IR, OUT IS, OUT IT
		EADC.pAdcInEa->i16Val = (ADC_B_buf[0] & ADC_MASKBIT);
		EADC.pAdcInEb->i16Val = (ADC_B_buf[1] & ADC_MASKBIT);
		EADC.pAdcInEc->i16Val = (ADC_B_buf[2] & ADC_MASKBIT);
		EADC.pAdcOutIa->i16Val = (ADC_B_buf[3] & ADC_MASKBIT);
		EADC.pAdcOutIb->i16Val = (ADC_B_buf[4] & ADC_MASKBIT);
		EADC.pAdcOutIc->i16Val = (ADC_B_buf[5] & ADC_MASKBIT);
	}
	else
	{
		//Active Low
		//GEN VT, GEN VS, GEN VR     DCLINK VP, BAT VP, BAT I
		EADC.pAdcGenEa->i16Val = (ADC_B_buf[0] & ADC_MASKBIT);
		EADC.pAdcGenEb->i16Val = (ADC_B_buf[1] & ADC_MASKBIT);
		EADC.pAdcGenEc->i16Val = (ADC_B_buf[2] & ADC_MASKBIT);
		EADC.pAdcDCLinkV->i16Val = (ADC_B_buf[3] & ADC_MASKBIT);
		EADC.pAdcDCBattV->i16Val = (ADC_B_buf[4] & ADC_MASKBIT);
		EADC.pAdcBatI->i16Val = (ADC_B_buf[5] & ADC_MASKBIT);
	}

	//AD7656(1)
	EADC.pAdcInvIc->i16Val = (ADC_C_buf[0] & ADC_MASKBIT);
	EADC.pAdcInvIb->i16Val = (ADC_C_buf[1] & ADC_MASKBIT);
	EADC.pAdcInvIa->i16Val = (ADC_C_buf[2] & ADC_MASKBIT);
	EADC.pAdcCapIa->i16Val = (ADC_C_buf[3] & ADC_MASKBIT);//= * 3.051757e-4 * 10;
	EADC.pAdcCapIb->i16Val = (ADC_C_buf[4] & ADC_MASKBIT);//= * 3.051757e-4 * 10;
	EADC.pAdcCapIc->i16Val = (ADC_C_buf[5] & ADC_MASKBIT);//= * 3.051757e-4 * 10;
#else
	//AD7606(1)
	EADC.pAdcDCLinkV->i16Val = (ADC_A_buf[0] & ADC_MASKBIT);
	EADC.pAdcDCBattV->i16Val = (ADC_A_buf[1] & ADC_MASKBIT);
	EADC.pAdcOutEc->i16Val = (ADC_A_buf[2] & ADC_MASKBIT);
	EADC.pAdcOutEb->i16Val = (ADC_A_buf[3] & ADC_MASKBIT);
	EADC.pAdcOutEa->i16Val = (ADC_A_buf[4] & ADC_MASKBIT);
	EADC.pAdcInvEc->i16Val = (ADC_A_buf[5] & ADC_MASKBIT);
	EADC.pAdcInvEb->i16Val = (ADC_A_buf[6] & ADC_MASKBIT);
	EADC.pAdcInvEa->i16Val = (ADC_A_buf[7] & ADC_MASKBIT);

	//AD7606(2)
	EADC.pAdcOutIa->i16Val = (ADC_B_buf[0] & ADC_MASKBIT);
	EADC.pAdcOutIb->i16Val = (ADC_B_buf[1] & ADC_MASKBIT);
	EADC.pAdcOutIc->i16Val = (ADC_B_buf[2] & ADC_MASKBIT);
	EADC.pAdcBatI->i16Val = (ADC_B_buf[3] & ADC_MASKBIT);
	EADC.pAdcRevB_a->i16Val = (ADC_B_buf[4] & ADC_MASKBIT);
	EADC.pAdcInEc->i16Val = (ADC_B_buf[5] & ADC_MASKBIT);
	EADC.pAdcInEb->i16Val = (ADC_B_buf[6] & ADC_MASKBIT);
	EADC.pAdcInEa->i16Val = (ADC_B_buf[7] & ADC_MASKBIT);

	//AD7656(1)
	EADC.pAdcInvIc->i16Val = (ADC_C_buf[0] & ADC_MASKBIT);
	EADC.pAdcInvIb->i16Val = (ADC_C_buf[1] & ADC_MASKBIT);
	EADC.pAdcInvIa->i16Val = (ADC_C_buf[2] & ADC_MASKBIT);
	EADC.pAdcCapIa->i16Val = (ADC_C_buf[3] & ADC_MASKBIT);//= * 3.051757e-4 * 10;
	EADC.pAdcCapIb->i16Val = (ADC_C_buf[4] & ADC_MASKBIT);//= * 3.051757e-4 * 10;
	EADC.pAdcCapIc->i16Val = (ADC_C_buf[5] & ADC_MASKBIT);//= * 3.051757e-4 * 10;
#endif

#if DBUG_MODE == 1 || DBUG_MODE == 2
	TEST_UpdateEVAConvResult();

	if( (CVC_GetStatus() != CVC_DISABLE || INVERTER.uStatus == SYS_INV_RUN ) || (((GenBlock.bypSw && GenBlock.bBypV) || (GenBlock.genSw && GenBlock.bGenV)) && MCCB_MIRROR.Status.BitVal.bCB4)  )
	{
		EADC.pAdcInvEa->i16Val = DBUG.Vinv_a * ADC_USEBIT_HALF;
		EADC.pAdcInvEb->i16Val = DBUG.Vinv_b * ADC_USEBIT_HALF;
		EADC.pAdcInvEc->i16Val = DBUG.Vinv_c * ADC_USEBIT_HALF;
	}
	else
	{
		EADC.pAdcInvEa->i16Val = 0;
		EADC.pAdcInvEb->i16Val = 0;
		EADC.pAdcInvEc->i16Val = 0;
	}

	//bypass V
	//if( GenBlock.bBypV )
	{
#ifndef STABLEEN
		EADC.pAdcBypEa->i16Val = (DBUG.Vinv_a ) * ADC_USEBIT_HALF;
		EADC.pAdcBypEb->i16Val = (DBUG.Vinv_b ) * ADC_USEBIT_HALF;
		EADC.pAdcBypEc->i16Val = (DBUG.Vinv_c ) * ADC_USEBIT_HALF;
#else
		EADC.pAdcBypEa->i16Val = (DBUG.Vbyp_a) * ADC_USEBIT_HALF;
		EADC.pAdcBypEb->i16Val = (DBUG.Vbyp_b) * ADC_USEBIT_HALF;
		EADC.pAdcBypEc->i16Val = (DBUG.Vbyp_c) * ADC_USEBIT_HALF;
#endif
	}
//	else
//	{
//		EADC.pAdcBypEa->i16Val = 0;
//		EADC.pAdcBypEb->i16Val = 0;
//		EADC.pAdcBypEc->i16Val = 0;
//	}

	EADC.pAdcInvIa->i16Val = -1*DBUG.Iinv_a * ADC_USEBIT_HALF + 0.5;
	EADC.pAdcInvIb->i16Val = -1*DBUG.Iinv_b * ADC_USEBIT_HALF + 0.5;
	EADC.pAdcInvIc->i16Val = -1*DBUG.Iinv_c * ADC_USEBIT_HALF + 0.5;

	EADC.pAdcCapIa->i16Val = (DBUG.Vg_a * 0.9) * ADC_USEBIT_HALF;
	EADC.pAdcCapIb->i16Val = (DBUG.Vg_b * 0.9) * ADC_USEBIT_HALF;
	EADC.pAdcCapIc->i16Val = (DBUG.Vg_c * 0.9) * ADC_USEBIT_HALF;

	EADC.pAdcDCLinkV->i16Val = DBUG.Vdc * ADC_USEBIT_HALF;
	EADC.pAdcDCBattV->i16Val = DBUG.Vdc_batt * ADC_USEBIT_HALF;

	// PCC V
#ifndef STABLEEN
	if( (GenBlock.bypSw && GenBlock.bBypV) || (GenBlock.genSw && GenBlock.bGenV) || !GenBlock.bOn || ( INVERTER.uStatus == SYS_INV_RE_SYNC)|| ( (CTRL.INV.ctrl_mode == PARAM_VSI_PR)&& CVC_GetStatus() != CVC_DISABLE && MCCB_MIRROR.Status.BitVal.bCB4) || (GenBlock.bGenVCB_Status))
#else
	//if( (GenBlock.bypSw && GenBlock.bBypV) || ( INVERTER.uStatus == SYS_INV_RE_SYNC) || ( (CTRL.INV.ctrl_mode == PARAM_VSI_PR) && CVC_GetStatus() != CVC_DISABLE && MCCB_MIRROR.Status.BitVal.bCB4) || MCCB_MIRROR.Status.BitVal.bCB4)
#endif	// #ifndef STABLEEN
	{
		EADC.pAdcOutEa->i16Val = DBUG.Vg_a * ADC_USEBIT_HALF;
		EADC.pAdcOutEb->i16Val = DBUG.Vg_b * ADC_USEBIT_HALF;
		EADC.pAdcOutEc->i16Val = DBUG.Vg_c * ADC_USEBIT_HALF;
	}
//	else
//	{
//		EADC.pAdcOutEa->i16Val = 0;
//		EADC.pAdcOutEb->i16Val = 0;
//		EADC.pAdcOutEc->i16Val = 0;
//	}
	// Tr I
	EADC.pAdcTrIa->i16Val = EADC.pAdcInvIa->i16Val * 4;// * -7.7/*ADC Range차이 -는 방향 바꾸기 위함*/;
	EADC.pAdcTrIb->i16Val = EADC.pAdcInvIb->i16Val * 4;// * -7.7;
	EADC.pAdcTrIc->i16Val = EADC.pAdcInvIc->i16Val * 4;// * -7.7;

	EADC.pAdcBatI->i16Val = DBUG.Ipv * ADC_USEBIT_HALF + 0.5;

	if( GenBlock.bGenV )
	{
		EADC.pAdcGenEa->i16Val = DBUG.Vinv_a * ADC_USEBIT_HALF;
		EADC.pAdcGenEb->i16Val = DBUG.Vinv_b * ADC_USEBIT_HALF;
		EADC.pAdcGenEc->i16Val = DBUG.Vinv_c * ADC_USEBIT_HALF;
	}
	else
	{
		EADC.pAdcGenEa->i16Val = 0;
		EADC.pAdcGenEb->i16Val = 0;
		EADC.pAdcGenEc->i16Val = 0;
	}

#ifdef STABLEEN || MG
/*	EADC.pAdcTrIa->i16Val = DBUG.Itr_a * ADC_USEBIT_HALF;
	EADC.pAdcTrIb->i16Val = DBUG.Itr_b * ADC_USEBIT_HALF;
	EADC.pAdcTrIc->i16Val = DBUG.Itr_c * ADC_USEBIT_HALF;*/

	EADC.pAdcBypIa->i16Val = DBUG.Ibyp_a * ADC_USEBIT_HALF + 0.5;
	EADC.pAdcBypIb->i16Val = DBUG.Ibyp_b * ADC_USEBIT_HALF + 0.5;
	EADC.pAdcBypIc->i16Val = DBUG.Ibyp_c * ADC_USEBIT_HALF + 0.5;
#endif


#endif

}
