/*
 * EADC.h
 *
 *  Created on: 2012. 11. 5.
 *      Author: destinPower
 */

#ifndef EADC_H_
#define EADC_H_

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include "LGS_Common.h"

#if 0//by JCNET
#if GRID_RATED_VOLTAGE == 480

// 인버터 전압-7606(3ch) 검출 범위 AVI
#define EADC_INV_V_MAX 575.468181774 // +- 10V 입력

// 출력 전압-7606(3ch) 검출 범위 AVI
#define EADC_OUTPUT_V_MAX 575.468181774 // +- 10V 입력

// 제네레이터 전압
#define EADC_GEN_V_MAX 575.468181774 // +- 10V 입력

// Bypass 전압-7606 (3ch) 검출 범위 AVI
#define EADC_INPUT_V_MAX 575.468181774 // +- 10V 입력

#else

// 인버터 전압-7606(3ch) 검출 범위 AVI
#define EADC_INV_V_MAX 473.63636

// 출력 전압-7606(3ch) 검출 범위 AVI
#define EADC_OUTPUT_V_MAX 473.6363636 // +- 10V 입력

// 제네레이터 전압
#define EADC_GEN_V_MAX 473.6363636 // MG-LV

// Bypass 전압-7606 (3ch) 검출 범위 AVI
#if OP_MODE == STBLN_300K
#define EADC_INPUT_V_MAX 473.6363636 // 16.07.25 Yang +- 10V 입력
#endif
#if OP_MODE == STBLN_500K || OP_MODE >= STBLN_1250K
#define EADC_INPUT_V_MAX 1500.277167
#endif

#endif
#else //by JCNET  TODO

#define EADC_INV_V_MAX 575.468181774
// 출력 전압-7606(3ch) 검출 범위 AVI
#define EADC_OUTPUT_V_MAX 575.468181774 // +- 10V 입력
// 제네레이터 전압
#define EADC_GEN_V_MAX 575.468181774 // +- 10V 입력

// Bypass 전압-7606 (3ch) 검출 범위 AVI
#define EADC_INPUT_V_MAX 575.468181774 // +- 10V 입력

#define EADC_OUTPUT_I_MAX 2500.0
#define EADC_OUTPUT_I_MIN -2500.0
#define EADC_BAT_I_MAX  1998.644986 // -1932.76009 //
#define EADC_ICAP_I_MAX 508.8652482 // 2284.1 // ( 1142.05 * 2 )
#define EADC_DCLINK_V_MAX 1030
#define EADC_BAT_V_MAX 1030 // 의미없는 값..

#define EADC_BYP_I_MAX -4000//-800.17
#define EADC_BYP_I_MIN 4000//800.14
#define EADC_INV_I_MAX 2500.
#define EADC_INV_I_MIN -2500.
#endif

#if 0 //by JCNET
/*
 *  DC Link 전압-7606 (1ch) 검출 범위 DVI
 */
//-#define EADC_DCLINK_V_MAX 4120 // (1030 * 4) +- 2.5V
#if USE_1000KW_FGDC_SWITCH == 1
#define EADC_DCLINK_V_MAX 1370
#else
#define EADC_DCLINK_V_MAX 1030
#endif



/*
 *  Battery 전압-7606 (1ch) 검출 범위 DVI
 */
#if USE_1000KW_FGDC_SWITCH == 1
#define EADC_BAT_V_MAX 1370
#else
#define EADC_BAT_V_MAX 1030
#endif

#endif //by jcnET
/*
 * 전류센서 방향이 GRID 로 향하면, 전류 검출을 반전 시켜야 함.
 * 전류제어기 모델은 Grid에서 들어오는 방향으로 만들어짐
 */
/* 인버터 전류-7656-1 (3ch) 검출 범위 ASI */
#if OP_MODE == STBLN_300K/* STBLEN300 ABB ES1000C */
#define EADC_INV_I_MAX 4545.454545 //
#define EADC_INV_I_MIN -4545.454545 //
#endif
#if OP_MODE == STBLN_500K
#define EADC_INV_I_MAX 4545.3851
#define EADC_INV_I_MIN -4545.5239
#endif
#if OP_MODE >= STBLN_1250K
#define EADC_INV_I_MAX 5617.891803
#define EADC_INV_I_MIN -5618.063253
#endif


/* TR 측 전류(TR없는 모델에서는 인버터 전류 이용)-7606 (3ch) 검출 범위 ASI */
#if OP_MODE == STBLN_300K
#define EADC_OUTPUT_I_MAX -800.17
#define EADC_OUTPUT_I_MIN 800.14
#endif

#if OP_MODE == STBLN_500K || OP_MODE >= STBLN_1250K
#define EADC_OUTPUT_I_MAX -1500.277167
#define EADC_OUTPUT_I_MIN 1500.322953
#endif

/* Battery 전류-7606 (1ch) 검출 범위 ASI */
#if OP_MODE == STBLN_300K/* STBLEN300 ABB ES1000C */
#define EADC_BAT_I_MAX -1932.76009 //
#endif
#if OP_MODE == STBLN_500K
#define EADC_BAT_I_MAX -1932.770118
#endif
#if OP_MODE == STBLN_1250K
#define EADC_BAT_I_MAX -2451.24339
#endif

/* Icap r,s,t (3ch) 검출 범위 ASI => Icap r,s,t */
#if OP_MODE == STBLN_300K
#define EADC_ICAP_I_MAX 2284.1 // ( 1142.05 * 2 )
//#define EADC_ICAP_I_MAX 1142.05
#endif
#if OP_MODE == STBLN_500K || OP_MODE >= STBLN_1250K
#define EADC_ICAP_I_MAX 2284.11
#endif

/* BYP 측 전류-7606 (3ch) 검출 범위 ASI */
#if OP_MODE == STBLN_300K
#define EADC_BYP_I_MAX -4000//-800.17
#define EADC_BYP_I_MIN 4000//800.14
#endif


/* PEBB TEMP 개수 */
#define EADC_PEBBTEMP_MAXNUM (3)


typedef struct _AdcItem
{
	Int16 i16Val; /* FIXME Check Value Range. 16bit or 32bit. 현, 16비트인데, 32비트로 바꿀 경우 int 16비트 값을 받으면 부호가 +로 고정. */
	float fTangent;
	float fOffset;
	float fGain;
}AdcItem;


typedef struct _Adc
{
	/*
	 * Ea Eb Ec : Voltage a,b,c 부하, 인버터, 바이패스 접압들에 대해서는 Object Create가 아닌 Tangent Init을 수행
	 * V : V Object Create
	 * Ia, Ib, Ic : I Object Create
	 */
	//ADC-A
 //ABI-1 by JCNet comment
	AdcItem *pAdcInvEa;
	AdcItem *pAdcInvEb;
	AdcItem *pAdcInvEc;
//ABI-2 by JCNet comment
	AdcItem *pAdcOutEa; // (PCC V)
	AdcItem *pAdcOutEb;
	AdcItem *pAdcOutEc;

	//ADC-B (MUX)
//ABI-3
	AdcItem *pAdcBypEa;  // LOAD(Bypass V)
	AdcItem *pAdcBypEb;
	AdcItem *pAdcBypEc;

	AdcItem *pAdcTrIa; // (PCC I)
	AdcItem *pAdcTrIb;
	AdcItem *pAdcTrIc;
	AdcItem *pAdcBypIa; // (BYP I)
	AdcItem *pAdcBypIb;
	AdcItem *pAdcBypIc;

	AdcItem *pAdcGenEa; // NOT USED
	AdcItem *pAdcGenEb;
	AdcItem *pAdcGenEc;
	AdcItem *pAdcDCLinkV;
	AdcItem *pAdcDCBattV;
	AdcItem *pAdcBatI;
	AdcItem *pAdcRevB_a;

	//ADC-C
	AdcItem *pAdcCapIa;
	AdcItem *pAdcCapIb;
	AdcItem *pAdcCapIc;
	AdcItem *pAdcInvIa;
	AdcItem *pAdcInvIb;
	AdcItem *pAdcInvIc;
}EAdc;

public void EADC_Create(void);
public void EADC_UpdateTangent(AdcItem *_this, float Max, float Min);
public void EADC_UpdateOffset(AdcItem *_this, float Offset);
public void EADC_CurrentAdcItemInit(void);
public void EADC_VoltageAdcItemInit(void);
public void EADC_VoltageAdcItemTangentInit(void);
public void EADC_ADC_Result(void);
extern EAdc EADC;
/*
 * pAdcItem: 가져올 값의 Adc포인터
 * pRetFloatVal: gain, offset 적용 된 값을 저장할 변수(float)
 * TODO : Test
 * AIB_Get
 */
inline public void EADC_GetAnalogValue(AdcItem *pAdcItem,out float *pRetFloatVal )
{
	Assert_isTrue(pAdcItem == NULL, NULL);
	Assert_isTrue(pRetFloatVal == NULL, NULL);

    *pRetFloatVal = (pAdcItem->i16Val * pAdcItem->fTangent);
    *pRetFloatVal *= (pAdcItem->fGain);
    *pRetFloatVal -= (pAdcItem->fOffset);
}

#endif /* EADC_H_ */
