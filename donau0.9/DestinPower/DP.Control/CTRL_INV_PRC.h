/*
 * CTRL_INV_PRC.h
 *
 *  Created on: 2014. 4. 3.
 *      Author: Seth Oh
 */

#ifndef CTRL_INV_PRC_H_
#define CTRL_INV_PRC_H_

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

typedef enum { FILTER_MODE_GC_SwF_1 = 0, FILTER_MODE_IS_SwF_2 } FILTER_MODE;

typedef struct _PResonatFilterCoeff
{
	float 		t_sample;
	float		f_cutoff;
	float		f_harmonic;
	float		w_cutoff;
	float		w_harmonic;
	float		a1;
	float		a2;
	float		b0;
//	float		b1; // Always 0
	float		b2;
} PResonatFilterCoeff;

typedef struct _PResonatFilter
{
	PResonatFilterCoeff GC_COEFF;
	PResonatFilterCoeff IS_COEFF;
	float		w0;
	float		w1;
	float		w2;
	float		y; // Output
} PResonatFilter;

// P+Resonat Filter
// 인자 설명
//     a ==> PResonatFilter * : 필터 처리 원하는 필터 객체의 포인터
//     bx ==> float : 필터 처리를 위한 입력값
// Example
//     PResonatFilter *pPRF;
//     ...
//     PRESONANT_Filter2( &har_1st, input );
// Macro 사용시 주위점
//     반드시 다음 내부변수를 정의할 것!
//     PResonatFilter *pPRF
#define PRESONANT_Filter_IS(a, x) \
	pPRF = a; \
	pPRF->w0 = x - (pPRF->IS_COEFF.a1 * pPRF->w1) - (pPRF->IS_COEFF.a2 * pPRF->w2); \
	pPRF->y = (pPRF->IS_COEFF.b0 * pPRF->w0) + (pPRF->IS_COEFF.b2 * pPRF->w2); \
	pPRF->w2 = pPRF->w1; \
	pPRF->w1 = pPRF->w0;

#define PRESONANT_Filter_GC(a, x) \
	pPRF = a; \
	pPRF->w0 = x - (pPRF->GC_COEFF.a1 * pPRF->w1) - (pPRF->GC_COEFF.a2 * pPRF->w2); \
	pPRF->y = (pPRF->GC_COEFF.b0 * pPRF->w0) + (pPRF->GC_COEFF.b2 * pPRF->w2); \
	pPRF->w2 = pPRF->w1; \
	pPRF->w1 = pPRF->w0;

typedef struct _PResonatFilterUnit
{
	PResonatFilter har_1st;
	PResonatFilter har_5th;
	PResonatFilter har_7th;
	PResonatFilter har_11th;
} PResonatFilterUnit;

typedef struct _PResonatGainUnit
{
	float Kp;
	float Ki;
	float K1st;
	float K5th;
	float K7th;
	float K11th;
} PResonatGainUnit;

typedef struct _PResonatBlock
{
	PResonatFilterUnit	FILTER;
	PResonatGainUnit	GAIN;
	float output;
} PResonatBlock;

void CTRL_INV_PRC_Create(void);
void CTRL_INV_PRC_UpdateParameter(void);
void CTRL_INV_PRC_InverterPResonat(PResonatBlock *, float input);
void CTRL_INV_PRC_Initialize(PResonatBlock *);
void CTRL_INV_PRC_Initialize_filter(PResonatBlock *this);
void CTRL_INV_PRC_Run(void);

void PResonatFilter_Creation_IS( PResonatFilter *, float SampleTime, float CutOffFreq, float HarmonicFreq );
void PResonatFilter_Creation_GC( PResonatFilter *, float SampleTime, float CutOffFreq, float HarmonicFreq );
void PResonatFilter_Initialize( PResonatFilter * );

extern void CTRL_INV_PRC_ApplyGC_GI_PI_Gain(void);

#endif /* CTRL_INV_PRC_H_ */
