/*
 * Filter.h
 *
 *  Created on: 2012. 11. 8.
 *      Author: destinPower
 */

#ifndef FILTER_H_
#define FILTER_H_

#include "LGS_Common.h"

/**********************************************************************/
/* Structure Definition for Filter                                    */
/**********************************************************************/
typedef struct _IIR1STFILTER
{
	float		fLa;
	float		fLa_is;
	float		fLb;
	float		fLb_is;
	float		fPrevIn;
	float		fOut;
} iir1st;

typedef struct _ALLPASSsHIFTFILTER
{
	float		Kxn; // Coeff. of Xn( Input )
	float		Kxn_1; // Coeff. of Xn_1( Previous Input )
	float		Kxn_2; // Coeff. of Xn_2( Previous Previous Input )
	float		Kyn_1; // Coeff. of Yn_1( Previous Output )
	float		Kyn_2; // Coeff. of Yn_2( Previous Previous Output )
	float		Kxn_is; // Coeff. of Xn( Input )
	float		Kxn_1_is; // Coeff. of Xn_1( Previous Input )
	float		Kxn_2_is; // Coeff. of Xn_2( Previous Previous Input )
	float		Kyn_1_is; // Coeff. of Yn_1( Previous Output )
	float		Kyn_2_is; // Coeff. of Yn_2( Previous Previous Output )
	float		Xn_1; // Previous Input
	float		Xn_2; // Previous Previous Input
	float		Yn_1; // Previous Output
	float		Yn_2; // Previous Previous Output
} AllPassShiftFilter;
#if 0
typedef struct _BandPassFilter
{
	float		Kxn; // Coeff. of Xn( Input )
	float		Kxn_1; // Coeff. of Xn_1( Previous Input )
	float		Kxn_2; // Coeff. of Xn_2( Previous Previous Input )
	float		Kyn_1; // Coeff. of Yn_1( Previous Output )
	float		Kyn_2; // Coeff. of Yn_2( Previous Previous Output )
	float		Xn_1; // Previous Input
	float		Xn_2; // Previous Previous Input
	float		Yn_1; // Previous Output
	float		Yn_2; // Previous Previous Output
} BandPassFilter;

typedef struct _HighPassFilter
{
	float		Kxn; // Coeff. of Xn( Input )
	float		Kxn_1; // Coeff. of Xn_1( Previous Input )
	float		Kyn_1; // Coeff. of Yn_1( Previous Output )
	float		Xn_1; // Previous Input
	float		Yn_1; // Previous Output
} HighPassFilter;
#endif

// 인자 설명
//     a ==> iir1st * : 필터 처리 원하는 필터 객체의 포인터
//     b ==> float : 필터 처리를 위한 입력값
//     c ==> float : 필터 처리된 출력값
// Example
//     iir1st *pIIR1;
//     ...
//     IIR1_Filter( &fltInvEqe, InvEqe, InvEqeFlt );
// Macro 사용시 주위점
//     반드시 다음 내부변수를 정의할 것!
//     iir1st *pIIR1
#define IIR1_Filter(a, b, c) \
	pIIR1 = a; \
    pIIR1->fOut = ( pIIR1->fLa * pIIR1->fOut ) + ( pIIR1->fLb * (b + pIIR1->fPrevIn) ); \
    pIIR1->fPrevIn = b; \
	c = pIIR1->fOut;

// Low Pass Filter
// 인자 설명
//     a ==> iir1st * : 필터 처리 원하는 필터 객체의 포인터
//     b ==> float : 필터 처리를 위한 입력값
// Example
//     iir1st *pIIR1;
//     ...
//     IIR1_Filter2( &fltInvEqe, InvEqe );
// Macro 사용시 주위점
//     반드시 다음 내부변수를 정의할 것!
//     iir1st *pIIR1
#define IIR1_Filter2(a, b) \
	pIIR1 = a; \
    pIIR1->fOut = ( pIIR1->fLa * pIIR1->fOut ) + ( pIIR1->fLb * (b + pIIR1->fPrevIn) ); \
    pIIR1->fPrevIn = b;

#define IIR1_Filter_GC(a, b, c) \
	pIIR1 = a; \
    pIIR1->fOut = ( pIIR1->fLa * pIIR1->fOut ) + ( pIIR1->fLb * (b + pIIR1->fPrevIn) ); \
    pIIR1->fPrevIn = b; \
	c = pIIR1->fOut;

#define IIR1_Filter2_GC(a, b) \
	pIIR1 = a; \
    pIIR1->fOut = ( pIIR1->fLa * pIIR1->fOut ) + ( pIIR1->fLb * (b + pIIR1->fPrevIn) ); \
    pIIR1->fPrevIn = b;

#define IIR1_Filter_IS(a, b, c) \
	pIIR1 = a; \
    pIIR1->fOut = ( pIIR1->fLa_is * pIIR1->fOut ) + ( pIIR1->fLb_is * (b + pIIR1->fPrevIn) ); \
    pIIR1->fPrevIn = b; \
	c = pIIR1->fOut;

#define IIR1_Filter2_IS(a, b) \
	pIIR1 = a; \
    pIIR1->fOut = ( pIIR1->fLa_is * pIIR1->fOut ) + ( pIIR1->fLb_is * (b + pIIR1->fPrevIn) ); \
    pIIR1->fPrevIn = b;

#define APS_DEGREE_90	90
#define APS_DEGREE_30	30

//public void IIR1_Creation( iir1st *Filter, float SampleTime, float CutOffFreq );
public void IIR1_Creation_IS( iir1st *Filter, float SampleTime, float CutOffFreq, float SampleTime_is_mode );
//public void IIR1_CalcShadow( iir1st *Filter, float SampleTime, float CutOffFreq );
//public float IIR1_Filtering( iir1st *Filter, float In );
public void IIR1_Initialize( iir1st *Filter );
void IIR1_UpdateCoeff( iir1st *Filter, float SampleTime, float CutOffFreq );
//public void IIR1_ChangeCoefficient( iir1st *Filter );

//public void APS_Creation( AllPassShiftFilter *Filter, float SampleTime, Uns DelayDegree, float Freq );
//public float APS_Filtering( AllPassShiftFilter *Filter, float Xn );
public void APS_Creation_IS( AllPassShiftFilter *Filter, float SampleTime, Uns DelayDegree, float Freq, float SampleTime_is_mode );
public float APS_Filtering_GC( AllPassShiftFilter *Filter, float Xn );
public float APS_Filtering_IS( AllPassShiftFilter *Filter, float Xn );

//public void BPF_Creation( BandPassFilter *Filter, float SampleTime, float Freq );
//public float BPF_Filtering( BandPassFilter *Filter, float Xn );

//public void HPF_Creation( HighPassFilter *Filter, float SampleTime, float Freq);
//public float HPF_Filtering( HighPassFilter *Filter, float Xn );

#endif /* FILTER_H_ */
