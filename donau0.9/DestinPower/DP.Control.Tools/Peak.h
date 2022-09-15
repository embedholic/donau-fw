/*
 * Peak.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef PEAK_H_
#define PEAK_H_

#include "LGS_Common.h"

#define PEAK_MODULE_ENB 0

#if PEAK_MODULE_ENB

#define PEAK_LINE_NUM 3
#define PEAK_NUM 12

typedef struct _PeakLine
{
	float fMax;
	float fMin;
	float fOffset;
} PeakLine;

typedef struct _Peak
{
	PeakLine *pL[PEAK_LINE_NUM];
	int iIndex;
	Bool bOffsetCalcEnb;
} Peak;

//////////////////////////////////////////////////////////////////////////////////////
//#define PEAK_OFFSET_BODY \
//	pPEAK->fValue = ( pPEAK->fMax - pPEAK->fMin ) * 0.5; pPEAK->iIndex = 0; \
//	pPEAK->fOffset = pPEAK->fMax - pPEAK->fValue; pPEAK->fMax=0.; pPEAK->fMin=0.;
//
//// ���� ����
////     a ==> Peak * : �ɼ��� �˱� ���ϴ� AC���� Peak ��ü�� ������
////     b ==> float : ��ȯ�� �Ƴ��α� ��
//// Example
////     Peak *pPEAK;
////     ...
////     PEAK_Offset(pBypEaPeak, BypEaRaw, BypEaOffset);
//// Macro ���� ������
////     �ݵ�� ���� ���κ����� ������ ��!
////     Peak *pPEAK
//#define PEAK_Offset(a, b, c) \
//	pPEAK = a; \
//	if (b >= pPEAK->fMax) pPEAK->fMax = b; \
//	if (b <= pPEAK->fMin) pPEAK->fMin = b; \
//	++(pPEAK->iIndex); \
//	if ( pPEAK->iIndex > pPEAK->iTsample ) { PEAK_OFFSET_BODY } \
//	c = pPEAK->fOffset;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
#define PEAK_OFFSET_BODY \
	pPEAK->fValue = ( pPEAK->fMax - pPEAK->fMin ) * 0.5; pPEAK->iIndex = 0; \
	pPEAK->fOffset = pPEAK->fMax - pPEAK->fValue; pPEAK->fMax=0.; pPEAK->fMin=0.;

// ���� ����
//     a ==> PeakLine * : �ɼ��� �˱� ���ϴ� AC���� Peak ��ü�� Line ��ü�� ������
//     b ==> float : ��ȯ�� �Ƴ��α� ��
// Example
//     PeakLine *pPEAKLN;
//     ...
//     PEAK_UpdateM(pPEAKConE->pL[idx], SampleVal);
// Macro ���� ������
//     �ݵ�� ���� ���κ����� ������ ��!
//     Peak *pPEAK
#define PEAK_UpdateM(a, b) \
	pPEAKLN = a; \
	if (b >= pPEAKLN->fMax) pPEAKLN->fMax = b; \
	if (b <= pPEAKLN->fMin) pPEAKLN->fMin = b;

////////////////////////////////////////////////////////////////////////////////////

void PEAK_UpdateAll(Peak *this, float *Raw);
void PEAK_OffsetCalc(void);
float PEAK_GetOffset(Peak *this, Uns LineNum);

extern Peak *pPEAKConE;
extern Peak *pPEAKBypE;
extern Peak *pPEAKInvE;
extern Peak *pPEAKLoadE;

#endif /* PEAK_MODULE_ENB */


#endif /* PEAK_H_ */
