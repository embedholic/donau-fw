/*
 * Peak.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */


#include "PEAK.h"
#include "FAULT.h"
#include "CC.h"

#if PEAK_MODULE_ENB

// ���� 1�ֱ⸦ 100usec ������ ���ø� �ϴ� ������ ������
// 100usec�� ������ �ִ� �����ֱ�(CCP)��. ���������� ���� CCP(SY_CC_PERIOD_USEC)�� 128��
// 50Hz������ ����Ͽ� ���� ���� 1 �ֱ⸦ 45Hz�� ����
// ���� �� �ֱ��� �ִ� ���� ���� 1/45/100usec = 222 ��
#define PEAK_MAX_SAMPLE 222

// ���ø� ���� 222�� �̹Ƿ� 222 * 100usec = 22.2msec ���� Offset�� ��� �ȴ�.
// ����, �� ���� ���͸��� 20msec���� �ص� ������ ����.

Peak *pPEAKConE;
Peak *pPEAKBypE;
Peak *pPEAKInvE;
Peak *pPEAKLoadE;

static PeakLine PEAKLINE_CONE[PEAK_LINE_NUM];
static PeakLine PEAKLINE_BYPE[PEAK_LINE_NUM];
static PeakLine PEAKLINE_INVE[PEAK_LINE_NUM];
static PeakLine PEAKLINE_LOADE[PEAK_LINE_NUM];

static Peak PEAK_CONE;
static Peak PEAK_BYPE;
static Peak PEAK_INVE;
static Peak PEAK_LOADE;

static int iIndex = 0;
static Bool bOffsetCalcEnb = FALSE;

/*******************************************************************/
/* PeakLine_() - PeakLine_ Class ����                              */
/* Parameters : void                                               */
/* Returns : PeakLine_ *                                           */
/* Called : Peak_()                                                */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2003.04.04 By S.J.OH                                     */
/* Modification Remarks : ���� ����                                */
/* Date : 2003.12.15 By sjoh                                       */
/* Modification Remarks : ���α׷� ũ�⸦ ���̱� ���Ͽ� malloc��   */
/*       �̿����� �ʵ��� ��                                        */
/*******************************************************************/
PeakLine *PeakLine_(PeakLine *this)
{
//	PeakLine	*this;

//	if ( ( this = (PeakLine *)malloc(sizeof(PeakLine)) ) == NULL ) FLT_Raise(FLT_MALLOC);
	this->fMax = 0;
	this->fMin = 0;
	this->fOffset = 0;
	return this;
}

/*******************************************************************/
/* Peak_() - Peak �� Class ����                                    */
/* Parameters : void                                               */
/* Returns : Peak *                                                */
/* Called :                                                        */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2003.04.04 By S.J.OH                                     */
/* Modification Remarks : ���� ����                                */
/* Date : 2003.12.15 By sjoh                                       */
/* Modification Remarks : ���α׷� ũ�⸦ ���̱� ���Ͽ� malloc��   */
/*       �̿����� �ʵ��� ��                                        */
/*******************************************************************/
Peak *Peak_(Peak *this)
{
//	Peak	*this;

//	if ( ( this = (Peak *)malloc(sizeof(Peak)) ) == NULL ) FLT_Raise(FLT_MALLOC);
	this->pL[0] = PeakLine_(&pPL[0]);
	this->pL[1] = PeakLine_(&pPL[1]);
	this->pL[2] = PeakLine_(&pPL[2]);
	this->iIndex = 0;
	this->bOffsetCalcEnb = FALSE;

	return this;
}

/*******************************************************************/
/* PEAK_Create() - PEAK Class ��� ����                            */
/* Parameters : void                                               */
/* Returns : void                                                  */
/* Called : main.c                                                 */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2003.04.04 By S.J.OH                                     */
/* Modification Remarks : ���� ����                                */
/* Date : 2003.12.15 By sjoh                                       */
/* Modification Remarks : ���α׷� ũ�⸦ ���̱� ���Ͽ� malloc��   */
/*       �̿����� �ʵ��� ��                                        */
/*******************************************************************/
void PEAK_Create(void)
{
	pPEAKConE = Peak_(&PEAK_CONE);
	pPEAKBypE = Peak_(&PEAK_BYPE);
	pPEAKInvE = Peak_(&PEAK_INVE);
	pPEAKLoadE = Peak_(&PEAK_LOADE);
}

/*******************************************************************/
/* PEAK_UpdateAll - ���� ��ũ, �ɼ� ����� ���� ���ð� ���        */
/* Parameters : float *ValBuf ���ð��� �迭                        */
/* Returns : void                                                  */
/* Called : cc.c                                                   */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2003.04.03 By S.J.OH                                     */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/
void PEAK_UpdateAll(Peak *this, float *Raw)
{
	if (Raw[0] > this->pL[0]->fMax) this->pL[0]->fMax = Raw[0];
	if (Raw[0] < this->pL[0]->fMin) this->pL[0]->fMin = Raw[0];
	if (Raw[1] > this->pL[1]->fMax) this->pL[1]->fMax = Raw[1];
	if (Raw[1] < this->pL[1]->fMin) this->pL[1]->fMin = Raw[1];
	if (Raw[2] > this->pL[2]->fMax) this->pL[2]->fMax = Raw[2];
	if (Raw[2] < this->pL[2]->fMin) this->pL[2]->fMin = Raw[2];

	if( ++this->iIndex > PEAK_MAX_SAMPLE )
	{
		this->iIndex = 0;
		this->bOffsetCalcEnb = TRUE;
	}
}

///*******************************************************************/
///* PEAK_UpdateAll - ���� ��ũ, �ɼ� ����� ���� ���ð� ���        */
///* Parameters : float *ValBuf ���ð��� �迭                        */
///* Returns : void                                                  */
///* Called : cc.c                                                   */
///* Related :                                                       */
///*-----------------------------------------------------------------*/
///* Example :                                                       */
///*-----------------------------------------------------------------*/
///* Date : 2003.04.03 By S.J.OH                                     */
///* Modification Remarks : Initial Version                          */
///*******************************************************************/
//void PEAK_UpdateAll(void)
//{
//	static float SampleVal;
//	static PeakLine *pPEAKLN;
//	static int idx = 1;
//
//	for ( idx = 0; idx < PEAK_LINE_NUM; idx++ )
//	{
//		SampleVal = rawConE[idx];
//		pPEAKLN = pPEAKConE->pL[idx];
//		PEAK_UpdateM(pPEAKLN, SampleVal);
//		SampleVal = rawBypE[idx];
//		pPEAKLN = pPEAKBypE->pL[idx];
//		PEAK_UpdateM(pPEAKLN, SampleVal);
//		SampleVal = rawInvE[idx];
//		pPEAKLN = pPEAKInvE->pL[idx];
//		PEAK_UpdateM(pPEAKLN, SampleVal);
//		SampleVal = rawLoadE[idx];
//		pPEAKLN = pPEAKLoadE->pL[idx];
//		PEAK_UpdateM(pPEAKLN, SampleVal);
//	}
//
//	if( ++iIndex > PEAK_MAX_SAMPLE )
//	{
//		iIndex = 0;
//		bOffsetCalcEnb = TRUE;
//	}
//}

static void OffsetCalculate(Peak *this)
{
	int idx;
	float Mean;

	if ( !this->bOffsetCalcEnb ) return;

	for ( idx = 0; idx < PEAK_LINE_NUM; idx++ )
	{
		Mean = ( this->pL[idx]->fMax - this->pL[idx]->fMin ) * 0.5;
		this->pL[idx]->fOffset = this->pL[idx]->fMax - Mean;
		this->pL[idx]->fMax=0.;
		this->pL[idx]->fMin=0.;
	}
	this->bOffsetCalcEnb = FALSE;
}

/*******************************************************************/
/* PEAK_OffsetCalc - Peak Offset ���                              */
/* Parameters : void                                               */
/* Returns : void                                                  */
/* Called : main.c                                                 */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2003.04.03 By S.J.OH                                     */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/
void PEAK_OffsetCalc(void)
{
	OffsetCalculate( pPEAKConE );
	OffsetCalculate( pPEAKBypE );
	OffsetCalculate( pPEAKInvE );
	OffsetCalculate( pPEAKLoadE );
}

/*******************************************************************/
/* PEAK_GetOffset - Offset ����                                    */
/* Parameters : Peak *this                                         */
/* Returns : void                                                  */
/* Called : MVP.c                                                  */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.07.25 By S.J.OH                                     */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/
float PEAK_GetOffset(Peak *this, Uns LineNum)
{
	return this->pL[LineNum]->fOffset;
}

#endif /* PEAK_MODULE_ENB */


