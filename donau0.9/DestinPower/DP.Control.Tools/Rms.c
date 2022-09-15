/*
 * Rms.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#include <math.h>
#include <string.h>

#include "LGS_Common.h"
#include "RMS.h"
#include "FAULT.h"
#include "system.h"
#include "parameter.h"
#include "prm_pcs.h"

#define RMS_ID_NUM 7 /* RMS ����� �ʿ��� ���� �� */
#define RMS_LINE_NUM 3 /* Phase Num */
#define RMS_50HZ_BUF_SIZE 25  /* 500msec/20msec, 500msec ����� ���Ͽ� */
#define RMS_60HZ_BUF_SIZE 30  /* 500msec/16.67msec, 500msec ����� ���Ͽ� */

#define RMS_NULL ((Rms *)0)

#pragma DATA_SECTION(RMS_PING_BUF, "L47")
#pragma DATA_SECTION(RMS_PONG_BUF, "L47")

// 28x �迭���� Uns ������ 16��Ʈ�� �޸� ������ ����� �ȴ�.
float RMS_PING_BUF[RMS_ID_NUM * RMS_LINE_NUM * RMS_PING_BUF_SIZE];
float RMS_PONG_BUF[RMS_ID_NUM * RMS_LINE_NUM * RMS_PING_BUF_SIZE];


/*-
#pragma DATA_SECTION(pRMSConE, ".iramdata")
#pragma DATA_SECTION(pRMSConI, ".iramdata")
#pragma DATA_SECTION(pRMSGridV, ".iramdata")
#pragma DATA_SECTION(pRMSGridEp, ".iramdata")
*/
Rms *pRMSConE;
Rms *pRMSConI;
Rms *pRMSGridV;
Rms *pRMSGridEp;
Rms *pRMSGridI;//13.3.27
Rms *pRMSBypassV;
Rms *pRMSBypassI;	//	16.08.10


static Uns BufSizeMax;
static RmsLine RMSLINE_ConE[RMS_LINE_NUM];
static RmsLine RMSLINE_ConI[RMS_LINE_NUM];
static RmsLine RMSLINE_GridV[RMS_LINE_NUM];
static RmsLine RMSLINE_GridEp[RMS_LINE_NUM];
static RmsLine RMSLINE_GridI[RMS_LINE_NUM]; // 13.3.27
static RmsLine RMSLINE_BYP[RMS_LINE_NUM]; // 13.3.27
static RmsLine RMSLINE_BypI[RMS_LINE_NUM];	//	16.08.10

static Rms RMS_ConE;
static Rms RMS_ConI;
static Rms RMS_GridV;
static Rms RMS_GridEp;	// ���� ��� �Ǵ� ���� ����.
static Rms RMS_GridI; 	// 13.3.27
static Rms RMS_BypV;    // 14.11.27
static Rms RMS_BypI;	//	16.08.10


/*
 * call: RMS_Calculate
 */
static void OneRMSCalc( RmsLine *this, float *Buf, Uns BufSize )
{
	int i;
	float fSum = 0;
	float fSample;
	float fMax = 0;
	float fMin = 0;
	//float fPeak;

	for ( i = 0; i < BufSize; i++ )
	{
		fSample = Buf[i];
		fSum += fSample*fSample;

		if(fSample > fMax)
		{
			fMax = fSample;
		}
		if(fSample < fMin)
			fMin = fSample;
	}

	//+Safe
	if(BufSize == 0)
	{
		this->fInstantValue = 0;
		return;
	}
#if DEBUG_MODE == 1
	if(BufSize == 0)
		error();
#endif
	this->fInstantValue = sqrt(fSum / BufSize);
	this->fSum500 += this->fInstantValue;

	if ( ++this->uSum500Count >= BufSizeMax )
	{
#if USE_FASTRTS_LIB_DIV
//-		InverseBufSizeMax = recipsp(BufSizeMax);
#else
//-13.12.23		InverseBufSizeMax = 1./ BufSizeMax;
#endif
//-13.12.23		this->fValue = this->fSum500 * InverseBufSizeMax; // 500msec ���

		this->fValue = this->fSum500 / BufSizeMax; // 500msec ���
		this->uSum500Count = 0;
		this->fSum500 = 0;
	}
}


/*******************************************************************/
/* RMS_Calculate - Rms ���                                        */
/* Parameters : Rms *this, Rms *volt                               */
/* Returns : void                                                  */
/* Called : RMS_AllCalculate()                                     */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.01.24 By S.J.OH                                     */
/* Modification Remarks : Initial Version                          */
/* Date : 2002.07.03 By S.J.OH                                     */
/* Modification Remarks : Power����� ���Ͽ� ����                  		*/
/*******************************************************************/

/*
 * call: RMS_AllCalculate
 */
static void RMS_Calculate(Rms *this)
{
	int Num;

	if ( this->bCalcEnable == FALSE ) return;

	if ( this->uStatus == RMS_PING ) // ���� �ι��ۿ� �����͸� ������ �ִ� ������
	{	// ���� �� ����� �������� ������ �̿��Ͽ� rms �Ի�
		for ( Num = 0; Num < RMS_LINE_NUM; Num++ )
		{
			OneRMSCalc( this->pL[Num], this->pL[Num]->pPongBuf, this->uPongBufSize );
		}
	}
	else // ���� �����ۿ� �����͸� ������ �ִ� ������
	{	// ���� �� ����� �ι����� ������ �̿��Ͽ� rms �Ի�
		for ( Num = 0; Num < RMS_LINE_NUM; Num++ )
		{
			OneRMSCalc( this->pL[Num], this->pL[Num]->pPingBuf, this->uPingBufSize );
		}
	}

	this->bCalcEnable = FALSE;
}


/*******************************************************************/
/* Change - Ping Buffer <==> Pong Buffer                           */
/* Parameters : Rms *this                                          */
/* Returns : void                                                  */
/* Called : another objects                                        */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.01.24 By S.J.OH                                     */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/
//-TODO DATA REGION: #pragma CODE_SECTION(RMS_Change, ".iram")

/*
 * call: c_int11()
 * when: ThetaDetectRun() ���� line�� fRadian�� PI ���� ũ��.
 */
void RMS_Change(Rms *this)
{
	if (this->uStatus == RMS_PING)
	{
		this->uStatus = RMS_PONG;
		this->uPongBufSize = 0;
	}
	else
	{
		this->uStatus = RMS_PING;
		this->uPingBufSize = 0;
	}
	this->bCalcEnable = TRUE;
}

/*******************************************************************/
/* AddSample - rms Sample �߰�                                     */
/* Parameters : Rms *this                                          */
/*        float SampleVal : Sample�� ��                            */
/* Returns : void                                                  */
/* Called : another objects                                        */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.01.24 By S.J.OH                                     */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/

/*
 * call: c_int11()
 * when: �ֱ⸶�� ȣ�� ��.
 */
void RMS_AddSampleAll(Rms *this)
{

	if (this->uStatus == RMS_PING)
	{
		this->pL[0]->pPingBuf[this->uPingBufSize] = this->pL[0]->fSample;
		this->pL[1]->pPingBuf[this->uPingBufSize] = this->pL[1]->fSample;
		this->pL[2]->pPingBuf[this->uPingBufSize] = this->pL[2]->fSample;
		// ���ļ��� ������������ ���� ��� RMS_BUF_SIZE�� �ʰ��� �� �ִ�.
		// �̷� ��Ȳ�� �߻��Ǹ� ���α׷��� �������� �� ����.
		// ���ļ��� ������������ ���� ��� RMS����� ��Ȯ���� �ʰ� �ȴ�.
		if ( ++this->uPingBufSize >= RMS_PING_BUF_SIZE )
		{
			this->uPingBufSize = RMS_PING_BUF_SIZE - 1;
		}

	}
	else
	{
		this->pL[0]->pPongBuf[this->uPongBufSize] = this->pL[0]->fSample;
		this->pL[1]->pPongBuf[this->uPongBufSize] = this->pL[1]->fSample;
		this->pL[2]->pPongBuf[this->uPongBufSize] = this->pL[2]->fSample;
		if ( ++this->uPongBufSize >= RMS_PING_BUF_SIZE ) this->uPongBufSize = RMS_PING_BUF_SIZE - 1;
	}
}

/*******************************************************************/
/* GetValue - rms 500msec ��հ� ����                              */
/* Parameters : Rms *this                                          */
/* Returns : float                                                 */
/* Called : another objects                                        */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.01.24 By S.J.OH                                     */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/

/*
 * call: MVP_Process20msec
 * when: RMS�� ���� MEASURE ������ �����Ѵ�.
 */
float RMS_GetValue(Rms *this, Uns LineNum)
{
	return this->pL[LineNum]->fValue;
}

/*******************************************************************/
/* GetInstantValue - rms(1�ֱ�) �� ����                            */
/* Parameters : Rms *this                                          */
/* Returns : float                                                 */
/* Called : another objects                                        */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.03.15 By S.J.OH                                     */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/
/*
 * call: SYS_VoltFreqMonitor <- MVP_Process20msec
 *
 */
float RMS_GetInstantValue(Rms *this, Uns LineNum)
{
	return this->pL[LineNum]->fInstantValue;
}

/*******************************************************************/
/* RmsLine_() - RmsLine_ Class ����                                */
/* Parameters : Uns BufIndex : ��/�� ������ �ܺθ޸� ����� ��ġ */
/* Returns : RmsLine_ *                                            */
/* Called : Rms_()                                                 */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2003.04.03 By S.J.OH                                     */
/* Modification Remarks : ���� ����                                */
/* Date : 2003.12.15 By sjoh                                       */
/* Modification Remarks : ���α׷� ũ�⸦ ���̱� ���Ͽ� malloc��   */
/*       �̿����� �ʵ��� ��                                        */
/*******************************************************************/
/*
 * call: Rms_()
 */
RmsLine *RmsLine_(RmsLine *this, Uns BufIndex)
{
	// BufIndex =
	// 0, 1, 2
	// 3, 4, 5
	// 6, 7, 8
	// 9, 10, 11

	this->pPingBuf = (float *)RMS_PING_BUF + (BufIndex * RMS_PING_BUF_SIZE);

	this->pPongBuf = (float *)RMS_PONG_BUF + (BufIndex * RMS_PING_BUF_SIZE);

	// 500msec ��տ� ���Ͽ�...
	this->uSum500Count = 0;
	this->fSum500 = 0;

	this->fValue = 0;
	this->fInstantValue = 0;

	return this;
}

/*******************************************************************/
/* Rms_() - Rms �� Class ����                                      */
/* Parameters : Uns BufIndex : ��/�� ������ �ܺθ޸� ����� ��ġ */
/* Returns : Rms *                                                 */
/* Called : RMS_Create()                                           */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.01.24 By S.J.OH                                     */
/* Modification Remarks : ���� ����                                */
/* Date : 2002.07.26 By S.J.OH                                     */
/* Modification Remarks : ���θ޸𸮺��� ����� ���۸�             */
/*                        �ܺ� �޸𸮿� �Ҵ��Ŵ                   */
/* Date : 2003.04.03 By S.J.OH                                     */
/* Modification Remarks : Rms ����ü ���濡 ���� ����              */
/* Date : 2003.12.15 By sjoh                                       */
/* Modification Remarks : ���α׷� ũ�⸦ ���̱� ���Ͽ� malloc��   */
/*       �̿����� �ʵ��� ��                                        */
/*******************************************************************/
/*
 * call: RMS_Create()
 */
Rms *Rms_(Rms *this, RmsLine *pL, Uns BufIndex)
{
//	Rms	*this;

//	if ( ( this = (Rms *)malloc(sizeof(Rms)) ) == NULL ) FLT_Raise(FLT_MALLOC);
	this->uStatus = RMS_PING;
	this->bCalcEnable = FALSE;
	this->uPingBufSize = 0;
	this->uPongBufSize = 0;

	// buf index : 0 3 6 9
	// 0: 0, 1, 2
	// 3: 3, 4, 5
	// 6: 6, 7, 8
	// 9: 9, 10, 11
	this->pL[0] = RmsLine_(&pL[0], BufIndex);
	this->pL[1] = RmsLine_(&pL[1], BufIndex+1);
	this->pL[2] = RmsLine_(&pL[2], BufIndex+2);

	return this;
}

/*******************************************************************/
/* RMS_Create() - RMS Class ��� ����                              */
/* Parameters : void                                               */
/* Returns : void                                                  */
/* Called : main.c                                                 */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.01.24 By S.J.OH                                     */
/* Modification Remarks : ���� ����                                */
/* Date : 2003.04.03 By S.J.OH                                     */
/* Modification Remarks : Rms ����ü ���濡 ���� ����              */
/* Date : 2003.12.15 By sjoh                                       */
/* Modification Remarks : ���α׷� ũ�⸦ ���̱� ���Ͽ� malloc��   */
/*       �̿����� �ʵ��� ��                                        */
/*******************************************************************/
void RMS_Create(void)
{
	memset(&RMS_PING_BUF, 0, sizeof(RMS_PING_BUF));
	memset(&RMS_PONG_BUF, 0, sizeof(RMS_PONG_BUF));

	memset(&RMSLINE_ConE, 0, sizeof(RMSLINE_ConE));
	memset(&RMSLINE_ConI, 0, sizeof(RMSLINE_ConI));
	memset(&RMSLINE_GridV, 0, sizeof(RMSLINE_GridV));
	memset(&RMSLINE_GridEp, 0, sizeof(RMSLINE_GridEp));
	memset(&RMSLINE_GridI, 0, sizeof(RMSLINE_GridI)); // 13.3.27
	memset(&RMSLINE_BYP, 0, sizeof(RMSLINE_BYP)); // 13.3.27

	memset(&RMS_ConE, 0, sizeof(RMS_ConE));
	memset(&RMS_ConI, 0, sizeof(RMS_ConI));
	memset(&RMS_GridV, 0, sizeof(RMS_GridV));
	memset(&RMS_GridEp, 0, sizeof(RMS_GridEp));
	memset(&RMS_GridI, 0, sizeof(RMS_GridI)); // 13.3.27
	memset(&RMS_BypV, 0, sizeof(RMS_BypV)); // 13.3.27
	memset(&RMS_BypI, 0, sizeof(RMS_BypI));	//	16.08.10


	if ( PRM_PCS[GRID_RATED_FREQ].iValue <= 50 ) // 50Hz ==> 25, 60Hz ==> 30
		BufSizeMax = RMS_50HZ_BUF_SIZE;
	else
		BufSizeMax = RMS_60HZ_BUF_SIZE;

	pRMSConE = Rms_(&RMS_ConE, RMSLINE_ConE, 0);
	pRMSConI = Rms_(&RMS_ConI, RMSLINE_ConI, 3);
	pRMSGridV = Rms_(&RMS_GridV, RMSLINE_GridV, 6);
	pRMSGridEp = Rms_(&RMS_GridEp, RMSLINE_GridEp, 9);
	pRMSGridI = Rms_(&RMS_GridI, RMSLINE_GridI, 12);
	pRMSBypassV = Rms_(&RMS_BypV, RMSLINE_BYP, 15);
	pRMSBypassI = Rms_(&RMS_BypI, RMSLINE_BypI, 18);
}

/*******************************************************************/
/* RMS_AllCalculate() - RMS Class ��ο� ���ؼ� ���� ����          */
/* Parameters : void                                               */
/* Returns : void                                                  */
/* Called : main.c                                                 */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.01.24 By S.J.OH                                     */
/* Modification Remarks : ���� ����                                */
/* Date : 2003.04.03 By S.J.OH                                     */
/* Modification Remarks : Rms ����ü ���濡 ���� ����              */
/*******************************************************************/
/*
 * call: idlLoop()
 */
void RMS_AllCalculate(void)
{
	RMS_Calculate(pRMSConE);
	RMS_Calculate(pRMSConI);
	RMS_Calculate(pRMSGridV);
	RMS_Calculate(pRMSGridEp);
	RMS_Calculate(pRMSGridI);
	RMS_Calculate(pRMSBypassV);
	RMS_Calculate(pRMSBypassI);
}

/*******************************************************************/
/* RMS_UpdateParameter() - �Ķ���Ͱ� ����� ��쿡                */
/*       ������ �������� �����Ѵ�.                                 */
/* Parameters : void                                               */
/* Returns : void                                                  */
/* Called : main.c                                                 */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.03.09 By S.J.OH                                     */
/* Modification Remarks : ���� ����                                */
/* Date : 2003.03.17 By S.J.OH                                     */
/* Modification Remarks : '==' �� '='�� �߸��� ���� ����,          */
/*    �Է� ���ļ��� �ٲٸ� ���װ� ������, ���� �Ķ���� �ʱ�ȭ�� */
/*    �Է� OF�� �����н� OF�� �߻���                               */
/*******************************************************************/
/*
 * call: PARAM_UpdateAll()
 */
void RMS_UpdateParameter(void)
{
	// 50Hz ==> 25, 60Hz ==> 30
	if ( PRM_PCS[GRID_RATED_FREQ].iValue <= 50 )
		BufSizeMax = RMS_50HZ_BUF_SIZE;
	else
		BufSizeMax = RMS_60HZ_BUF_SIZE;
}



