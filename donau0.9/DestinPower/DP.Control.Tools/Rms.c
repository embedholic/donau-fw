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

#define RMS_ID_NUM 7 /* RMS 계산이 필요한 성분 수 */
#define RMS_LINE_NUM 3 /* Phase Num */
#define RMS_50HZ_BUF_SIZE 25  /* 500msec/20msec, 500msec 평균을 위하여 */
#define RMS_60HZ_BUF_SIZE 30  /* 500msec/16.67msec, 500msec 평균을 위하여 */

#define RMS_NULL ((Rms *)0)

#pragma DATA_SECTION(RMS_PING_BUF, "L47")
#pragma DATA_SECTION(RMS_PONG_BUF, "L47")

// 28x 계열에서 Uns 변수는 16비트라서 메모리 범위를 벗어나게 된다.
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
static Rms RMS_GridEp;	// 현재 사용 되는 곳은 없다.
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
//-13.12.23		this->fValue = this->fSum500 * InverseBufSizeMax; // 500msec 평균

		this->fValue = this->fSum500 / BufSizeMax; // 500msec 평균
		this->uSum500Count = 0;
		this->fSum500 = 0;
	}
}


/*******************************************************************/
/* RMS_Calculate - Rms 계산                                        */
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
/* Modification Remarks : Power계산을 위하여 변경                  		*/
/*******************************************************************/

/*
 * call: RMS_AllCalculate
 */
static void RMS_Calculate(Rms *this)
{
	int Num;

	if ( this->bCalcEnable == FALSE ) return;

	if ( this->uStatus == RMS_PING ) // 현재 핑버퍼에 데이터를 모으고 있는 상태임
	{	// 따라서 다 모아진 퐁버퍼의 내용을 이용하여 rms 게산
		for ( Num = 0; Num < RMS_LINE_NUM; Num++ )
		{
			OneRMSCalc( this->pL[Num], this->pL[Num]->pPongBuf, this->uPongBufSize );
		}
	}
	else // 현재 퐁버퍼에 데이터를 모으고 있는 상태임
	{	// 따라서 다 모아진 핑버퍼의 내용을 이용하여 rms 게산
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
 * when: ThetaDetectRun() 에서 line의 fRadian이 PI 보다 크면.
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
/* AddSample - rms Sample 추가                                     */
/* Parameters : Rms *this                                          */
/*        float SampleVal : Sample된 값                            */
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
 * when: 주기마다 호출 됨.
 */
void RMS_AddSampleAll(Rms *this)
{

	if (this->uStatus == RMS_PING)
	{
		this->pL[0]->pPingBuf[this->uPingBufSize] = this->pL[0]->fSample;
		this->pL[1]->pPingBuf[this->uPingBufSize] = this->pL[1]->fSample;
		this->pL[2]->pPingBuf[this->uPingBufSize] = this->pL[2]->fSample;
		// 주파수가 비정상적으로 느린 경우 RMS_BUF_SIZE를 초과할 수 있다.
		// 이런 상황이 발생되면 프로그램은 정상적일 수 없다.
		// 주파수가 비정상적으로 느린 경우 RMS계산은 정확하지 않게 된다.
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
/* GetValue - rms 500msec 평균값 제공                              */
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
 * when: RMS된 값을 MEASURE 변수에 대입한다.
 */
float RMS_GetValue(Rms *this, Uns LineNum)
{
	return this->pL[LineNum]->fValue;
}

/*******************************************************************/
/* GetInstantValue - rms(1주기) 값 제공                            */
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
/* RmsLine_() - RmsLine_ Class 생성                                */
/* Parameters : Uns BufIndex : 핑/퐁 버퍼의 외부메모리 상대적 위치 */
/* Returns : RmsLine_ *                                            */
/* Called : Rms_()                                                 */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2003.04.03 By S.J.OH                                     */
/* Modification Remarks : 최초 버전                                */
/* Date : 2003.12.15 By sjoh                                       */
/* Modification Remarks : 프로그램 크기를 줄이기 위하여 malloc을   */
/*       이용하지 않도록 함                                        */
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

	// 500msec 평균에 대하여...
	this->uSum500Count = 0;
	this->fSum500 = 0;

	this->fValue = 0;
	this->fInstantValue = 0;

	return this;
}

/*******************************************************************/
/* Rms_() - Rms 값 Class 생성                                      */
/* Parameters : Uns BufIndex : 핑/퐁 버퍼의 외부메모리 상대적 위치 */
/* Returns : Rms *                                                 */
/* Called : RMS_Create()                                           */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.01.24 By S.J.OH                                     */
/* Modification Remarks : 최초 버전                                */
/* Date : 2002.07.26 By S.J.OH                                     */
/* Modification Remarks : 내부메모리부족 관계로 버퍼를             */
/*                        외부 메모리에 할당시킴                   */
/* Date : 2003.04.03 By S.J.OH                                     */
/* Modification Remarks : Rms 구조체 변경에 따른 수정              */
/* Date : 2003.12.15 By sjoh                                       */
/* Modification Remarks : 프로그램 크기를 줄이기 위하여 malloc을   */
/*       이용하지 않도록 함                                        */
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
/* RMS_Create() - RMS Class 모두 생성                              */
/* Parameters : void                                               */
/* Returns : void                                                  */
/* Called : main.c                                                 */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.01.24 By S.J.OH                                     */
/* Modification Remarks : 최초 버전                                */
/* Date : 2003.04.03 By S.J.OH                                     */
/* Modification Remarks : Rms 구조체 변경에 따른 수정              */
/* Date : 2003.12.15 By sjoh                                       */
/* Modification Remarks : 프로그램 크기를 줄이기 위하여 malloc을   */
/*       이용하지 않도록 함                                        */
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
/* RMS_AllCalculate() - RMS Class 모두에 대해서 연산 수행          */
/* Parameters : void                                               */
/* Returns : void                                                  */
/* Called : main.c                                                 */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.01.24 By S.J.OH                                     */
/* Modification Remarks : 최초 버전                                */
/* Date : 2003.04.03 By S.J.OH                                     */
/* Modification Remarks : Rms 구조체 변경에 따른 수정              */
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
/* RMS_UpdateParameter() - 파라미터가 변경된 경우에                */
/*       연관된 변수들을 갱신한다.                                 */
/* Parameters : void                                               */
/* Returns : void                                                  */
/* Called : main.c                                                 */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 2002.03.09 By S.J.OH                                     */
/* Modification Remarks : 최초 버전                                */
/* Date : 2003.03.17 By S.J.OH                                     */
/* Modification Remarks : '==' 를 '='로 잘못쓴 버그 수정,          */
/*    입력 주파수를 바꾸면 버그가 출현함, 따라서 파라미터 초기화시 */
/*    입력 OF와 바이패스 OF가 발생함                               */
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



