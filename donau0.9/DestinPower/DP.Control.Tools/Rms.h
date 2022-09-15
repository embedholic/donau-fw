/*
 * Rms.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef RMS_H_
#define RMS_H_


#include "LGS_Common.h"
#include "CC.h"

#define RMS_LINE_NUM 3

typedef struct _RmsLine
{
	float fSample;
	float *pPingBuf;
	float *pPongBuf;
	Uns uSum500Count;
	float fSum500;
	float fInstantValue;
	float fValue;
} RmsLine;

typedef struct _Rms
{
	Uns uStatus;
	Bool bCalcEnable;
	Uns uPingBufSize;
	Uns uPongBufSize;
	RmsLine *pL[RMS_LINE_NUM];
} Rms;

enum { RMS_PING, RMS_PONG };

//FIXED SDRAM이 없어서 일단 버퍼 사이즈를 1로 만듬. before: 200
// 3.5kHz제어에서는 120개 정도의 버퍼를 사용한다.
// 5kHz제어에서는 170개 정도의 버퍼를 사용한다.
#define RMS_PING_BUF_SIZE 200

/////////////////////////////////////////////////////////////////////////////////////////
#define RMS_CHANGE_BODY1 \
        pRMS->uStatus = RMS_PONG; pRMS->uPongBufSize = 0;

#define RMS_CHANGE_BODY2 \
        pRMS->uStatus = RMS_PING; pRMS->uPingBufSize = 0;

// 인자 설명
//     a ==> Rms * : Ping Buf와 Pong Buf의 교채를 원하는 Rms 객체의 포인터
// Example
//     Rms *pRMS;
//     ...
//     RMS_ChangeM(pRMSInputE[0]);
// Macro 사용시 주위점
//     반드시 다음 내부변수를 정의할 것!
//     Rms *pRMS
#define RMS_ChangeM(a) \
	pRMS = a; \
	if (pRMS->uStatus == RMS_PING) { RMS_CHANGE_BODY1 } else { RMS_CHANGE_BODY2 } \
	pRMS->bCalcEnable = TRUE;

/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
#define RMS_SAMPLE_BODY1 \
		pRMS->pPingBuf[pRMS->uPingBufSize] = b; \
		if ( ++pRMS->uPingBufSize >= RMS_PING_BUF_SIZE ) pRMS->uPingBufSize = RMS_PING_BUF_SIZE - 1;

#define RMS_SAMPLE_BODY2 \
		pRMS->pPongBuf[pRMS->uPongBufSize] = b; \
		if ( ++pRMS->uPongBufSize >= RMS_PING_BUF_SIZE ) pRMS->uPongBufSize = RMS_PING_BUF_SIZE - 1;

// 인자 설명
//     a ==> Rms * : Rms 객체의 포인터
//     b ==> float : Sample Value
// Example
//     Rms *pRMS;
//     ...
//     RMS_AddSampleM(pRMSInputE[0], ConEa);
// Macro 사용시 주위점
//     반드시 다음 내부변수를 정의할 것!
//     Rms *pRMS
#define RMS_AddSampleM(a, b) \
	pRMS = a; \
	if (pRMS->uStatus == RMS_PING) { RMS_SAMPLE_BODY1 } else { RMS_SAMPLE_BODY2 }

/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
// 인자 설명
//     a ==> Rms * : Rms 객체의 포인터
//     b ==> Uns : Line Number
//     c ==> float : Sample Value
// Example
//     RMS_GetM(pRMSInputE, 0, ConEa);
#define RMS_GetM(a, b, c) ((a)->pL[(b)]->fSample = (c))

/////////////////////////////////////////////////////////////////////////////////////////

Rms *Rms_(Rms *this, RmsLine *pL, Uns BufIndex);
//void Rms__(Rms *this);
void RMS_Create(void);
void RMS_AllCalculate(void);
void RMS_UpdateParameter(void);
void RMS_Change(Rms *this);
void RMS_AddSample(Rms *this, float SampleVal);
void RMS_AddSampleAll(Rms *this);
float RMS_GetValue(Rms *this, Uns LineNum);
float RMS_GetInstantValue(Rms *this, Uns LineNum);

extern Rms *pRMSConE;
extern Rms *pRMSConI;
extern Rms *pRMSGridV;
extern Rms *pRMSGridEp;
extern Rms *pRMSGridI;
extern Rms *pRMSBypassV;
extern Rms *pRMSBypassI;



#endif /* RMS_H_ */
