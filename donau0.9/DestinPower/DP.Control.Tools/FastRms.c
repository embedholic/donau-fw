/******************************************************************************
 *
 * 파일명    : FastRms.c
 * 작성자    : Destin Power
 * 목적      : 순간 적인 RMS 값 계산
 * 사용방식  : Crate() -> public functions
 * 사용파일  :
 * 제한사항  : CC와 Parameter 전역 변수를 사용함.
 * 오류처리  :
 * 이력사항
 * 			2012. 11/07 신요섭
 * 			1. 최초 생성
 */

#include <string.h>
#include "FastRms.h"
#include "LGS_Common.h"
#include "CC.h"
#include "Parameter.h"
#include "prm_pcs.h"
#include "Fault.h"

enum
{
	STATE_INIT,
	STATE_COLLECTING,
	STATE_RUN
};

public FASTRMS_ITEMS FastRmsItems;

#if DBUG_MODE == 3
Uint16 uN; /* 전원 한주기에 대한 샘플 개수 = 1/ (f*Ts) */
float fNInverse; /* 1/uN */
UInt32 BUFFER[(FASTSUM_ITEM_COUNT*FASTSUM_LINE_NUM)*FASTSUM_BUF_SIZE_MAX];
#else
private Uint16 uN; /* 전원 한주기에 대한 샘플 개수 = 1/ (f*Ts) */
private float fNInverse; /* 1/uN */
private UInt32 BUFFER[(FASTSUM_ITEM_COUNT*FASTSUM_LINE_NUM)*FASTSUM_BUF_SIZE_MAX];
#endif


/*
 * CC보다 늦게 생성되어야 함
 */
public void FASTRMS_Create()
{
	FASTRMS_UpdateParameter();

	memset(&FastRmsItems, 0 , sizeof(FastRmsItems)); 	/* Checked : check sizeof FastSum */
	memset(&BUFFER, 0, sizeof(BUFFER)); /* Checked : check sizeof FastSum */


	FastRmsItems.fsGridV[0].pBuf = (Uint32 *)BUFFER;
	FastRmsItems.fsGridV[1].pBuf = (Uint32 *)BUFFER + (1 * FASTSUM_BUF_SIZE_MAX);
	FastRmsItems.fsGridV[2].pBuf = (Uint32 *)BUFFER + (2 * FASTSUM_BUF_SIZE_MAX);

	FastRmsItems.fsConV[0].pBuf = (Uint32 *)BUFFER + (3 * FASTSUM_BUF_SIZE_MAX);
	FastRmsItems.fsConV[1].pBuf = (Uint32 *)BUFFER + (4 * FASTSUM_BUF_SIZE_MAX);
	FastRmsItems.fsConV[2].pBuf = (Uint32 *)BUFFER + (5 * FASTSUM_BUF_SIZE_MAX);

	FastRmsItems.fsConVp[0].pBuf = (Uint32 *)BUFFER + (6 * FASTSUM_BUF_SIZE_MAX);
	FastRmsItems.fsConVp[1].pBuf = (Uint32 *)BUFFER + (7 * FASTSUM_BUF_SIZE_MAX);
	FastRmsItems.fsConVp[2].pBuf = (Uint32 *)BUFFER + (8 * FASTSUM_BUF_SIZE_MAX);

	FastRmsItems.fsBypV[0].pBuf = (Uint32 *)BUFFER + (9 * FASTSUM_BUF_SIZE_MAX);
	FastRmsItems.fsBypV[1].pBuf = (Uint32 *)BUFFER + (10 * FASTSUM_BUF_SIZE_MAX);
	FastRmsItems.fsBypV[2].pBuf = (Uint32 *)BUFFER + (11 * FASTSUM_BUF_SIZE_MAX);

	FastRmsItems.fsGenV[0].pBuf = (Uint32 *)BUFFER + (12 * FASTSUM_BUF_SIZE_MAX);
	FastRmsItems.fsGenV[1].pBuf = (Uint32 *)BUFFER + (13 * FASTSUM_BUF_SIZE_MAX);
	FastRmsItems.fsGenV[2].pBuf = (Uint32 *)BUFFER + (14 * FASTSUM_BUF_SIZE_MAX);

	FastRmsItems.fsBypI[0].pBuf = (Uint32 *)BUFFER + (15 * FASTSUM_BUF_SIZE_MAX);
	FastRmsItems.fsBypI[1].pBuf = (Uint32 *)BUFFER + (16 * FASTSUM_BUF_SIZE_MAX);
	FastRmsItems.fsBypI[2].pBuf = (Uint32 *)BUFFER + (17 * FASTSUM_BUF_SIZE_MAX);
}

/*
 * 샘플 개수 업데이트 및 초기 상태로 변경
 * 계산 값 확인 : OK
 *
 * 150520 June. warn: system memory err when FREQ < 24(uN이 300보다 크게 되어 BOV발생.)
 */
public void FASTRMS_UpdateParameter()
{
	/*Calc (1/F) / ccPeriod = 1/F*ccPeriod */
	uN = (1. / (CC_tsSample * (float)PRM_PCS[GRID_RATED_FREQ].iValue)) + 0.5;

	//+150520 June. System memory crash 방지.
	if( uN > FASTSUM_BUF_SIZE_MAX )
		uN = FASTSUM_BUF_SIZE_MAX;

	fNInverse = 1. / uN;

	FastRmsItems.fsGridV[0].uState = STATE_INIT;
	FastRmsItems.fsGridV[1].uState = STATE_INIT;
	FastRmsItems.fsGridV[2].uState = STATE_INIT;

	FastRmsItems.fsConV[0].uState = STATE_INIT;
	FastRmsItems.fsConV[1].uState = STATE_INIT;
	FastRmsItems.fsConV[2].uState = STATE_INIT;

	FastRmsItems.fsConVp[0].uState = STATE_INIT;
	FastRmsItems.fsConVp[1].uState = STATE_INIT;
	FastRmsItems.fsConVp[2].uState = STATE_INIT;

	FastRmsItems.fsBypV[0].uState = STATE_INIT;
	FastRmsItems.fsBypV[1].uState = STATE_INIT;
	FastRmsItems.fsBypV[2].uState = STATE_INIT;

	FastRmsItems.fsGenV[0].uState = STATE_INIT;
	FastRmsItems.fsGenV[1].uState = STATE_INIT;
	FastRmsItems.fsGenV[2].uState = STATE_INIT;

	FastRmsItems.fsBypI[0].uState = STATE_INIT;
	FastRmsItems.fsBypI[1].uState = STATE_INIT;
	FastRmsItems.fsBypI[2].uState = STATE_INIT;
}

#define PRECISION_LEVEL	(10)
#define PRECISION_ROUNDING_LEVEL	(0.5)
/*
 * 샘플 추가
 * 계산값 확인:
 * Uns 범위 확인(16비트를 써도 안전한지.):
 */
public void FASTRMS_AddSample(FASTRMS *_this, float Sample )
{
	Uint32 uOldest;
	Uint32 uNew;
	//-float uOldest;
	//-float uNew;

	/*
	 * Rouding off the float value
	 * The FASTSUM keeps accumulated value then
	 * remove the oldest sample and add new sample.
	 * So calculation error may be significant.
	 * (maybe so not using float type)
	 */

	// 음수일 경우 0으로 입력 되어 Sample되지 못하는 버그 수정.
	if(Sample < 0)
	{
		Sample = -Sample;
	}
	uNew = (Uint32)(Sample*PRECISION_LEVEL + PRECISION_ROUNDING_LEVEL);
	uNew = uNew * uNew;
	//-uNew = Sample * Sample;

	switch ( _this->uState )
	{
		case STATE_INIT:
			_this->uCnt = 0;
			_this->u32Sum = 0;
			_this->uTail = 0;
			_this->uState = STATE_COLLECTING;
			break;
		case STATE_COLLECTING:
			_this->pBuf[_this->uCnt++] = uNew;
			_this->u32Sum += uNew;
			if (_this->uCnt >= uN)
				_this->uState = STATE_RUN;
			break;
		case STATE_RUN:
			uOldest = _this->pBuf[_this->uTail];
			_this->u32Sum = _this->u32Sum - uOldest + uNew;

			_this->pBuf[_this->uTail] = uNew;
			if ( ++_this->uTail >= uN ) _this->uTail = 0;
			/*
			 * Result is squared value
			 */
			/*calc*/
			_this->fRMS2 = _this->u32Sum /(PRECISION_LEVEL*PRECISION_LEVEL) * fNInverse;
			//-this->fRMS2 = this->u32Sum * fNInverse;
			break;
		default:
			_this->uState = STATE_COLLECTING;
	}
}
