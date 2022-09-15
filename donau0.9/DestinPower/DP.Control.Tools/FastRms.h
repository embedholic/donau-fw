/*
 * FastRms.h
 *
 *  Created on: 2012. 11. 7.
 *      Author: destinPower
 */

#ifndef FASTRMS_H_
#define FASTRMS_H_

#include <xdc/std.h>
#include "LGS_Common.h"

#define FASTSUM_ITEM_COUNT 6
#define FASTSUM_LINE_NUM 3
#define FASTSUM_BUF_SIZE_MAX 300

typedef struct _FASTRMS_ITEM
{
	Uint16 uState;
	Uint16 uTail;
	Uint16 uCnt;
	Uint32 u32Sum;
	//-float u32Sum;
	Uint32 *pBuf;
	//-float *pBuf;
	float fRMS2; /* RMSÀÇ Á¦°ö */
}FASTRMS;

typedef struct _FASTRMS
{
	FASTRMS fsGridV[FASTSUM_LINE_NUM];
	FASTRMS fsBypV[FASTSUM_LINE_NUM];
	FASTRMS fsConV[FASTSUM_LINE_NUM];
	FASTRMS fsConVp[FASTSUM_LINE_NUM];
	FASTRMS fsGenV[FASTSUM_LINE_NUM];

	//	16.07.01
	FASTRMS fsBypI[FASTSUM_LINE_NUM];
}FASTRMS_ITEMS;

extern FASTRMS_ITEMS FastRmsItems;

public void FASTRMS_Create();
public void FASTRMS_UpdateParameter();
public void FASTRMS_AddSample(FASTRMS *_this, float Sample );

#endif /* FASTRMS_H_ */
