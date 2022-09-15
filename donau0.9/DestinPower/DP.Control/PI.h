/*
 * PI.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef PI_H_
#define PI_H_

#include "LGS_Common.h"

typedef struct _gain
{
	float fP;  /* ��ʰ��� */
	float fIT; /* ���а��� * �����ֱ�(�ð�) */
	float fA;  /* Anti-Windup */
} Gain;

typedef struct _picon
{
	float	fIOut;
	float	fPIOut;
	float	fOut;
} PICon;

typedef struct _piargument
{
	float fErr;			// Ref - Fbk
	Gain *pK;			// pointer of PI Gains
	float fMax;			// PI Output Max
	float fMin;			// PI Output Min
	float fCompen;		// PI Controller Compensation(����)
	float fFbk;			// PI Controller Feedback
	float fAlpha;		// PI-IP ȥ�յ�, 1==>PI, 0==>IP
} PIArg;

typedef struct PICon_INCFORM_s
{
	float outDelta;
	Gain *pK;
	float ref, fbk;			// ref: ���۷���, fbk: �ǵ��
	float err, err_z; 		// err: ���۷����� �ǵ�� �������� ��(delta)
	unsigned int initFlag;  // �ʱ�ȭ �÷���. ���� ���� �� err_z���� �����Ƿ� err_z ���� ������ err ������ �ʱ�ȭ.
} PICon_INCFORM;

typedef struct PIControlUnit
{
	Gain K;
	PICon CORE;
	PIArg ARG;
} PIControlUnit;

#define PICON_RunM_INCFORM(PI_INC)\
{\
	(PI_INC).err = (PI_INC).ref - (PI_INC).fbk; \
	if(!(PI_INC).initFlag){ \
		(PI_INC).err_z = (PI_INC).err; \
		(PI_INC).initFlag = TRUE; \
	} \
	(PI_INC).outDelta = (PI_INC).pK->fP*((PI_INC).err - (PI_INC).err_z) + (PI_INC).pK->fIT*(PI_INC).err; \
	(PI_INC).err_z = (PI_INC).err; \
}

#define PICON_RunM(a, b, c) \
	pPICon = a; \
	pPIArg = b; \
    pPICon->fIOut += pPIArg->pK->fIT * ( pPIArg->fErr - (pPIArg->pK->fA * (pPICon->fPIOut - pPICon->fOut)) ); \
	pPICon->fPIOut = pPIArg->fAlpha * pPIArg->pK->fP * pPIArg->fErr - ( 1 - pPIArg->fAlpha ) * pPIArg->pK->fP * pPIArg->fFbk \
		+ pPICon->fIOut + pPIArg->fCompen; \
    pPICon->fOut = pPICon->fPIOut; \
    if ( pPICon->fOut > pPIArg->fMax ) pPICon->fOut = pPIArg->fMax; \
    else if ( pPICon->fOut < pPIArg->fMin ) pPICon->fOut = pPIArg->fMin; c = pPICon->fOut;

#define PICON_PI_ONLY_RunM(a, b) \
	pPICon = a; \
	pPIArg = b; \
    pPICon->fIOut += pPIArg->pK->fIT * ( pPIArg->fErr - (pPIArg->pK->fA * (pPICon->fPIOut - pPICon->fOut)) ); \
    if ( pPICon->fIOut > pPIArg->fMax ) pPICon->fIOut = pPIArg->fMax; \
    else if ( pPICon->fIOut < pPIArg->fMin ) pPICon->fIOut = pPIArg->fMin; \
	pPICon->fPIOut = pPIArg->pK->fP * pPIArg->fErr + pPICon->fIOut; \
    pPICon->fOut = pPICon->fPIOut; \
    if ( pPICon->fOut > pPIArg->fMax ) pPICon->fOut = pPIArg->fMax; \
    else if ( pPICon->fOut < pPIArg->fMin ) pPICon->fOut = pPIArg->fMin;

/*
 * This should be called whenever the output of the PICON is limited.
 * Otherwise anti-windup can't be calculated correctly.
 */
#define PICON_SetActualOutM(pPIConObj, LimitedOut)	\
	((pPIConObj)->fOut = LimitedOut)


float PICON_Run(PICon *this, PIArg *arg);
float PICON_PI_ONLY_Run(PICon *this, PIArg *arg);
float PICON_NO_COMPEN_Run(PICon *this, PIArg *arg);

void PICON_Initialize(PICon *this);
void PICON_Initialize2(PICon *this, float IOut);

PICon *PICon_(PICon *this);
PIArg *PIArg_(PIArg * this, Gain *pK);



#endif /* PI_H_ */
