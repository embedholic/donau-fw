/*
 * PI.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */



#include "PI.h"
//-#include <stdlib.h>
#include "FAULT.h"
#include "MathConst.h"

/*******************************************************************/
/* PICON_Run - PI����� Run                                        */
/* Parameters : PICon *this => PI Obj.�� Pointer                   */
/*              PIArg *arg => PI Controller �Ű��������� ������    */
/* Returns : float => PI ����� ���                               */
/*******************************************************************/
//-TODO DATA REGION: #pragma CODE_SECTION(PICON_Run, ".iram")
float PICON_Run(PICon *this, PIArg *arg)
{
    this->fIOut += arg->pK->fIT * ( arg->fErr - (arg->pK->fA * (this->fPIOut - this->fOut)) );
    if ( this->fIOut > arg->fMax ) this->fIOut = arg->fMax;
    else if ( this->fIOut < arg->fMin ) this->fIOut = arg->fMin;
	this->fPIOut = arg->fAlpha * arg->pK->fP * arg->fErr - ( 1 - arg->fAlpha ) * arg->pK->fP * arg->fFbk
		+ this->fIOut + arg->fCompen;

    this->fOut = this->fPIOut;
    if ( this->fOut > arg->fMax ) this->fOut = arg->fMax;
    else if ( this->fOut < arg->fMin ) this->fOut = arg->fMin;

    return this->fOut;
}

// PI ���� ����, alpha �׻� 1 & ������ ����
float PICON_PI_ONLY_Run(PICon *this, PIArg *arg)
{
 //   this->fIOut += arg->pK->fIT * ( arg->fErr - (arg->pK->fA * (this->fPIOut - this->fOut)) );
    this->fIOut += arg->pK->fIT * ( arg->fErr );
    if ( this->fIOut > arg->fMax ) this->fIOut = arg->fMax;
    else if ( this->fIOut < arg->fMin ) this->fIOut = arg->fMin;
	this->fPIOut = arg->pK->fP * arg->fErr + this->fIOut;

    this->fOut = this->fPIOut;
    if ( this->fOut > arg->fMax ) this->fOut = arg->fMax;
    else if ( this->fOut < arg->fMin ) this->fOut = arg->fMin;

    return this->fOut;
}

// ������ ����
float PICON_NO_COMPEN_Run(PICon *this, PIArg *arg)
{
    this->fIOut += arg->pK->fIT * ( arg->fErr - (arg->pK->fA * (this->fPIOut - this->fOut)) );
    if ( this->fIOut > arg->fMax ) this->fIOut = arg->fMax;
    else if ( this->fIOut < arg->fMin ) this->fIOut = arg->fMin;
	this->fPIOut = arg->fAlpha * arg->pK->fP * arg->fErr - ( 1 - arg->fAlpha ) * arg->pK->fP * arg->fFbk
		+ this->fIOut ;

    this->fOut = this->fPIOut;
    if ( this->fOut > arg->fMax ) this->fOut = arg->fMax;
    else if ( this->fOut < arg->fMin ) this->fOut = arg->fMin;

    return this->fOut;
}

#if 0
float PICON_FollowRun(PICon *this, PIArg *arg)
{
    this->fIOut += arg->pK->fIT * arg->fErr;
    if ( this->fIOut > PI ) this->fIOut += -TWO_PI;
    else if ( this->fIOut < -PI ) this->fIOut += TWO_PI;
	this->fPIOut = arg->pK->fP * arg->fErr 	+ this->fIOut;
    if ( this->fPIOut > PI ) this->fPIOut += -TWO_PI;
    else if ( this->fPIOut < -PI ) this->fPIOut += TWO_PI;

    this->fOut = this->fPIOut;

    return this->fOut;
}
#endif

/*******************************************************************/
/* PICON_Initialize - PI����� �ʱ�ȭ                              */
/* Parameters : PICon *this => PI Obj.�� Pointer                   */
/* Returns : void                                                  */
/*******************************************************************/
//-TODO DATA REGION: #pragma CODE_SECTION(PICON_Initialize, ".iram")
void PICON_Initialize(PICon *this)
{
	this->fIOut = 0.;
	this->fPIOut = 0.;
	this->fOut = 0.;
}

void PICON_Initialize2(PICon *this, float IOut)
{
	this->fIOut = IOut;
	this->fPIOut = 0.;
	this->fOut = 0.;
}

/*******************************************************************/
/* PICon_ - PICon Obj. ����                                        */
/* Parameters : void                                               */
/* Returns : PICon *                                               */
/*******************************************************************/
//-TODO DATA REGION: #pragma CODE_SECTION(PICon_, ".iram")
PICon *PICon_(PICon *this)
{
//	PICon	*this;

//	if ( (this = (PICon *)malloc(sizeof(PICon)) ) == NULL ) FLT_Raise(FLT_MALLOC);
	this->fIOut = 0.;
	this->fPIOut = 0.;
	this->fOut = 0.;
	return this;
}

#if 0
/*******************************************************************/
/* PICon__ - PICon Obj. �Ҹ�                                       */
/* Parameters : PICon *                                            */
/* Returns : void                                                  */
/*******************************************************************/
void PICon__(PICon *this)
{
	free(this);
}
#endif

/*******************************************************************/
/* PIArg_ - PIArg Obj. ����                                        */
/* Parameters : Gain *pK => pointer of PI controller gain object   */
/*              float *pT => pointer of PI controller period       */
/* Returns : PIArg *                                               */
/*******************************************************************/
//-TODO DATA REGION: #pragma CODE_SECTION(PIArg_, ".iram")
PIArg *PIArg_(PIArg * this, Gain *pK)
{
//	PIArg	*this;

//	if ( (this = (PIArg *)malloc(sizeof(PIArg)) ) == NULL ) FLT_Raise(FLT_MALLOC);
	this->fErr = 0.;
	this->pK = pK;
	this->fMax = 0.;
	this->fMin = 0.;
	this->fFbk = 0.;
	this->fCompen = 0.;
	this->fAlpha = 1.; // 1 ==> PI����, 0 ==> IP����
	return this;
}

#if 0
/*******************************************************************/
/* PIArg__ - PIArg Obj. �Ҹ�                                       */
/* Parameters : PIArg *                                            */
/* Returns : void                                                  */
/*******************************************************************/
void PIArg__(PIArg *this)
{
	free(this);
}
#endif

#if 0
/*******************************************************************/
/* Gain_ - Gain Obj. ����                                          */
/* Parameters : void                                               */
/* Returns : Gain *                                                */
/*******************************************************************/
Gain *Gain_(Parameter *pP, Parameter *pI)
{
	Gain	*this;

	if ( ( this = (Gain *)malloc(sizeof(Gain)) ) == NULL ) FLT_Raise(FLT_MALLOC);
	this->pP = pP;
	this->pI = pI;
	return this;
}

/*******************************************************************/
/* Gain__ - Gain Obj. �Ҹ�                                         */
/* Parameters : Gain *                                             */
/* Returns : void                                                  */
/*******************************************************************/
void Gain__(Gain *this)
{
	free(this);
}
#endif


