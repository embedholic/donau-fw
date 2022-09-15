/*
 * RAMP.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */



#include "RAMP.h"
//-#include <stdlib.h>
#include "FAULT.h"

/*******************************************************************/
/* Change - fDSt Value���� ���⸦ ���� ���Ѵ�                    */
/* Parameters : Ramp *this => ���� ��� Obj.�� Pointer             */
/*              float fDSt => ���� ��ǥ��                          */
/* Returns : float => Ramp ���                                    */
/* Called : ConverterVoltControl()(CVC.c),                         */
/*          InverterVoltControl()(IVC.c)                           */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 1999.09.07 By sjoh                                       */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/
//-#pragma CODE_SECTION(RAMP_Change, ".iram")
float RAMP_Change(Ramp *this, float fDSt)
{
	float diff;

	diff = fDSt - this->fOut;

	if ( diff >= this->fDelta ) this->fOut = this->fOut + this->fDelta;
	else if ( diff <= -this->fDelta ) this->fOut = this->fOut - this->fDelta;
	else this->fOut = fDSt;

	return this->fOut;
}

/*******************************************************************/
/* Initialize - Ramp �ʱ�ȭ                                        */
/* Parameters : Ramp *this => Ramp Obj.�� Pointer                  */
/* Returns : void                                                  */
/* Called : CVC_Initialize()(CVC.c),                               */
/*          IVC_Initialize()(IVC.c)                                */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 1999.09.07 By sjoh                                       */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/
//-#pragma CODE_SECTION(RAMP_Initialize, ".iram")
void RAMP_Initialize(Ramp *this)
{
	this->fOut = 0.;
}

/*******************************************************************/
/* SetDelta - Delta(��ȭ��) ����                                   */
/* Parameters : Ramp *this => ���� ��� Obj.�� Pointer             */
/*              float fNewDelta => ���ο� Delta                    */
/* Returns : void                                                  */
/* Called : CVC_StartRamp(), CVC_UpdateParameter()(CVC.c)          */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 1999.09.07 By sjoh                                       */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/
//-#pragma CODE_SECTION(RAMP_SetDelta, ".iram")
void RAMP_SetDelta(Ramp *this, float fNewDelta)
{
	this->fDelta = fNewDelta;
}

/*******************************************************************/
/* RAMP_SetInitOut - Soft Start �ʱ� ���� ���� ����                */
/* Parameters : Ramp *this => ���� ��� Obj.�� Pointer             */
/*              float fInitOut => �ʱ� ���� ����                   */
/* Returns : void                                                  */
/* Called : CVC_StartRamp()(CVC.c)                                 */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 1999.09.07 By sjoh                                       */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/
//-#pragma CODE_SECTION(RAMP_SetInitOut, ".iram")
void RAMP_SetInitOut(Ramp *this, float fInitOut)
{
	this->fOut = fInitOut;
}

/*******************************************************************/
/* Ramp_ - Ramp Obj. ����                                          */
/* Parameters : void                                               */
/* Returns : Ramp *                                                */
/* Called : CC_Create()(CC.c)                                      */
/* Related :                                                       */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 1999.09.07 By sjoh                                       */
/* Modification Remarks : Initial Version                          */
/* Date : 2003.12.15 By sjoh                                       */
/* Modification Remarks : ���α׷� ũ�⸦ ���̱� ���Ͽ� malloc��   */
/*       �̿����� �ʵ��� ��                                        */
/*******************************************************************/
//-#pragma CODE_SECTION(Ramp_, ".iram")
Ramp *Ramp_(Ramp *this, float Delta)
{
//	Ramp	*this;

//	if ( (this = (Ramp *)malloc(sizeof(Ramp)) ) == NULL ) FLT_Raise(FLT_MALLOC);
	this->fDelta = Delta;
	this->fOut = 0.;
	return this;
}

#if 0
/*******************************************************************/
/* Ramp__ - Ramp Obj. �Ҹ�                                         */
/* Parameters : void                                               */
/* Returns : void                                                  */
/* Called :                                                        */
/* Related : VC                                                    */
/*-----------------------------------------------------------------*/
/* Example :                                                       */
/*-----------------------------------------------------------------*/
/* Date : 1999.09.07 By sjoh                                       */
/* Modification Remarks : Initial Version                          */
/*******************************************************************/
void Ramp__(Ramp *this)
{
	free(this);
}
#endif


