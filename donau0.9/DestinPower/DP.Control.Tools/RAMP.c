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
/* Change - fDSt Value으로 기울기를 갖고 변한다                    */
/* Parameters : Ramp *this => 변경 대상 Obj.의 Pointer             */
/*              float fDSt => 최종 목표값                          */
/* Returns : float => Ramp 출력                                    */
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
/* Initialize - Ramp 초기화                                        */
/* Parameters : Ramp *this => Ramp Obj.의 Pointer                  */
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
/* SetDelta - Delta(변화량) 변경                                   */
/* Parameters : Ramp *this => 변경 대상 Obj.의 Pointer             */
/*              float fNewDelta => 새로운 Delta                    */
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
/* RAMP_SetInitOut - Soft Start 초기 시작 지점 설정                */
/* Parameters : Ramp *this => 변경 대상 Obj.의 Pointer             */
/*              float fInitOut => 초기 시작 지점                   */
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
/* Ramp_ - Ramp Obj. 생성                                          */
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
/* Modification Remarks : 프로그램 크기를 줄이기 위하여 malloc을   */
/*       이용하지 않도록 함                                        */
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
/* Ramp__ - Ramp Obj. 소멸                                         */
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


