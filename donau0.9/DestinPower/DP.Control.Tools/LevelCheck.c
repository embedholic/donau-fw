/*
 * LevelCheck.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */


#include <string.h>
#include "LevelCheck.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "SYSTEM.h"
#include "MathConst.h"


//#define LVL_HOLD_COUNT 3 /* 60 msec */
#define LVL_CLEAR_COUNT 25 /* 500msec ( LVL_CheckOverClear is called 20msec ) */

//-#pragma DATA_SECTION(ObjLVL, ".iramdata")
LVL ObjLVL;

void LVL_Create(void)
{
	memset(&ObjLVL, 0, sizeof(LVL));

	//LVL_UpdateAllLevel();
}

void LVL_Initialize( LevelCheck *this, float Level, float Hysteresis, Uns uTripCnt)
{
	this->fLevel = Level;
	this->fHys = Hysteresis;
	this->uTripCnt = uTripCnt;
}

void LVL_CheckOver(LevelCheck *this, float Value)
{
	if ( !this->bFlag )
	{	// Over�� �ƴ� ����, ������¸� �����ð� ������ ��� ����
		if ( Value >= this->fLevel ) ++this->uCnt;
		else this->uCnt = 0;

		// 160401 canada
		if((INVERTER.uStatus == SYS_INV_RUN && PRM_PCS[BYP_MODE].iValue == 100))
		{
			if ( this->uCnt >= 4 /*Parameter?*/)
				this->bFlag = TRUE; // Over �߻� !
			this->uClearCnt = 0;
		}
		else
		{
			if ( this->uCnt >= this->uTripCnt )
				this->bFlag = TRUE; // Over �߻� !
			this->uClearCnt = 0;
		}

		/*
		if ( this->uCnt >= PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue ) this->bFlag = TRUE; // Over �߻� !
		this->uClearCnt = 0;
		*/
	}
}

void LVL_CheckOverClear(LevelCheck *this, float Value)
{
	if ( this->bFlag )
	{	// Over ���¿��� �����׸��ý� ���� �Ʒ��� ���������� ������������ ����
		if ( Value <= (this->fLevel - this->fHys) ) ++this->uClearCnt;
		else this->uClearCnt = 0;

		if ( this->uClearCnt >= LVL_CLEAR_COUNT ) this->bFlag = FALSE; // Over ���� !
		this->uCnt = 0;
	}
}

// for debug
int uCnt;
float value;
void LVL_CheckUnder(LevelCheck *this, float Value)
{
	if ( !this->bFlag )
	{	// Under�� �ƴ� ����, ������¸� �����ð� ������ ��� ����
		if ( Value <= this->fLevel )
			++this->uCnt;
		else
			this->uCnt = 0;

		if( this->uCnt > 0 )
		{
			uCnt = this->uCnt;
			value = Value;
		}

		// 160401 canada
		if(INVERTER.uStatus == SYS_INV_RUN && (PRM_PCS[BYP_MODE].iValue == 100 || PRM_PCS[BYP_MODE].iValue == 101))
		{
			if ( this->uCnt >= 4 /*Parameter?*/)
				this->bFlag = TRUE; // Under �߻� !
			this->uClearCnt = 0;
		}
		else
		{
			if ( this->uCnt >= this->uTripCnt )
				this->bFlag = TRUE; // Under �߻� !
		}

		this->uClearCnt = 0;
		/*
		if ( this->uCnt >= PRM_PCS[CTRL_VOLT_DEVIATION_TIME].iValue ) this->bFlag = TRUE; // Under �߻� !
		this->uClearCnt = 0;
		*/
	}
}

void LVL_CheckUnderClear(LevelCheck *this, float Value)
{
	if ( this->bFlag )
	{	// Under ���¿��� �����׸��ý� ���� ���� ���������� �ö󰡴����� ����
		if ( Value >= (this->fLevel + this->fHys) ) ++this->uClearCnt;
		else this->uClearCnt = 0;
		if ( this->uClearCnt >= LVL_CLEAR_COUNT ) this->bFlag = FALSE; // Under ���� !
		this->uCnt = 0;
	}
}


