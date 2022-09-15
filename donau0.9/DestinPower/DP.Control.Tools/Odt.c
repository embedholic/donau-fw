/*
 * Odt.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#include "odt.h"


Uns ODT_Update(Odt *this, Bool TrueFalseStatement)
{
	if ( TrueFalseStatement == FALSE )
	{
		this->bLiveFlag = OFF;
		return ODT_BROKEN;
	}
	else /* TrueFalseStatement == TRUE */
	{
		if ( this->bLiveFlag == OFF )
		{
			if ( this->uHoldTime == ZERO )   /* �ּ������ð��� */
				return ODT_FINISH;           /* Period���� ���� �� */
			this->uCount = 0; /* Timer Reset & Start */
			this->bLiveFlag = ON;
			return ODT_ING;
		}
		else /* this->bLiveFlag == ON */
		{
			this->uCount++;
			if ( this->uHoldTime <= this->uCount ) /* ���� */
			{
				this->uCount = this->uHoldTime;
				return ODT_FINISH;
			}
			else /* �����ð� �������� */
				return ODT_ING;
		}
	}
}

void ODT_Initialize(Odt *this)
{
	this->uCount = 0;
	this->bLiveFlag = OFF;
}

//TODO - Check here. 28346���� Uns�� 16��Ʈ �̹Ƿ�, �Ű� ���� ���� Ŭ ��� ���� �߸��� ������ �־ Uns�� Uint32�� ������.
Odt *Odt_(Odt	*this, Uint32 uHoldTime_msec, Uns uPeriod_msec)
{
	this->uHoldTime = uHoldTime_msec / uPeriod_msec; // msec����
	this->uPeriod = uPeriod_msec; // msec����
	this->uCount = 0;
	this->bLiveFlag = OFF;

	return this;
}

void ODT_HoldTimeChange(Odt *this, Uns uHoldTime_msec)
{
	this->uHoldTime = uHoldTime_msec / this->uPeriod; // msec����
}


