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
			if ( this->uHoldTime == ZERO )   /* 최소유지시간이 */
				return ODT_FINISH;           /* Period보다 작을 때 */
			this->uCount = 0; /* Timer Reset & Start */
			this->bLiveFlag = ON;
			return ODT_ING;
		}
		else /* this->bLiveFlag == ON */
		{
			this->uCount++;
			if ( this->uHoldTime <= this->uCount ) /* 성공 */
			{
				this->uCount = this->uHoldTime;
				return ODT_FINISH;
			}
			else /* 유지시간 넘지못함 */
				return ODT_ING;
		}
	}
}

void ODT_Initialize(Odt *this)
{
	this->uCount = 0;
	this->bLiveFlag = OFF;
}

//TODO - Check here. 28346에서 Uns가 16비트 이므로, 매개 변수 값이 클 경우 값이 잘리는 문제가 있어서 Uns를 Uint32로 변경함.
Odt *Odt_(Odt	*this, Uint32 uHoldTime_msec, Uns uPeriod_msec)
{
	this->uHoldTime = uHoldTime_msec / uPeriod_msec; // msec단위
	this->uPeriod = uPeriod_msec; // msec단위
	this->uCount = 0;
	this->bLiveFlag = OFF;

	return this;
}

void ODT_HoldTimeChange(Odt *this, Uns uHoldTime_msec)
{
	this->uHoldTime = uHoldTime_msec / this->uPeriod; // msec단위
}


