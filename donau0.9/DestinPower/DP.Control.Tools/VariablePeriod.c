/*
 * VariablePeriod.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#include "VariablePeriod.h"
#include "cc.h"

#define VPRD_PERIOD_MIN 200
#define VPRD_PERIOD_MAX 333

//-#pragma DATA_SECTION(VPRD, ".iramdata")
VariablePeriod VPRD;

static Ramp VPRD_RAMP;

void VPRD_Create( void )
{
	VPRD.pRAMP = Ramp_(&VPRD_RAMP, 1.); // 1초에 1usec 변화
	VPRD.fRef = 200.;
	VPRD.uRef = 200;
	RAMP_SetInitOut(VPRD.pRAMP, VPRD.fRef);
}

// 매 100msec 마다 실행
void VPRD_UpdateReference( void )
{
	float fPVPowerAbs, fDst;

	fPVPowerAbs = fabs(BATCTRL.fPowerFilterd_1st	) * 0.001; // W ==> kW
	if ( fPVPowerAbs <= 50. )
		fDst = (float)VPRD_PERIOD_MIN;
	else if ( fPVPowerAbs > 50. && fPVPowerAbs <= 90. )
		fDst = 3.325 * fPVPowerAbs + 33.75;
	else
		fDst = (float)VPRD_PERIOD_MAX;

	VPRD.fRef = RAMP_Change(VPRD.pRAMP, fDst );
	VPRD.uRef = VPRD.fRef;
	if ( VPRD.uRef < VPRD_PERIOD_MIN ) VPRD.uRef = VPRD_PERIOD_MIN;
	if ( VPRD.uRef > VPRD_PERIOD_MAX ) VPRD.uRef = VPRD_PERIOD_MAX;
}


