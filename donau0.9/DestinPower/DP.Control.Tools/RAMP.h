/*
 * RAMP.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef RAMP_H_
#define RAMP_H_

#include "LGS_Common.h"

typedef struct _RAMP
{
	float	fDelta;
	float	fOut;
} Ramp;

float RAMP_Change(Ramp *, float);
void RAMP_Initialize(Ramp *);
void RAMP_SetDelta(Ramp *, float);
void RAMP_SetInitOut(Ramp *, float);

Ramp *Ramp_(Ramp *this, float Delta);
//void Ramp__(Ramp *);


#endif /* RAMP_H_ */
