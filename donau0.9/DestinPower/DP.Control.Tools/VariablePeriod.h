/*
 * VariablePeriod.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef VARIABLEPERIOD_H_
#define VARIABLEPERIOD_H_

#include "LGS_Common.h"
#include "ramp.h"

typedef struct _VariablePeriod
{
	float fRef;
	Uns uRef;
	Ramp  *pRAMP;
} VariablePeriod;

void VPRD_Create();
void VPRD_UpdateReference();

extern VariablePeriod VPRD;


#endif /* VARIABLEPERIOD_H_ */
