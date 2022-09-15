/*
 * PhaseCheck.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef PHASECHECK_H_
#define PHASECHECK_H_

#include "LGS_Common.h"

typedef struct _PhaseCheck
{
	int iFcnt;
	float fVaPrev;
} PhaseCheck;

extern PhaseCheck GPC;
extern PhaseCheck IPC;
extern PhaseCheck BYP;

void PHS_Check( PhaseCheck *this , float Va, float Vb, float Vc , int num);


#endif /* PHASECHECK_H_ */
