/*
 * CTRL_FILTER.h
 *
 *  Created on: 2014. 6. 10.
 *      Author: Seth Oh
 */

#ifndef CTRL_FILTER_H_
#define CTRL_FILTER_H_

#include <xdc/runtime/Assert.h>
#include "LGS_Common.h"
#include "MathConst.h"
#include "PI.h"
#include "FILTER.h"

float CTRL_FILTER_GetSampleTime(Uns Period_Para_ID);
void CTRL_FILTER_Create(void);
void CTRL_FILTER_UpdateAPS(void);

#endif /* CTRL_FILTER_H_ */
