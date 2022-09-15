/*
 * CTRL_INV_PQC.h
 *
 *  Created on: 2014. 4. 3.
 *      Author: Seth Oh
 */

#ifndef CTRL_INV_PQC_H_
#define CTRL_INV_PQC_H_

#include <xdc/runtime/Assert.h>
#include "LGS_Common.h"
#include "MathConst.h"
#include "PI.h"
#include "FILTER.h"
#include "PWM.h"
#include "RAMP.h"
#include "FastRms.h"
#include "EADC.h"
#include "SYSTEM.h"

typedef struct _PQControlRampUnit
{
	Ramp w_ref;
	Ramp v_ref;
} PQControlRampUnit;

typedef struct _PQControlBlock
{
	PIControlUnit PI_P;
	PIControlUnit PI_Q;
	PQControlRampUnit RAMP;
	Uns enb;
	float	p_ref;
	float	q_ref;
	float	w_ref;
	float	v_ref;
	float	w_ref_ramp;
	float	v_ref_ramp;
	float	p_compen_coeff;
	float	q_compen_coeff;
} PQControlBlock;

void CTRL_INV_PQC_UpdateParameterPIGain(void);
void CTRL_INV_PQC_UpdateParameterRamp(void);
void CTRL_INV_PQC_UpdateParameter(void);
void CTRL_INV_PQC_Create(void);



#endif /* CTRL_INV_PQC_H_ */
