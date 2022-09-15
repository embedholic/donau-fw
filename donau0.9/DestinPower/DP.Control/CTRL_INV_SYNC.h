/*
 * CTRL_INV_SYNC.h
 *
 *  Created on: 2014. 4. 3.
 *      Author: Seth Oh
 */

#ifndef CTRL_INV_SYNC_H_
#define CTRL_INV_SYNC_H_

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

typedef struct _SyncFilterUnit
{
	iir1st iir_w;
	iir1st iir_v;

	iir1st iir_inv_v;
	//iir1st iir_pi_mag;
} SyncFilterUnit;

typedef struct _SyncBlock
{
	SyncFilterUnit FILTER;
	PIControlUnit PI_W;
	PIControlUnit PI_MAG; /* Generator */
	PIControlUnit PI_MAG_BYP; /* + bypassV */
	float	w;
	float	v;
	float pi_mag_input;
} SyncBlock;

void CTRL_INV_SYNC_Proceed(void);
void CTRL_INV_SYNC_UpdateParameter(void);
void CTRL_INV_SYNC_Create(void);


#endif /* CTRL_INV_SYNC_H_ */
