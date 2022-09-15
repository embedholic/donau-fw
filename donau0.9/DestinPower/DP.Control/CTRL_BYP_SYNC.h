/*
 * CTRL_BYP_SYNC.h
 *
 *  Created on: 2014. 4. 21.
 *      Author: Seth Oh
 */

#ifndef CTRL_BYP_SYNC_H_
#define CTRL_BYP_SYNC_H_

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


typedef struct _BypassSyncFilterUnit
{
	iir1st iir_w;
	iir1st iir_v;
} BypassSyncFilterUnit;

typedef struct _BypassSyncBlock
{
	Bool g_enb;
	BypassSyncFilterUnit FILTER;
	PIControlUnit PI_W;
	float	w;
	float	v;
	float sync_accept_rad;

	Odt		*odtBypNormal;
	Odt		ODT_BYP_NORMAL;
} BypassSyncBlock;

void CTRL_BYP_SYNC_Proceed(void);
void CTRL_BYP_SYNC_UpdateParameter(void);
void CTRL_BYP_SYNC_Create(void);
Bool CTRL_BYP_NormalOK(void);
Bool CTRL_BYP_PCC_SyncOK( void );



#endif /* CTRL_BYP_SYNC_H_ */
