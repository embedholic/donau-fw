/*
 * CTRL_INV_SEAMLESS.h
 *
 *  Created on: 2016. 8. 12.
 *      Author: Yang_PC
 */

#ifndef DESTINPOWER_DP_CONTROL_CTRL_INV_SEAMLESS_H_
#define DESTINPOWER_DP_CONTROL_CTRL_INV_SEAMLESS_H_

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
#include "SagEventHistory.h"
#include "CTRL_BYP_EVT_OPERATION.h"
#include "GPIO.h"

typedef struct _InverterSeamlessBlock
{
	Bool g_enb;
	Bool pcc_blackout_enb;
	float pcc_ov_level_instant;
	float pcc_uv_level_instant;

	float pcc_ov_level2_instant;
	float pcc_uv_level2_instant;

	float pcc_uv_level1_instant_restore;
	float pcc_uv_level2_instant_restore;

	Odt		*odt;
	Odt		ODT;

	Bool enb_for_BEO;

	UInt16 PwmOffCnt;
} InverterSeamlessBlock;



void CTRL_INV_SEAMLESS_Proceed(void);
Void CTRL_INV_SEAMLESS_UpdateParameter_running();
void CTRL_INV_SEAMLESS_UpdateParameter(void);
void CTRL_INV_SEAMLESS_Reset();
void CTRL_INV_SEAMLESS_Create(void);
Bool CTRL_INV_SEAMLESS_PccNormalOK(void);
Bool CTRL_INV_SEAMLESS_PccBlackout(void);
extern Bool CTRL_Vpcc_NormalOK(void);
extern Uns ctrl_seamless_uv_cnt;
extern Uns ctrl_seamless_ov_cnt;
extern Uns ctrl_seamless_uv_lv2_cnt;
extern Uns ctrl_seamless_ov_lv2_cnt;
extern void CTRL_SEAMLESS_PWM_OFF(void);

#endif /* DESTINPOWER_DP_CONTROL_CTRL_INV_SEAMLESS_H_ */
