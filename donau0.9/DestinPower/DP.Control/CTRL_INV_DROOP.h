/*
 * CTRL_INV_DROOP.h
 *
 *  Created on: 2014. 4. 3.
 *      Author: Seth Oh
 */

#ifndef CTRL_INV_DROOP_H_
#define CTRL_INV_DROOP_H_

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

typedef struct _DroopFilterUnit
{
	iir1st iir_Z_tr;
} DroopFilterUnit;

typedef struct _DroopGainUnit
{
	float M;
	float N;
} DroopGainUnit;

typedef struct _DroopRampUnit
{
	Ramp v_ref;
	Ramp p_ref;
	Ramp q_ref;
} DroopRampUnit;

typedef struct _DroopBlock
{
	DroopFilterUnit	FILTER;
	DroopGainUnit	GAIN;
	DroopRampUnit	RAMP;
	Bool		enb;
	Bool		power_fbk_enb;
	float		Omega_ref;
	float		Voltage_ref;
	float		w_ref;
	float		w_sec;
	float		w_ref_sec;
	float		v_ref;
	float		v_sec;
	float		v_ref_sec;
	float		v_ref_tr;
	float		v_ref_ramp;
	float		w_ref_ramp; /* 151215 */

} DroopBlock;

typedef struct _VirtualImpedanceBlock
{
	Bool		enb;
	Bool		start;
	float		Time;
	float		R_p;
	float		R_p_i;
	float		r;

} VirtualImpedanceBlock;


void CTRL_INV_DROOP_Control(void);
void CTRL_INV_DROOP_UpdateRamp(void);
void CTRL_INV_DROOP_GI_V_Update();
void CTRL_INV_DROOP_UpdateParameter(void);
void CTRL_INV_DROOP_1Sec_func();
void CTRL_INV_DROOP_100ms_func();
void CTRL_INV_DROOP_Decrease_w();
void CTRL_INV_DROOP_Create(void);

#endif /* CTRL_INV_DROOP_H_ */
