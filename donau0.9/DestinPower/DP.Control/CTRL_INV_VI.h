/*
 * CTRL_INV_VI.h
 *
 *  Created on: 2014. 5. 16.
 *      Author: Seth
 */

#ifndef CTRL_INV_VI_H_
#define CTRL_INV_VI_H_

#include <xdc/runtime/Assert.h>
#include "LGS_Common.h"
#include "MathConst.h"
#include "Filter.h"

typedef struct _ViBlock
{
	Bool	g_enb;
	Bool	one_shot_enb;
	Bool	enb;
	float	vds_compen;
	float	vqs_compen;
	float	r;
	float	R_p;
	float	R_pi;
	float	l;
	float	L_p;
	float	L_pi;
	float	T_sec;
	float	Inverse_T_sec;
	float	t_sec;
	float	R_a;
	float	R_b;
	float	L_a;
	float	L_b;
	AllPassShiftFilter	fltIqsDelay90;
	float 	iqs_delay90;

} ViBlock;


void CTRL_INV_VI_Proceed(void);
void CTRL_INV_VI_OneShotEnable(void);
void CTRL_INV_VI_UpdateParameter(void);
void CTRL_INV_VI_Create(void);
void CTRL_INV_VI_Initialize(void);

#endif /* CTRL_INV_VI_H_ */
