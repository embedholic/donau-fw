/*
 * CTRL_INV_VUC.h
 *
 *  Created on: 2014. 5. 16.
 *      Author: Seth
 */

#ifndef CTRL_INV_VUC_H_
#define CTRL_INV_VUC_H_

#include <xdc/runtime/Assert.h>
#include "LGS_Common.h"
#include "MathConst.h"
#include "PI.h"
#include "FILTER.h"
#include "SYSTEM.h"

typedef struct _VucFilterUnit
{
	iir1st iir_pos;
	iir1st iir_neg;
} VucFilterUnit;

typedef struct _VucBlock
{
	VucFilterUnit FILTER;
	PIControlUnit PI_UF;
	Bool	g_enb;
	Bool	enb;
//	Bool	filter_coeff_shadow_enb;
	float	vuf;
	float	vds_compen;
	float	vqs_compen;
} VucBlock;

void CTRL_INV_VUC_Proceed(void);
void CTRL_INV_VUC_UpdateParameterPIGain(void);
void CTRL_INV_VUC_UpdateParameter(void);
void CTRL_INV_VUC_Create(void);
void CTRL_INV_VUC_UpdateFilterCoefficient(void);
void CTRL_INV_VUC_CalcFilterShadow(void);

#endif /* CTRL_INV_VUC_H_ */
