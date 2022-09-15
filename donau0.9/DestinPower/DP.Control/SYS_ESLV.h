/*
 * SYS_ESLV.h
 *
 *  Created on: 2014. 11. 6.
 *      Author: destinPower
 */

#include <xdc/runtime/Assert.h>
#include "LGS_Common.h"

typedef enum { ESLV_CTRL_ING, ESLV_CTRL_SUCC, ESLV_CTRL_FAIL } ESLV_RETURN;

typedef struct {
	Bool g_bEnb;
}ESLV;

void SYS_ESLV_Create();
void SYS_ESLV_UpdateParameter();
ESLV_RETURN SYS_ESLV_BYP_Control();
Bool SYS_ESLV_BYP_Resync();

extern ESLV eslvCtrl;


