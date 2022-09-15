/*
 * CTRL_MODE.h
 *
 *  Created on: 2014. 6. 26.
 *      Author: destinPower
 */

#ifndef CTRL_MODE_H_
#define CTRL_MODE_H_

#include <xdc/runtime/Assert.h>
#include "LGS_Common.h"

Void CTRL_MODE_Create();
int CTRL_MODE_GetCurrentSetMode();
int CTRL_MODE_GetGC_GI_Mode();
Bool CTRL_MODE_IsGIMode_Master();
Bool CTRL_MODE_IsGIMode_Slave();
Bool CTRL_MODE_Set(int mode);
Bool CTRL_MODE_DPWM_ON_POSSIBLE();
Void CTRL_MODE_CheckParam();


#endif /* CTRL_MODE_H_ */
