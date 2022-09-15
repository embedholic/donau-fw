/*
 * BATTCB.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef BATTCB_H_
#define BATTCB_H_

#include "LGS_Common.h"

typedef enum
{
	CB_RESET,
	CB_OFF,
	CB_ON,
	CB_TRIP
}MCB_STATUS;
typedef enum
{
	CB_CMD_RESET,
	CB_CMD_OFF,
	CB_CMD_ON,
	CB_CMD_TRIP
}M_CB_CMD;

typedef enum
{
//by JCNET	M_CB1_BATT, M_CB4_SSW , M_CB3
    M_CB1_BATT, M_CB2_SSW, M_CB1, M_CB2, M_CB3
}M_CB_LIST;

typedef enum {
	CBT_NORMAL,
	CBT_F_G_UVR,
	CBT_END
}e_CB_TYPE;

void MCB_Create();
void MCB_UpdateCmd(M_CB_LIST cbId, M_CB_CMD Cmd);
void SYS_UpdateMCbStatus(M_CB_LIST cbId);
Uns MCB_GetStatus(M_CB_LIST cbId);
Bool MCB_SetType(M_CB_LIST cbId, e_CB_TYPE type );

#endif /* BATTCB_H_ */
