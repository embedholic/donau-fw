/*
 * LevelCheck.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef LEVELCHECK_H_
#define LEVELCHECK_H_

#include "LGS_Common.h"

typedef	struct _LevelCheck
{
	float fLevel;
	float fHys;
	Bool bFlag;
	Uns uCnt;
	Uns uClearCnt;
	Uns uTripCnt;
} LevelCheck;

typedef struct _LVL
{
	LevelCheck InvOV;
	LevelCheck InvUV;
	LevelCheck InvOF;
	LevelCheck InvUF;
	LevelCheck GridOVLevel1[3];
	LevelCheck GridUVLevel1[3];
	LevelCheck GridUFLevel1;
	LevelCheck GridOFLevel1;

	LevelCheck GridUFLevel2_seamless;
	LevelCheck GridOFLevel2_seamless;

	LevelCheck GridOVLevel2[3];
	LevelCheck GridUVLevel2[3];
	LevelCheck GridUFLevel2;
	LevelCheck GridOFLevel2;

	LevelCheck BypassOVLevel1[3];
	LevelCheck BypassUVLevel1[3];
	LevelCheck BypassUFLevel1;
	LevelCheck BypassOFLevel1;

	LevelCheck GenOVLevel[3];
	LevelCheck GenUVLevel[3];
	LevelCheck GenOFLevel;
	LevelCheck GenUFLevel;
} LVL;

void LVL_Create(void);
void LVL_Initialize( LevelCheck *this, float Level, float Hysteresis, Uns uTripCnt);
void LVL_CheckOver(LevelCheck *this, float Value);
void LVL_CheckOverClear(LevelCheck *this, float Value);
void LVL_CheckUnder(LevelCheck *this, float Value);
void LVL_CheckUnderClear(LevelCheck *this, float Value);

extern LVL ObjLVL;


#endif /* LEVELCHECK_H_ */
