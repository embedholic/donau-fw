/*
 * CTRL_GEN.h
 *
 *  Created on: 2015. 8. 28.
 *      Author: destinPower
 */

#ifndef CTRL_GEN_H_
#define CTRL_GEN_H_


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
#include "ODT.h"
//#include "CTRL_BYP_EVT_OPERATION.h"

typedef enum _GEN_POWER_CONTROL { GPC_NONE, GPC_HOLDING, GPC_DECREASE } GEN_POWER_CTRL_STATE;
typedef enum _GEN_AUTO_CHARGE_STATE { GAC_STANDBY, GAC_CHARGING } GEN_AUTO_CHARGING_STATE;
typedef struct _GeneratorSyncCheckBlock
{
	float diffThetaRadian;
	float accept_rad;
	Bool bOn;
	UInt16 SET_powerHoldTime;
	UInt16 SET_powerDecreaseTime;
	UInt16 SET_generatorMinimumPowerP;
	UInt16 SET_generatorMinimumPowerQ;
	float SET_loadPowerMeterP;
	float SET_loadPowerMeterQ;
	float SET_loadPowerMeterS;

	float holdingPowerP;		// fabs(SET_loadPowerMeter) -  generatorPower |W|
	float holdingPowerQ;		// fabs(SET_loadPowerMeter) -  generatorPower |W|
	float holdingLoadP;			//
	float holdingLoadQ;			//

	float pcsDecreasePowerP;		// fabs(SET_loadPowerMeter) -  generatorPower |W|
	float pcsDecreasePowerQ;		// fabs(SET_loadPowerMeter) -  generatorPower |W|
	float generatorExpectPowerP;
	float generatorExpectPowerQ;
	float decreaseRatio;  // holdingPower / sec
	Bool bGenNormalOk;
	Bool bPccSyncOk;
	Bool bMCOffCmd; /* 현재는 사용 안함 */
	Bool bGeneratorMC_Off_command; /* Seamless latch 상태에서 제네레이터 MC OFF 수행 요청 */

	UInt16 timer_hold_p_sec;
	UInt16 timer_decrease_p_sec;

	GEN_POWER_CTRL_STATE powerDecreaseState;
	Bool bPowerDecreaseEnable;

	Odt		*odtGenNormal;
	Odt		ODT_GEN_NORMAL;

	Odt		*odtMC6OpenCheck;
	Odt		ODT_MC6_OPEN_CHECK;

	Odt		*odtGenSyncOk;
	Odt		ODT_GEN_SYNC_OK;

	Odt		*odtGridConnectDelay;
	Odt		ODT_GridConnectDelay;

	UInt16 debug;
	UInt16 debugPrev;

	UInt16 genSw;
	UInt16 bGenVCB_On_Req; /* PULSE 신호임 */

	GEN_AUTO_CHARGING_STATE autoChargingState;

	Ramp  *pRAMPOutput;
	Ramp  *pRAMPOutputQ;
	Ramp POWER_RAMP_OUTPUT;

	Bool bRunWithoutPMS;
	Bool bSetInit_w_ramp;
	Bool bApply_w_ramp;

#if DBUG_MODE == 2 || DBUG_MODE == 1
	/* BYP MODE 20 DEBUG */
	UInt16 bypSw;
	//UInt16 genSw;
	UInt16 bBypV;
	UInt16 bGenV;
	Bool bGeneratorSyncOk;

	/* BYP MODE 30 DEBUG - GI->RESYNC에서 bGenVCB_On_Req가 Set되면 bGenVCB_Status을 1로 줘야 함. */

	UInt16 bGenVCB_Status;
#endif

} GeneratorSyncCheckBlock;

extern void gd(int status);
void CTRL_GEN_Proceed(void);
GEN_POWER_CTRL_STATE CTRL_GEN_PowerDecrease();
void CTRL_GEN_RampInit();
void CTRL_GEN_SetCharge();
void CTRL_GEN_UpdateParameter();

void CTRL_GEN_Create(void);
Bool CTRL_GEN_NormalOK(void);
Bool CTRL_GEN_PCC_SyncOK( void );
void CTRL_GEN_ClearTimer();

extern GeneratorSyncCheckBlock GenBlock;

#endif /* CTRL_GEN_H_ */
