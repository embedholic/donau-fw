/*
 * MCU_SW_Init.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include "DSP28x_Project.h" // for Enable Watch Dog

#include "LGS_Common.h"
#include "fault.h"
#include "Event.h"
#include "MCCB.h"
#include "version.h"
#include "system.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "RtcTime.h"
#include "cc.h"
#include "peak.h"
#include "trace.h"
#include "MCB.h"
#include "VariablePeriod.h"

#include "mvp.h"
#include "rms.h"
#include "DIO.h"
#include "SCI.h"
#include "Version.h"
#include "Queue.h"
#include "MO.h"
#include "CTRL_MODE.h"
#include "DBUG.h"
#include "LevelCheck.h"
#include "GPIO.h"
#include "EVE_DC_CHARGER.h"

typedef struct _SwFactory
{
	Bool isCreated;
}SwFactory;
static SwFactory this = { 0 };

//+++by JCNET
extern void DONAU_Machine_Create();
//---
void MSI_InitGlovalVariables(void);

void MSI_Create(void)
{
	if( this.isCreated )
			return;

#if DBUG_MODE || DEBUG_MODE
	TEST_Create();
#endif

	bSystemStarted = FALSE;

	MSI_InitGlovalVariables();
	VER_create();
	FLT_Create();
	QUEUE_Create();
	EVT_Create(); // fault 생성 후 event 생성.
	DIO_Create(); /* 2013 01 23 Added */
	TRC_Create();
    PWM_Create();
    RTCTIME_Create();

	#if CC_SINE_TABLE_USE
	SNT_Create(); /* Sine, Cosine Table 생성 */
	#endif

	PARA_Create();
	CC_AddInfoNode();
	SYS_AddInfoNode();
	LVL_Create();
	#if FREQ_MODE_ENB
	FREQ_AddInfoNode();
	#endif
	MCCB_Create();
    EADC_Create();
    SYS_Create(); // 순서 주의! 정격 Value들의 초기화
	MVP_Create();
	CC_Create();
//	FLT_Reset();
	FLT_Handling();
	TRC_CreateTracePool();// InfoPool이 다 만들어진 이후에 TracePool을 생성할 것
	RTCTIME_Update();
	RMS_Create();
	#if PEAK_MODULE_ENB
	PEAK_Create();
	#endif
	MCB_Create();
	FASTRMS_Create();
#if COSPHI_ENB == 1
	COSPHI_Create();
#endif

	MO_Create();
	CTRL_MODE_Create();
	GPIO_StaticSwitchOff();
	EVE_DC_Charger_Create();
	{
	    extern void DONAU_Machine_Create();
	    DONAU_Machine_Create();
	}
	this.isCreated = TRUE;

	asm(" clrc INTM"); // EINT;

	EnableDog();

}
Bool MSI_IsCreated()
{
	return this.isCreated;
}

void MSI_InitGlovalVariables(void)
{
	memset(&BATCTRL, 0, sizeof(BATCTRL));
	memset(&GRIDTHETA, 0, sizeof(GRIDTHETA));
	memset(&CONTHETA, 0, sizeof(CONTHETA));
	memset(&GRIDTHETA_AI, 0, sizeof(GRIDTHETA_AI));
	memset(&GRID_BYP_THETA, 0, sizeof(GRID_BYP_THETA));
	memset(&EXCTRL, 0, sizeof(EXCTRL));
	memset(&INVCTRL, 0, sizeof(INVCTRL));
	memset(&OPTION, 0, sizeof(OPTION));
	memset(&CTRL, 0, sizeof(CTRL));
	memset(&ACP, 0, sizeof(ACP));
	memset(&BCState, 0, sizeof(BCState));
//++JCNET
    memset(&SCCTHETA, 0, sizeof(SCCTHETA));
//--
}
