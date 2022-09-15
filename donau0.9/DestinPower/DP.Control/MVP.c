/*
 * MVP.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#include "MVP.h"
#include "MathConst.h"
#include "RMS.h"
#include "FILTER.h"
#include "SYSTEM.h"
#include "ODT.h"
#include "PEAK.h"
#include "EADC.h"
#include "cc.h"
#include "trace.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "FAULT.h"
#include "CTRL_INV_PRC.h"
#include "Event.h"

#define MVP_LINE_NUM 3

#define MVP_100MSEC_PERIOD 	5
#define MVP_1SEC_PERIOD 	50
#define MVP_PERIOD	20e-3

#if DBUG_MODE == 2
#define MVP_WH_FACTOR		0.00033333 /* 20ms, 1분 기준 Wh */
#else
#define MVP_WH_FACTOR		0.0000055555 /* 20ms, Wh */
#endif


//-extern far STS_Obj sts20msec;
extern	AC_Panel	ACP;

Uns Counter100msec = 0;
Uns Counter1sec = 0;
Bool b100msecTick = FALSE;
Bool bOneSecondTick = FALSE;
Bool bAfterInit5Second = FALSE;

#if MVP_EXECUTION_PERIOD
#pragma DATA_SECTION(MVP_startTime, ".iramdata")
#pragma DATA_SECTION(MVP_endTime, ".iramdata")
Uns MVP_startTime;
Uns MVP_endTime;
Uns MVP_uExecutionPeriod;
#endif

void MVP_PowerAccumulate_mapping();
void MVP_PowerAccumulate();


void MVP_Create( void )
{
	MVP_AddTrace();
	MVP_PowerAccumulate_mapping();
}

void MVP_Process10msec( void )
{
	if( bAfterInit5Second )
	{
		SYS_VoltFreqMonitor();
	}
}

void MVP_Process20msec( void )
{
	static Uns uInit5SecondCnt = 0;

	static Uns uUnbalanceCnt = 0;
	int i;
	float a=0, b=0, c= 0;
	float Ta =0, Tb =0, Tc=0;

	if(!bSystemStarted)
		return;

#if MVP_EXECUTION_PERIOD
	MVP_startTime = CLK_gethtime();
	if ( MVP_startTime < MVP_endTime )
		MVP_uExecutionPeriod = (0xFFFFFFFF - MVP_endTime + MVP_startTime) * 0.0266667;
	else
		MVP_uExecutionPeriod = (MVP_startTime - MVP_endTime) * 0.0266667;
#endif

	if ( ++Counter100msec >= MVP_100MSEC_PERIOD )
	{
		b100msecTick = TRUE;
		Counter100msec = 0;
	}

	if ( ++Counter1sec >= MVP_1SEC_PERIOD )
	{
		bOneSecondTick = TRUE;
		Counter1sec = 0;
	}

	for ( i = 0; i < MVP_LINE_NUM; i++ )
	{
		INVERTER.MEASURE[i].fCurrent = RMS_GetValue(pRMSConI, i);
		INVERTER.MEASURE[i].fPhaseVoltage = RMS_GetValue(pRMSConE, i);

		//13.8.26
		//-GRID_ARG.MEASURE[i].fCurrent = INVERTER.MEASURE[i].fCurrent * INVERTER.fTRRatio;
		GRID_ARG.MEASURE[i].fCurrent = RMS_GetValue(pRMSGridI, i);
		//---------------------------------------------------------------------------------
		//GRID_ARG.MEASURE[i].fPeak = divsp(1.414, GRID_ARG.MEASURE[i].fCurrent) * 100;
		GRID_ARG.MEASURE[i].fPhaseVoltage = RMS_GetValue(pRMSGridV, i);
		// 역률 1이 보장되므로...
		GRID_ARG.MEASURE[i].fPower = GRID_ARG.MEASURE[i].fPhaseVoltage * GRID_ARG.MEASURE[i].fCurrent;

		GRID_ARG.fVoltPositive[i] = RMS_GetValue(pRMSGridEp, i);

		GRID_ARG.MEASURE_BYP[i].fPhaseVoltage = RMS_GetValue(pRMSBypassV, i);
	}

	if (CTRL.INV.filter_mode == FILTER_MODE_GC_SwF_1)
	{
		// Current Unbalance check
		Ta = INVERTER.MEASURE[0].fCurrent;
		Tb = INVERTER.MEASURE[1].fCurrent;
		Tc = INVERTER.MEASURE[2].fCurrent;
		if (Ta > Tb)	{ a = Ta;	b = Tb;	}
		else	        { a = Tb;	b = Ta;	}
		if (Tc > a)	a = Tc;
		if (Tc < b)	b = Tc;

	//	c= INVERTER.RATE.fCurrent * (float)PRM_PCS[CTRL_CURR_UNBAL_LIMIT].iValue*0.01;
		c= ACP.INV.RATE.Iph * (float)PRM_PCS[CTRL_CURR_UNBAL_LIMIT].iValue*0.01;
		if ( (a-b) > c )
		{
			if ( ++uUnbalanceCnt>1 )
				FLT_Raise(FLTH_INV_CURR_UNBAL);
		}
		else
			uUnbalanceCnt = 0;
	}
	/////////////////////////////////////////////////////

	//GRID_ARG.fTotalPower = GRID_ARG.MEASURE[0].fPower + GRID_ARG.MEASURE[1].fPower + GRID_ARG.MEASURE[2].fPower;
	GRID_ARG.fTotalPower = INVCTRL.fltPout2nd.fOut;

	if ( ++uInit5SecondCnt >= (5 * MVP_1SEC_PERIOD) )
	{
		 bAfterInit5Second = TRUE;
		 uInit5SecondCnt = 5 * MVP_1SEC_PERIOD;
	}

	if (bAfterInit5Second)
	{
		// 150916 전기연구원 OF 발생 시 차단 시간이 160ms로 기준 값에 걸림.
		// 10ms 마다 검출 하도록 옮김.
		//- SYS_VoltFreqMonitor();

		MVP_PowerAccumulate();
	}

	SYS_VoltFreqFaultClear();
	SYS_BattVoltFaultClear();
	SYS_BattUnderVoltFaultClear(); //13.3.3

#if MVP_EXECUTION_PERIOD
	MVP_endTime = CLK_gethtime();
#endif
}

void  MVP_ClearOneSencondTick(void)
{
	bOneSecondTick = FALSE;
}

Bool  MVP_GetOneSencondTick(void)
{
	return bOneSecondTick;
}

void  MVP_Clear100msecTick(void)
{
	b100msecTick = FALSE;
}

Bool  MVP_Get100msecTick(void)
{
	return b100msecTick;
}

void MVP_AddTrace()
{
#if MVP_EXECUTION_PERIOD
	TRC_AddNode( TRC_95_MVP_PERIOD_TIME, TRC_INT, &MVP_uExecutionPeriod);
#endif
}


ACC_POWER accPower;

void MVP_PowerAccumulate_mapping()
{

}


void MVP_PowerAccumulate()
{

}

