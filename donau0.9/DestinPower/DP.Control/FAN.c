/*
 * FAN.c
 *
 *  Created on: 2017. 7. 17.
 *      Author: destinPower
 */

#include "FAN.h"
#include "CAN_GBI.h"
#include "CC.h"
#include "MCCB.h"

#define FAN_TEST 0
#if FAN_TEST == 1
int FAN_TEST_PebbTemp = 0;
int FAN_TEST_fanSpeed = 0;
#endif
unsigned int iPebbTempMax = 0;
int iPebbFan_Cmd = 0;
void FAN_Control_heatsink()
{
#if FAN_AUTO_CTRL == 1
	UInt16 i = 0;
	//UInt16 uiLoadValue = 0;
	float fCurve = 0;
	float fFanSpeed = 0;
	iPebbTempMax = 0;

	if( PRM_PCS[PARAM_TEMP_MIN].iValue < 0 || PRM_PCS[PARAM_TEMP_MIN].iValue > 70 )
		PRM_PCS[PARAM_TEMP_MIN].iValue = 40;

	if( PRM_PCS[PARAM_TEMP_MAX].iValue < 0 || PRM_PCS[PARAM_TEMP_MAX].iValue > 90 )
		PRM_PCS[PARAM_TEMP_MAX].iValue = 55;

	if( PRM_PCS[PARAM_FAN_START_LOAD].iValue < 0 || PRM_PCS[PARAM_FAN_START_LOAD].iValue > 60 )
		PRM_PCS[PARAM_FAN_START_LOAD].iValue = 40;

	for(i = 0; i < PEBB_COUNT_MAX; i++ )
	{
		if( iPebbTempMax < g_gbiRxData[i].rxM4.tempHeatSync)
			iPebbTempMax = g_gbiRxData[i].rxM4.tempHeatSync;
	}
#if FAN_TEST == 1
	iPebbTempMax = FAN_TEST_PebbTemp;
#endif

	//XXX 임시 테스트 코드
	if( PRM_PCS[DGT_PWR_CTRL_MODE].iValue == 0x1A || PRM_PCS[DGT_PWR_CTRL_MODE].iValue == 0x2A )
		iPebbTempMax = PRM_PCS[CTRL_COSPHI_P_5].iValue;

	if( iPebbTempMax < PRM_PCS[PARAM_TEMP_MIN].iValue )
		fFanSpeed = 0;
	else if( iPebbTempMax >= PRM_PCS[PARAM_TEMP_MAX].iValue)
		fFanSpeed = 100;
	else
	{
		fFanSpeed = ( (float)iPebbTempMax - PRM_PCS[PARAM_TEMP_MIN].iValue ) / (PRM_PCS[PARAM_TEMP_MAX].iValue - PRM_PCS[PARAM_TEMP_MIN].iValue) * 100.;
	}

	if(fFanSpeed<=0)
		fCurve = 0;
	else
		fCurve = (fFanSpeed * ((100- PRM_PCS[PARAM_FAN_START_LOAD].iValue)/100.)) + PRM_PCS[PARAM_FAN_START_LOAD].iValue; // FAN 실 동작: 40%~100%

	fFanSpeed = fCurve + 0.5f;

#if FAN_CTRL_VIA_LOAD  == 1
	// Heatsink 온도 검출 고장 대 Load에 따른 Fan 제어
	// Heaysink 온도 검출이 안될 경우 40도 이상으로 올라가지 않는 다고 가정.
	if( iPebbTempMax < PRM_PCS[PARAM_TEMP_MIN].iValue )
	{
		setLoadAverage( CC_GetCurrentLoadI() );
		if( getLoadAverage() < 35 )
		{
			// 35% 이하 -> 팬 Slow
			uiLoadValue = PRM_PCS[PARAM_FAN_START_LOAD].iValue;
			//	CAN_GBI_FanSpeed(0);
		}
		else
		{
			// 35%~55% 부하 -> 팬 Slow~Full
			uiLoadValue = (getLoadAverage() - 35) * 3 + 40;
			if( uiLoadValue < 40 )
				uiLoadValue = PRM_PCS[PARAM_FAN_START_LOAD].iValue;
			if( uiLoadValue > 100)
				uiLoadValue = 100;
			//CAN_GBI_FanSpeed(uiLoadValue);
		}

		//171115 35%부하 부터 팬 기동.
		if( fFanSpeed < uiLoadValue )
			fFanSpeed = uiLoadValue;
	}
#endif
		CAN_GBI_FanSpeed((int)fFanSpeed);
#endif
}



