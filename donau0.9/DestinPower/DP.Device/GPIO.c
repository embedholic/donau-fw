/*
 * GPIO.c
 *
 *  Created on: 2014. 3. 5.
 *      Author: destinPower
 */

#include "GPIO.h"
#include "DSP28x_Project.h"
#include "Event.h"
#include "Fault.h"
#include "CTRL_GEN.h"
#include "EPWM.h"
#include "prm_pcs.h"
#include "CC.h"

public void GPIO_StaticSwitch(SSW_Status OnOff)
{
	if( PRM_PCS[BYP_MODE].iValue != 20 )
	{
		return;
	}

#if 0 //by JCNET
	if( PRM_PCS[SYS_DPWM_OFFTEMP].iValue == 1 )
	{
		MC_UpdateStatus(CTRL_SSEFAN_CTRL, OnOff);
	}
#endif

#if DBUG_MODE == 2
	if( OnOff == GPIO_STATIC_SW_ON )
		GenBlock.bypSw = 1;
	else
		GenBlock.bypSw = 0;
#endif
	if((GpioDataRegs.GPBDAT.bit.GPIO63 == GPIO_STATIC_SW_OFF) && (OnOff == GPIO_STATIC_SW_ON))
		EVT_Store(EVT_SSW_ON);
	if((GpioDataRegs.GPBDAT.bit.GPIO63 == GPIO_STATIC_SW_ON) && (OnOff == GPIO_STATIC_SW_OFF))
		EVT_Store(EVT_SSW_OFF);

	// 새로운 SSW 제어 방식이 아닐 경우에만 사용
	GpioDataRegs.GPBDAT.bit.GPIO63 = (Bool)OnOff;

	if( OnOff == GPIO_STATIC_SW_ON )
		EPWM_SSW_ON();
	else
		EPWM_SSW_OFF();
}

public void GPIO_StaticSwitchOn()
{
	if( PRM_PCS[BYP_MODE].iValue != 20 )
	{
		return;
	}

#if DBUG_MODE == 2
	GenBlock.bypSw = 1;
#else
	if(GpioDataRegs.GPBDAT.bit.GPIO63 == GPIO_STATIC_SW_OFF)
		EVT_Store(EVT_SSW_ON);

	GpioDataRegs.GPBDAT.bit.GPIO63 = GPIO_STATIC_SW_ON;

	// 새로운 SSW 제어 방식이 아닐 경우에만 사용
	EPWM_SSW_ON();
#endif
}

public void GPIO_StaticSwitchOff()
{
	// 160329 -  BYP 스위치 사용하는 모드에서만 제어 되도록 수정
	if( PRM_PCS[BYP_MODE].iValue != 20 )
	{
		return;
	}

#if DBUG_MODE == 2
	GenBlock.bypSw = 0;
#else
	if(GpioDataRegs.GPBDAT.bit.GPIO63 == GPIO_STATIC_SW_ON)
		EVT_Store(EVT_SSW_OFF);

	GpioDataRegs.GPBDAT.bit.GPIO63 = GPIO_STATIC_SW_OFF;

	// 새로운 SSW 제어 방식이 아닐 경우에만 사용
	EPWM_SSW_OFF();
#endif
}

public Bool GPIO_GetStaticSwitchOn()
{
#if DBUG_MODE == 2
	return GenBlock.bypSw;
#else
	return GpioDataRegs.GPBDAT.bit.GPIO63;
#endif
}

public SSW_Status GPIO_GetStaticSwitch()
{
#if DBUG_MODE == 2
	if( GenBlock.bypSw )
		return GPIO_STATIC_SW_ON;
	else
		return GPIO_STATIC_SW_OFF;
#else
	if( GpioDataRegs.GPBDAT.bit.GPIO63 == GPIO_STATIC_SW_ON )
		return GPIO_STATIC_SW_ON;
	else
		return GPIO_STATIC_SW_OFF;
#endif
}

public Bool GPIO_Status_StaticSwitch()
{
	if( (GpioDataRegs.GPBDAT.bit.GPIO63 == GPIO_STATIC_SW_ON)
			&& (ACP.BYP.v_pk * 0.9 > ACP.PCC.v_pk) )
	{
		return TRUE;
	}
	else
	{
		EVT_Store_NoDup(SCR_FAIL);
		return FALSE;
	}
}

/*
 * Call: c_int25. 5ms 마다 호출 됨.
 */
public void GPIO_UpdateStatus()
{
	static int now = 0;
	static int prev = 1;

	now = GPIO_GetStaticSwitchOn();

	if( now != prev )
	{
		if( now )
			EVT_Store_NoDup(EVT_SSW_ON);
		else
			EVT_Store(EVT_SSW_OFF);
	}

	prev = GPIO_GetStaticSwitchOn();
}
