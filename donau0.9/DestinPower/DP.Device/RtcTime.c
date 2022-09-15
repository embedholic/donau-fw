/*
 * Time.c
 *
 *  Created on: 2012. 11. 12.
 *      Author: destinPower
 */


//-#include "parameter.h"
#include "prm_pcs.h"
#include "RtcTime.h"
#include "I2C.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

RtcTime TI_BUF;
RtcTime *TI = &TI_BUF;
TimePack TPack;
TimePack TPackShadow;

/*
 * 주의! I2C가 먼저 생성되어야 한다.
 */
public void RTCTIME_Create(void)
{
	TPack.UnsVal = 0;
	RTCTIME_Update();
}

public void RTCTIME_Update(void)
{
	//-DS1743HW_get(TI);
	if(I2C_GET_RTC_Time(TI) == FALSE )
	{
#if DEBUG_MODE == 1
		error();
#endif

//	#if DBUG_MODE == 2
//		System_printf("RTCTIME_Update FAIL.\n");
//		System_flush();
//	#endif
		return;
	}

	// TODO SET PARAM
	PRM_PCS[TIME_YEAR].iValue=TI->Year;
	PRM_PCS[TIME_MONTH].iValue=TI->Month;
	PRM_PCS[TIME_DAY].iValue=TI->Date;
	PRM_PCS[TIME_HOUR].iValue=TI->Hour;
	PRM_PCS[TIME_MINUTE].iValue=TI->Minute;
	PRM_PCS[TIME_SECOND].iValue=TI->Second;


	//Test 완료.
	TPack.UnsVal = 0L;	//+
	TPack.UnsVal |= (((Uint32)TI->Year-2000L)<<26) & 0x7C000000; // 5 bit
	TPack.UnsVal |= ((Uint32)TI->Month<<22) & 0x3C00000; // 4 bit
	TPack.UnsVal |= ((Uint32)TI->Date<<17) & 0x3E0000; // 5 bit
	TPack.UnsVal |= ((Uint32)TI->Hour<<12) & 0x1F000; // 5 bit
	TPack.UnsVal |= (TI->Minute<<6) & 0xFC0; // 6 bit
	TPack.UnsVal |= (TI->Second) & 0x3F; // 6 bit

	TPackShadow = TPack;
}

public Bool RTCTIME_Set(RtcTime *pSetVal)
{
	//-DS1743HW_set(pSetVal);
	return I2C_SET_RTC_Time(pSetVal);
}

public TimePack RTCTIME_GetPackValue( void )
{
	return TPackShadow;
}





