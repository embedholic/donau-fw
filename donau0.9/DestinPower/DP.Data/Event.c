/*
 * Event.c
 *
 *  Created on: 2012. 11. 8.
 *      Author: destinPower
 */

#include "LGS_Common.h"
#include "Util.h"
#include "Event.h"
#include "History.h"
#include "Fault.h"
#include "RtcTime.h"

#define NO_DUP_CNT_MAX 3

typedef struct EVT_STRUCT_A
{
	int flag_table[NO_DUP_CNT_MAX];
	int idx;
}EVT_NODUP;
EVT_NODUP Nodup;



void EVT_Create( void )
{
	HISTORY_create();
	EVT_Store_NoDupReset();
}

void EVT_Store(Uns EventID)
{
	volatile HISTORY_STORE	hisStore;

	hisStore.hisId=EventID;
	hisStore.timePackValue=RTCTIME_GetPackValue();
	hisStore.msUnitForHistory=0;
	HISTORY_store(hisStore);

}

void EVT_Store_NoDupReset()
{
	int i;

	Nodup.idx = 0;

	for(i = 0; i < NO_DUP_CNT_MAX; i++)
	{
		Nodup.flag_table[i] = -1;
	}
}

void EVT_Store_NoDupInsert(Uns id)
{
	if( Nodup.idx + 1 >= NO_DUP_CNT_MAX )
		Nodup.idx = 0;

	Nodup.flag_table[Nodup.idx++] = id;
}

Bool ENT_Store_NoDupSearch(Uns id)
{
	int i;
	for(i = 0; i < NO_DUP_CNT_MAX; i++)
	{
		if( Nodup.flag_table[i] == id )
			return TRUE;
	}
	return FALSE;
}

/*
 * 160323 중복 저장 방지.
 */
void EVT_Store_NoDup(Uns EventID)
{
	if( ENT_Store_NoDupSearch(EventID) )
		return;

	EVT_Store_NoDupInsert(EventID);
	return EVT_Store(EventID);
}

void EVT_ClearCache( void )
{
	HISTORY_clearCache();
}

String EVT_GetCacheItem(Uns Nth)
{
	HISTORY_getCacheItem(Nth, FLT.cNthStatus);
	return FLT.cNthStatus;
}

void EVT_GetCacheItem_Modbus(Uns Nth, Uint16* pBuf)
{
	HISTORY_getCacheItem_Modbus(Nth, pBuf);
	return;
}

int EVT_GetEventCode(unsigned int uDevice)
{
	return -1;
}



