/*
 * Event.h
 *
 *  Created on: 2012. 11. 8.
 *      Author: destinPower
 */

#ifndef EVENT_H_
#define EVENT_H_


#include "LGS_Common.h"

#define		MAX_HISTORY_STAT	15

enum
{
	EVENT_TYPE_SYSTEM_STATE,
	EVENT_TYPE_OPERATION,
	EVENT_TYPE_FAULT,
	EVENT_TYPE_WARNING,
	EVENT_TYPE_COMMAND
};



void EVT_Create( void );
void EVT_Store(Uns EventID);
void EVT_Store_NoDupReset();
void EVT_Store_NoDup(Uns EventID);
String EVT_GetCacheItem(Uns Nth);
void EVT_GetCacheItem_Modbus(Uns Nth, Uint16* pBuf);
void EVT_ClearCache(void);
int EVT_GetEventCode(unsigned int uDevice);


#endif /* EVENT_H_ */
