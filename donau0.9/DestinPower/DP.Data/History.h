/*
 * History.h
 *
 *  Created on: 2012. 12. 3.
 *      Author: destinPower
 */

#ifndef HISTORY_H_
#define HISTORY_H_

#include "LGS_Common.h"

#define		MAX_HISTORY_STAT	15

typedef struct _HISTORY_ITEM
{
Uns index;
char* item;
} HISTORY_ITEM;

typedef struct _HISTORY_STORE
{
	Uns		hisId;
	TimePack	timePackValue;
	Uns		msUnitForHistory;
} HISTORY_STORE;
/**************************************************************************/
/* EVT Methods                                                            */
/**************************************************************************/
void HISTORY_create( void );
void HISTORY_store(HISTORY_STORE hisStore);
public void HISTORY_WriteToNvSRAM();
void HISTORY_getCacheItem(Uns index, char * item);
void HISTORY_getCacheItem_Modbus(Uns index, Uint16 * pBuf);
void HISTORY_clearCache(void);
int HISTORY_GetEventID(unsigned int idx);


#endif /* HISTORY_H_ */
