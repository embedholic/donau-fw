/*
 * History.c
 *
 *  Created on: 2012. 12. 3.
 *      Author: destinPower
 */

#include "history.h"
#include "Util.h"
#include "NvSRAM.h"
#include "Queue.h"

/**************************************************************************/
/* Structure definition for History                                       */
/**************************************************************************/
typedef struct _History
{
	Uns	uIDLSB;
	Uns	uIDMSB;
	Uns uTime0; /* LSB */
	Uns uTime1;
	Uns uTime2;
	Uns uTime3; /* MSB */
} HistoryCache;

typedef struct _HistoryInit
{
	Uns	uAALSB;
	Uns	uAAMSB;
	Uns u55LSB;
	Uns u55MSB;
} HistoryInit;

typedef struct _HistoryHead
{
	Uns	uLSB;
	Uns	uMSB;
} HistoryHead;

#define HISTORY_CACHE_ITEM_SIZE 6 // ������ ��� NvSRAM NvSRAM_StoreEvent()�κе� ���� �Ǿ�� ��.
#define HISTORY_CACHE_SIZE		1024 * HISTORY_CACHE_ITEM_SIZE // 1024 EA * 6 BYTE

static Bool bCacheFull = FALSE;
static UInt16 uCacheHead = 0;

static void HISTORY_initHead(void)
{
	NvSRAM_SetEventHead(0,0);
	NvSRAM_SetCacheFull(FALSE);

	uCacheHead = 0;
	bCacheFull = FALSE;
}

/*******************************************************************/
/*FUNCTION:													       */
/*PARAMETERS:													   */
/*RETURN:		                                                   */
/*NOTE:				                                               */
/*******************************************************************/
void HISTORY_create( void )
{
	// �ʱ�ȭ �ĺ��ڿ� AAAA5555�� �������� �ʴٸ� �ʱ�ȭ�� �ʿ�
	if(!NvSRAM_CheckEventInit())
	{
		NvSRAM_SetEventInit();
		HISTORY_initHead();
	}
	else
	{
		uCacheHead = NvSRAM_GetEventHead();
		if( uCacheHead >=HISTORY_CACHE_SIZE)
		{
			HISTORY_initHead();
		}
		bCacheFull = NvSRAM_isCacheFull();
	}
}

/*******************************************************************/
/*FUNCTION:													       */
/*PARAMETERS:													   */
/*RETURN:		                                                   */
/*NOTE:				                                               */
/*******************************************************************/
void HISTORY_store(HISTORY_STORE hisStore)
{
#if 0 // XXX SWI���� DeadLock �������� ���� Queue �� ���� �� Idle Thread���� ó���ϴ� ������ ���� ��.
	TimePack TimeP;
	TimeP = hisStore.timePackValue;
	hisStore.hisId+=hisStore.msUnitForHistory;
	// LSB, MSB
	NvSRAM_StoreEvent(&uCacheHead, hisStore.hisId & 0xFF,(hisStore.hisId>>8)& 0xFF,TimeP.ByteVal.By0,TimeP.ByteVal.By1,TimeP.ByteVal.By2,TimeP.ByteVal.By3);
	if ( uCacheHead >= HISTORY_CACHE_SIZE )
	{
		uCacheHead = 0;
		bCacheFull = TRUE;
		NvSRAM_SetCacheFull(TRUE);
	}
	NvSRAM_SetEventHead(uCacheHead & 0xFF,uCacheHead>>8 );
#endif
	QUEUE_enqueHistory(hisStore);
}

/*
 * SWI ���� ȣ�� ���� �ʵ��� �ؾ� ��.
 */
HISTORY_STORE his;
public void HISTORY_WriteToNvSRAM()
{
	if( QUEUE_dequeHistory(&his) )
	{
		TimePack TimeP;
		TimeP = his.timePackValue;
		his.hisId+=his.msUnitForHistory;
		// LSB, MSB
		NvSRAM_StoreEvent(&uCacheHead, his.hisId & 0xFF,(his.hisId>>8)& 0xFF,TimeP.ByteVal.By0,TimeP.ByteVal.By1,TimeP.ByteVal.By2,TimeP.ByteVal.By3);
		if ( uCacheHead >= HISTORY_CACHE_SIZE )
		{
			uCacheHead = 0;
			bCacheFull = TRUE;
			NvSRAM_SetCacheFull(TRUE);
		}
		NvSRAM_SetEventHead(uCacheHead & 0xFF,uCacheHead>>8 );
	}
}
/*******************************************************************/
/*FUNCTION:													       */
/*PARAMETERS:													   */
/*RETURN:		                                                   */
/*NOTE:				                                               */
/*******************************************************************/
void HISTORY_clearCache( void )
{
	//-IRQ_globalDisable();
	HISTORY_initHead();
	//-IRQ_globalEnable();
}
/*******************************************************************/
/*FUNCTION:													       */
/*PARAMETERS:													   */
/*RETURN:		                                                   */
/*NOTE:				                                               */
/*******************************************************************/
void HISTORY_getCacheItem(Uns index, char * item)
{
	int idx;
	int Delta;
	int iStartPos;
	int iEndPos;
	UInt16 HistoryID;
	Uint16 Time1,Time2,Time3,Time4;

	// ����! uCacheHead�� 0���� �����ϰ� Nth�� 1���� �����Ѵ�.
	//-Delta = uCacheHead - index;
	Delta = uCacheHead - (index*HISTORY_CACHE_ITEM_SIZE);

	// (n)�� Cache�� ũ�⺸�� ũ�ٸ� ������ �����ʹ� ����.
	// ���� n ��° ���� ��û���� Head - (n)�� ���� 0���� �۾�����
	// bCacheFull�� FALSE��� �� �̻��� �����ʹ� ����.
	if ( index > HISTORY_CACHE_SIZE || ( Delta < 0 && bCacheFull == FALSE ) )
	{
		item[0] = '0';
		for ( idx = 1; idx < MAX_HISTORY_STAT; idx++ )
			item[idx] = 0;
		return;
	}
	if ( Delta < 0 ) Delta += HISTORY_CACHE_SIZE;

	// n��° �̺�Ʈ ������ ã�����Ƿ� ���ڿ��� ��ȯ�Ѵ�.
	NvSRAM_GetEvent(Delta,&HistoryID,&Time1,&Time2,&Time3,&Time4);
	iStartPos = 0;
	utilLtoa( HistoryID, item, iStartPos, &iEndPos, TRUE );
	Time1 = Time1 & 0xFF; // LSB
	item[iEndPos + 7] = Uns2Hex( Time1&0xF );
	item[iEndPos + 6] = Uns2Hex( (Time1>>4)&0xF );
	Time2 = Time2 & 0xFF;
	item[iEndPos + 5] = Uns2Hex( Time2&0xF );
	item[iEndPos + 4] = Uns2Hex( (Time2>>4)&0xF );
	Time3 = Time3 & 0xFF;
	item[iEndPos + 3] = Uns2Hex( Time3&0xF );
	item[iEndPos + 2] = Uns2Hex( (Time3>>4)&0xF );
	Time4 = Time4 & 0xFF; // MSB
	item[iEndPos + 1] = Uns2Hex( Time4&0xF );
	item[iEndPos + 0] = Uns2Hex( (Time4>>4)&0xF );
	item[iEndPos + 8] = 0; // Null�� ����ǵ��� ����
}

void HISTORY_getCacheItem_Modbus(Uns index, Uint16 * pBuf)
{
	//-int idx;
	int Delta;
	//-int iStartPos;
	//-int iEndPos;
	UInt16 HistoryID;
	Uint16 Time1,Time2,Time3,Time4;

	// ����! uCacheHead�� 0���� �����ϰ� Nth�� 1���� �����Ѵ�.
	//-Delta = uCacheHead - index;
	Delta = uCacheHead - (index*HISTORY_CACHE_ITEM_SIZE);

	// (n)�� Cache�� ũ�⺸�� ũ�ٸ� ������ �����ʹ� ����.
	// ���� n ��° ���� ��û���� Head - (n)�� ���� 0���� �۾�����
	// bCacheFull�� FALSE��� �� �̻��� �����ʹ� ����.
	if ( index > HISTORY_CACHE_SIZE || ( Delta < 0 && bCacheFull == FALSE ) )
	{
		pBuf[0] = pBuf[1] = pBuf[2] = 0;
		return;
	}
	if ( Delta < 0 ) Delta += HISTORY_CACHE_SIZE;


	NvSRAM_GetEvent(Delta,&HistoryID,&Time1,&Time2,&Time3,&Time4);

	pBuf[0] = HistoryID;
	pBuf[1] = Time1 << 8; // LSB
	pBuf[1] |= Time2 & 0xFF;
	pBuf[2] = Time3 << 8;
	pBuf[2] |= Time4 & 0xFF; // MSB
}
int HISTORY_GetEventID(unsigned int idx)
{
	int Delta;

	// ����! uCacheHead�� 0���� �����ϰ� Nth�� 1���� �����Ѵ�.
	Delta = uCacheHead - (idx*HISTORY_CACHE_ITEM_SIZE);
	if ( Delta < 0 ) Delta += HISTORY_CACHE_SIZE;

	//-temp = ((pCACHE[Delta].uIDMSB & 0xFF)<<8) + (pCACHE[Delta].uIDLSB & 0xFF);
	return NvSRAM_GetEventId(Delta);
}



