/*
 * Queue.c
 *
 *  Created on: 2013. 01. 22.
 *      Author: destinPower
 */
#include "Queue.h"
//-#include <ti/sysbios/knl/Queue.h>
/*
 * NvSRAM�� write/read���� SWI�� task�� DeadLock ������ �ӽ� �ذ��ϱ� ���� Event�߻� �� Queue�� ���� �����ϰ�,
 * ������ ��/���� Task���� Queue�� �����͸� �ֵ��� �Ͽ���.
 * write�� GPIO CS�� Task���� ���� ���¿��� SWI�� �߻��ϰ� Task�� ���� GPIO CS�� SWI�� ����ϸ鼭 Task�� �۾���
 * ������ ���Ͽ� �߻��ϰ� �ȴ�. ���� ������� ���� �����ϴ�.
 */

typedef struct CIRC_Obj {
    Uns         writeIndex;     /* write pointer for the buffer */
    Uns         readIndex;      /* read pointer fro the buffer */
    Uns         charCount;      /* buffer character count */
    Uns         size;
    HISTORY_STORE*       buf;      /* circular buffer */
} CIRCH_Obj, *CIRCH_Handle;

CIRCH_Obj circHistory;
HISTORY_STORE ARRAY_HIS_BUFFER[50];

#if 0 // �ϴܰ� ���� ��ũ�δ� ����. �Ű� ���� ��� ���� ������ ���� �� �� ����.
	  // 140210 ��ũ�� ������� �ʵ��� ����. �����丮 ���� ���� ����. ( warnning�� �ִ� ���¿��� Fault Reset �� ���. �׽�Ʈ ����. )
#define CIRCH_nextIndex(circHistory,index)      (((index) + 1) & ((circHistory).size - 1))
#define CIRCH_prevIndex(circHistory,index)      (((index) - 1) & ((circHistory).size - 1))
#endif

public void QUEUE_Create()
{
	circHistory.writeIndex = 0;
	circHistory.readIndex = 0;
	circHistory.charCount = 0;
	circHistory.size = 50;
	circHistory.buf=ARRAY_HIS_BUFFER;
}

public Bool QUEUE_enqueHistory(HISTORY_STORE hisData )
{
	//-Bool res=TRUE;
	if(circHistory.charCount==circHistory.size)
	{
#if DEBUG_MODE == DEBUG_DEVELOP
		//OverFlow
		error();
#endif
		return FALSE;
	}

	// fixed ���� ����Ű���� queue�� �������� ���� �ְ��� �߻��Ͽ�, �����丮 ������ ���忡 ���� �߻�
	// ���� ���縦 �����ϵ��� ���� �Ͽ� ���� ó���� Ȯ��.
    /* write character and decrement the character count */
	circHistory.buf[circHistory.writeIndex].hisId = hisData.hisId;
	circHistory.buf[circHistory.writeIndex].msUnitForHistory = hisData.msUnitForHistory;
	circHistory.buf[circHistory.writeIndex].timePackValue = hisData.timePackValue;

	if( circHistory.writeIndex + 1 > circHistory.size - 1)
	{
		circHistory.writeIndex = 0;
	}
	else
	{
		circHistory.writeIndex += 1;
	}
	//-circHistory.writeIndex = CIRCH_nextIndex(circHistory,circHistory.writeIndex);
	(circHistory.charCount)++;
	return TRUE;
}
public Bool QUEUE_dequeHistory(HISTORY_STORE* pHisData)
{
	if( circHistory.charCount <= 0)
	{
		return FALSE;
	}

	*pHisData = (circHistory.buf[circHistory.readIndex]);

	if( circHistory.readIndex + 1 > circHistory.size - 1)
	{
		circHistory.readIndex = 0;
	}
	else
	{
		circHistory.readIndex += 1;
	}

	//-circHistory.readIndex = CIRCH_nextIndex(circHistory,circHistory.readIndex);
	(circHistory.charCount)--;

	return TRUE;
}

#if 0
typedef struct Rec {
  Queue_Elem _elem;
  HISTORY_STORE data;
}HisQueue;
static Queue_Handle q;

/*
 * BIOS�� Queue�� BIOS_START()�� �� ���Ŀ� ��� ���� �� �� ����.
 */
public void QUEUE_Create()
{
	// create a Queue instance 'q'
	q = Queue_create(NULL, NULL);
#if DEBUG_MODE == DEBUG_DEVELOP
	if( !q )
		error();
#endif
}

public void QUEUE_enqueHistory(HISTORY_STORE hisData )
{
	HisQueue qData;
	qData.data = hisData;
	Queue_enqueue(q, &qData._elem);
}
public Bool QUEUE_dequeHistory(HISTORY_STORE* pHisData)
{
	HisQueue* qData;

	if(!Queue_empty(q))
	{
		qData = Queue_dequeue(q);
		*pHisData = qData->data;
		return TRUE;
	}

	return FALSE;
}
#endif

