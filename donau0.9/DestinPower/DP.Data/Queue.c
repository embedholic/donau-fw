/*
 * Queue.c
 *
 *  Created on: 2013. 01. 22.
 *      Author: destinPower
 */
#include "Queue.h"
//-#include <ti/sysbios/knl/Queue.h>
/*
 * NvSRAM의 write/read에서 SWI와 task간 DeadLock 현상을 임시 해결하기 위해 Event발생 시 Queue에 값을 삽입하고,
 * 데이터 송/수신 Task에서 Queue의 데이터를 넣도록 하였다.
 * write시 GPIO CS를 Task에서 잡은 상태에서 SWI가 발생하고 Task가 잡은 GPIO CS를 SWI가 대기하면서 Task의 작업이
 * 끝나지 못하여 발생하게 된다. 버퍼 사이즈는 수정 가능하다.
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

#if 0 // 하단과 같은 매크로는 위험. 매개 변수 대신 전역 변수가 참조 될 수 있음.
	  // 140210 매크로 사용하지 않도록 수정. 히스토리 저장 버그 수정. ( warnning이 있는 상태에서 Fault Reset 할 경우. 테스트 가능. )
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

	// fixed 스택 가리키도록 queue에 넣음으로 스택 왜곡이 발생하여, 히스토리 데이터 저장에 교란 발생
	// 낮은 복사를 수행하도록 수정 하여 정상 처리됨 확임.
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
 * BIOS의 Queue는 BIOS_START()가 된 이후에 사용 가능 한 것 같다.
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

