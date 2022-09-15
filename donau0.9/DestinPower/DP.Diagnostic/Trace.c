/*
 * Trace.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */


#include <string.h>
#include "trace.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "Util.h"
#include "SagEventHistory.h"


InfoNode INFO[TRC_NUM];

#pragma DATA_SECTION(TPOOL, "H45")
TracePoolNode TPOOL[TRACE_NUM];

#pragma DATA_SECTION(TRACE, "L47")
Trace TRACE;

#pragma DATA_SECTION(TraceListMax, "L47")
Uns TraceListMax = TRC_NUM;



/*
*********************************************************************************************************
*                                         Create TraceList
*
* Description: Initialize TraceList
*
* Arguments  : none
*
* Returns    : none
*********************************************************************************************************
*/
void TRC_Create( void )
{
	int i;

	memset(&TPOOL, 0, sizeof(TPOOL));
	memset(&TRACE, 0, sizeof(TRACE));

	for(i = 0; i < TRC_NUM; i++)
	{
		INFO[i].iItemDataType = TRC_UNS;
		INFO[i].pItemAddr = &TraceListMax;
	}

	TRC_CreateTracePool();

	TRC_TraceReset();
}

/*
*********************************************************************************************************
*                                         ADD Nodes to TraceList
*
* Description:
*
* Arguments  : ID               is :
*
*              InfoItemDataType is :
*
*              pItemAddr        is :
* Returns    :
*********************************************************************************************************
*/
Bool TRC_AddNode( InfoID ID, int InfoItemDataType, void *pItemAddr )
{
	if ( ID >= TRC_NUM ) return FALSE;
	INFO[ID].iItemDataType = InfoItemDataType;
	INFO[ID].pItemAddr = pItemAddr;
	INFO[ID].bInUsed=1;
	return TRUE;
}

//+june. before : Bool TRC_GetItemFloatValue( int ID, float *Val )
Bool TRC_GetItemFloatValue( InfoID ID, float *Val )
{
	int Type;
	if ( ID >= TRC_NUM ) return FALSE;
    Type = INFO[ID].iItemDataType;
	if(INFO[ID].bInUsed==1)
	{
		switch( Type )
		{
			case TRC_FLOAT:
				*Val = *(float *)(INFO[ID].pItemAddr);
				break;
			case TRC_INT:
				*Val = (float)(*(int *)(INFO[ID].pItemAddr));
				break;
			case TRC_UNS:
				*Val = (float)(*(Uns *)(INFO[ID].pItemAddr));
				break;
			default:
				*Val = 0.;
		}
		return (Bool)TRUE;
	}
	else
	{
		*Val=0;
		return (Bool)FALSE;
	}
}

Bool TRC_CreateTracePool( void )
{
	int i;

	TRACE.iMemorySize = TRACE_MEMORY_SIZE;
	TRACE.iSamplingTime = PRM_PCS[TRC_TRACE_SAMPLING].iValue;
	TRACE.iSamplingTimeCounter = 0;

	TRACE.fStopTimeRatio = 97.9;//2.1; //2.1% ( 20
	TRACE.iStop = OFF;
	TRACE.iStopTime = (TRACE.iMemorySize * TRACE.fStopTimeRatio) / 100;
	TRACE.iStopTimeCounter = 0;
	for( i=0;i<TRACE_LEVEL;i++ )
		TRACE.iTail[i] = -1;
	TRACE.iCurrentLevel = 0;
	TRACE.iNextLevel = 0;


	for( i = 0; i < TRACE_NUM; ++i )
	{
		TPOOL[i].nIndex = PRM_PCS[TRC_TRACE1 + i].iValue;
		//TPOOL[i].pItemAddr = INFO[TPOOL[i].nIndex].pItemAddr;
		/*
		if ( TPOOL[i].pItemAddr == 0x0 )
		{
			TPOOL[i].nIndex = TRC_214_CC_PERIOD;
			//TPOOL[i].pItemAddr = INFO[TRC_214_CC_PERIOD].pItemAddr;
		}
		TPOOL[i].pStartAddr = TRACE_BUF[i];
		*/
	}
	return TRUE;
}

//-TODO DATA REGION: #pragma CODE_SECTION(TRC_UpdateTraceChannel, ".iram")
void TRC_UpdateTraceChannel( void )
{
	int i;

	if (++TRACE.iSamplingTimeCounter < TRACE.iSamplingTime) return;
	else TRACE.iSamplingTimeCounter = 0;

 	if ( TRACE.iStop == ON ) ++TRACE.iStopTimeCounter;

	if (TRACE.iStopTimeCounter >= TRACE.iStopTime)
	{
		TRACE.iStopTimeCounter = TRACE.iStopTime;
		TRACE.iCurrentLevel = TRACE.iNextLevel;
		return;
	}

	if (++TRACE.iTail[TRACE.iCurrentLevel] >= TRACE.iMemorySize) TRACE.iTail[TRACE.iCurrentLevel] = 0;
	for(i = 0; i < TRACE_NUM; i++)
	{
//		*((float *)TPOOL[i].pStartAddr + TRACE.iTail) =  *(float *)(INFO[TPOOL[i].nIndex].pItemAddr) ;
		if(INFO[TPOOL[i].nIndex].bInUsed==0)
			*((float *)&TPOOL[i].TRACE_BUF[TRACE.iCurrentLevel][TRACE.iTail[TRACE.iCurrentLevel]])=0;
		else
			*((float *)&TPOOL[i].TRACE_BUF[TRACE.iCurrentLevel][TRACE.iTail[TRACE.iCurrentLevel]])=  *(float *)(INFO[TPOOL[i].nIndex].pItemAddr) ;
	}
}

void TRC_StopTrace( void )
{
	int i;

	if ( TRACE.iStop == ON )
		return;

	if( ++TRACE.iNextLevel >= TRACE_LEVEL)
	{
		TRACE.iNextLevel=0;
	}
	for(i = 0; i < TRACE_NUM; i++)
		PRM_PCS[TRC_TRACE1+36+i].iValue = PRM_PCS[TRC_TRACE1+24+i].iValue;
	for(i = 0; i < TRACE_NUM; i++)
		PRM_PCS[TRC_TRACE1+24+i].iValue = PRM_PCS[TRC_TRACE1+12+i].iValue;
	for(i = 0; i < TRACE_NUM; i++)
		PRM_PCS[TRC_TRACE1+12+i].iValue = PRM_PCS[TRC_TRACE1+i].iValue;

	TRACE.iStop = ON;

}

void TRC_StartTrace( void )
{
	if ( TRACE.iStop == OFF )
		return;
	TRACE.iSamplingTimeCounter = 0;
	TRACE.iStopTimeCounter = 0;
	TRACE.iTail[TRACE.iCurrentLevel] = -1;
	TRACE.iStop = OFF;
}

//extern int bTraceSaved;
Bool TRC_TraceReset( void )
{
	//bTraceSaved = FALSE;
	//Sag_data_ready_flag = FALSE;
	//Sag_data_update_flag = TRUE;

	/* Trace하는 도중에는 Trace Reset을 할 수 없다 */
	if ( TRACE.iStop == OFF ) return FALSE;

	//TRACE.iSamplingTimeCounter = 0;
	TRACE.iStopTimeCounter = 0;
	TRACE.iTail[TRACE.iCurrentLevel] = -1;
	TRACE.iStop = OFF;

	return TRUE;
}

void TRC_SetTraceStopTime( float fStopTime )
{
	TRACE.fStopTimeRatio = fStopTime;
	TRACE.iStop = OFF;
	TRACE.iStopTime = (TRACE.iMemorySize * TRACE.fStopTimeRatio) / 100;
	TRACE.iStopTimeCounter = 0;
	TRACE.iTail[TRACE.iCurrentLevel] = -1;
}

void TRC_ChangeTracePoolNode( int ChNum, int DstID )
{
	TPOOL[ChNum].nIndex = DstID;
}

void TRC_UpdateParameter( void )
{
	TRACE.iSamplingTime = PRM_PCS[TRC_TRACE_SAMPLING].iValue;
	TRACE.iSamplingTimeCounter = 0;
}

