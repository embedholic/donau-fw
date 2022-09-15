/*
 * MO.c
 *
 * ��� ����.
 *
 *  Created on: 2013. 10. 2.
 *      Author: destinPower
 *
 *
 *      OFF���� ä�͸��� ���� check time T�� MCU�� state���� üũ�Ѵ�.
 *      ��, eGBI�� ON/OFF�� ����� �޾� ���� �ϵ�, ON�� gate on - delay - mc on OFF�� mc off - delay - gate off �� ó���Ѵ�.
 *
 *      ������ MO_SetMcOnWaitTime_ms ���� �͵��� ������ can tx ������ �Ǿ �ֱ������� �Ǵ� ��� ���� ó���Ѹ�
 *      gate, mc �� on/off�� manual op ����� ��쿡�� ����ǵ��� eGBI�� �����ϰ�, mcu������ �Լ��� MANUAL �� ���δ�.
 *
 */

#include "LGS_Common.h"
#include "MO.h"
#include "CAN_GBI.h"
#include "PRM_PCS.h"
#include <string.h>
#include "CC.h"
#include "Fault.h"

#define PEBB_CAPACITY (125.)

MO_STATE_LIST MO_state = PEBB_ALL_OFF;
MO_STATE_LIST beforeState = PEBB_ALL_OFF;
MO_BI MO;

void MO_Processing();

void MO_Create()
{
	int i = 0;
	memset(&MO, 0, sizeof(MO));

	// ù��° ���� ���۰� ���ÿ� ON
	MO.fPebbOnLoad[0] = 0;
	MO.fPebbOffLoad[0] = 0;

	// PEBB 2�� �������� 8������ ���� XXX
	for( i=0; i<PEBB_COUNT_MAX-1; i++)
	{
		MO.fPebbOffLoad[i+1] = PARAM_VAL(SYS_SLAVE_PEBB2_OFF_LOAD_OFFSET+(i*2));
		MO.fPebbOnLoad[i+1]  = PARAM_VAL(SYS_SLAVE_PEBB2_ON_LOAD_OFFSET+(i*2));
	}

	beforeState = PEBB_ALL_OFF;
	MO_state = PEBB_ALL_OFF;

	MO_SetOnOffWaitTime_ms(PRM_PCS[SYS_PEBB_ONOFF_WAIT_TIME].iValue);
}


void MO_SetState(MO_STATE_LIST l)
{
	MO_state = l;
	int i;
	int OnOffCount;
	i = 0;
	OnOffCount = 0;

	if( beforeState != MO_state)
	{
		switch(MO_state)
		{
		case PEBB_ALL_OFF:
		case PEBB_ALL_ON:
			break;
		case PEBB0_ON: 				OnOffCount = 1;
			break;
		case PEBB1_ON:				OnOffCount = 2;
			break;
		case PEBB2_ON:				OnOffCount = 3;
			break;
		case PEBB3_ON:				OnOffCount = 4;
			break;
		case PEBB4_ON:				OnOffCount = 5;
			break;
		case PEBB5_ON:				OnOffCount = 6;
			break;
		case PEBB6_ON:				OnOffCount = 7;
			break;
		case PEBB7_ON:				OnOffCount = 8;
			break;

		default:
			break;
		}

		if( MO_state == PEBB_ALL_OFF)
		{
			for(i=0;i<PEBB_COUNT_MAX;i++)
				MO_PebbCtrl(i,0);
		}
		else if ( MO_state == PEBB_ALL_ON )
		{
			for(i=0;i<PEBB_COUNT_MAX;i++)
					MO_PebbCtrl(i,1);
		}
		else
		{
			for(i=0;i<PEBB_COUNT_MAX;i++)
			{
				if ( i < OnOffCount)
					MO_PebbCtrl(i,1);
				else
					MO_PebbCtrl(i,0);
			}
		}

		beforeState = MO_state;
	}
}

Bool MO_AllPebbON()
{
	if( MO_GetPebbOnCount() == PEBB_COUNT_MAX )
		return TRUE;
	else
		return FALSE;
}
Bool MO_FaultResetCheck()
{
	int i = 0;

	for( i = 0; i < PEBB_COUNT_MAX; i++)
	{
		if( g_gbiTxData[i].C0.C0_0U.BIT.FAULT_RESET  )
		{
			return TRUE;
		}
	}

	return FALSE;
}
void MO_PebbFaultReset()
{
	int i = 0;

	for( i = 0; i < PEBB_COUNT_MAX; i++)
	{
		g_gbiTxData[i].C0.C0_0U.BIT.FAULT_RESET = 1;
		g_gbiTxData[i].bTxNew = 1;
	}
}

float MO_SetPebbOnLoad(int pebbId, float val)
{
	// 100 / (PEBB_MAX)

	if( val > 100 )
		val = 100;
	if( val < 0)
		val = 0;

	// ���� ������ ���� ������.
	//-if( val <= MO.fPebbOffLoad[pebbId] )
	//-	val = MO.fPebbOffLoad[pebbId] + 2;

	return  MO.fPebbOnLoad[pebbId] = val;
}

float MO_SetPebbOffLoad(int pebbId, float val)
{
	if( val > 100 )
		val = 100;
	if( val < 0)
		val = 0;

	//-if( val >= MO.fPebbOnLoad[pebbId])
	//-	val = MO.fPebbOnLoad[pebbId] - 2;

	return  MO.fPebbOffLoad[pebbId] = val;
}

/*
 * Gate ON �� MC�� ���̱� �� ��� �ð�.
 * Gate OFF �� MC�� OFF �ϱ��� ��� �ð�.
 */
int MO_SetOnOffWaitTime_ms(int val)
{
	int i;

	if( val < 5)
		val = 5;

	for(i = 0; i < PEBB_COUNT_MAX; i++)
	{
		g_gbiTxData[i].C0.MoCtrlDelay = val;
		g_gbiTxData[i].bTxNew = 1;
	}


	return val;
}

void MO_PebbCtrl(int PebbId, int OnOff)
{
	if( PebbId >= PEBB_COUNT_MAX )
		return;

	g_gbiTxData[PebbId].C0.C0_0U.BIT.MO_STATE = OnOff;
	g_gbiTxData[PebbId].bTxNew = 1;

#if DBUG_MODE == 2 || DBUG_MODE == 3
	 g_gbiRxData[PebbId].rxM0.STATE.BIT.MASTER_GATE = OnOff;
	 g_gbiRxData[PebbId].rxM0.STATE.BIT.MC_STATE = OnOff;
#endif
}


int MO_PebbCtrlCmdSend(int PebbId)
{
	return CAN_GBI_SendCmd(&(g_gbiTxData[PebbId]),M_BOX1,PebbId);
}

int MO_GetStatus(int PebbId)
{
	if( PebbId >= PEBB_COUNT_MAX )
		return 0;

#if USE_EGBI == 0
#if DEBUG_MODE == DEBUG_SIMULSATION // FIXME DELETE
		return 1;
#endif
#endif

	if( g_gbiRxData[PebbId].rxM0.STATE.BIT.MASTER_GATE == 1 && g_gbiRxData[PebbId].rxM0.STATE.BIT.MC_STATE == 1 )
		return 1;

	return 0;
}

float MO_GetCurrentLmt()
{
	float a;
	a = MO_GetPebbOnCount();

	return 100. / (PARAM_VAL(INV_CAPACITY) / ( a * PEBB_CAPACITY ));
}

int MO_GetMaxHeatSinkTemp()
{
	int i = 0;
	int iMax =  0;

	for(i =0; i<PEBB_COUNT_MAX; i++)
	{
		if( g_gbiRxData[i].rxM0.STATE.BIT.MASTER_GATE == 1 && g_gbiRxData[i].rxM0.STATE.BIT.MC_STATE == 1 )
		{
			if( g_gbiRxData[i].rxM4.tempHeatSync > iMax )
			{
				iMax = g_gbiRxData[i].rxM4.tempHeatSync;
			}
		}
	}

	return iMax;
}
int MO_GetPebbOnCount()
{
	int i = 0;
	int count =  0;
	for(i =0; i<PEBB_COUNT_MAX; i++)
	{
		if( g_gbiRxData[i].rxM0.STATE.BIT.MASTER_GATE == 1 && g_gbiRxData[i].rxM0.STATE.BIT.MC_STATE == 1 )
			count++;
	}

#if USE_EGBI == 0
#if DBUG_MODE == 2 || DBUG_MODE == 3
	switch(MO_state)
		{
		case PEBB_ALL_OFF:
			return 0;
		case PEBB0_ON:
			return 1;
		case PEBB1_ON:
			return 2;
		case PEBB2_ON:
			return 3;
		case PEBB3_ON:
			return 4;
		case PEBB4_ON:
			return 5;
		case PEBB5_ON:
			return 6;
		case PEBB6_ON:
			return 7;
		case PEBB7_ON:
			return 8;
		case PEBB_ALL_ON:
			return PEBB_COUNT_MAX;
		default:
			return 0;
		}
#endif
#endif

	return count;
}

void MO_Run()
{
	//float fTemp;
	//fTemp = 0;


	//fTemp = CC_GetCurrentLoadI();

	// Nothing.
	return;
}


void MO_Processing()
{
	int iPebbNumber;
	float fCurrentLoad;
	fCurrentLoad = CC_GetCurrentLoadI();

	for(iPebbNumber = PEBB_COUNT_MAX-1; iPebbNumber > 0 ; iPebbNumber-- )
	{
		if( fCurrentLoad >= MO.fPebbOnLoad[iPebbNumber] && beforeState < PEBB0_ON + iPebbNumber ) // On
		{
			MO_SetState((MO_STATE_LIST)(PEBB0_ON + iPebbNumber));
		}
		else if( fCurrentLoad <= MO.fPebbOffLoad[iPebbNumber] && beforeState >= PEBB0_ON + iPebbNumber ) // Off
		{
			// OFF Load Check & 5 �� ����...
			if( MO.uMO_Off_Wait_Delay[iPebbNumber] >= 5 )
			{
				MO.uMO_Off_Wait_Delay[iPebbNumber] = 5;
				MO_SetState((MO_STATE_LIST)(PEBB0_ON + iPebbNumber - 1));
			}
		}
		else
		{
			// ������ ��Ȳ������ �׻� �̰����� ��.
			MO.uMO_Off_Wait_Delay[iPebbNumber] = 0;
		}
	}
}
