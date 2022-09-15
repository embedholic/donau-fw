/*
 * MO.h
 *
 *  Created on: 2013. 10. 2.
 *      Author: destinPower
 */

#ifndef MO_H_
#define MO_H_

typedef struct _MO_Manage
{
	float fPebbOnLoad[PEBB_COUNT_MAX];
	float fPebbOffLoad[PEBB_COUNT_MAX];
	unsigned int uMO_Off_Wait_Delay[PEBB_COUNT_MAX];
}MO_BI; // 행위 정보

typedef enum
{
	PEBB_ALL_OFF,
	PEBB0_ON, // PEBB 1
	PEBB1_ON, // PEBB 2
	PEBB2_ON, // PEBB 3
	PEBB3_ON, // PEBB 4
	PEBB4_ON, // PEBB 5
	PEBB5_ON, // PEBB 6
	PEBB6_ON, // PEBB 7
	PEBB7_ON,  // PEBB 8
	PEBB_ALL_ON
} MO_STATE_LIST;

void MO_Create();
void MO_SetPebbVarient();
void MO_Run();
Bool MO_AllPebbON();
void MO_SetState(MO_STATE_LIST l);
int MO_GetStatus(int PebbId);
Bool MO_FaultResetCheck();
void MO_PebbFaultReset();
float MO_SetPebbOnLoad(int pebbId, float val);
float MO_SetPebbOffLoad(int pebbId, float val);
int MO_SetOnOffWaitTime_ms(int val);
void MO_PebbCtrl(int PebbId, int OnOff);
int MO_PebbCtrlCmdSend(int PebbId);
int MO_GetStatus(int PebbId);
int MO_GetPebbOnCount();


extern MO_BI MO;
extern MO_STATE_LIST MO_state;
#endif /* MO_H_ */
