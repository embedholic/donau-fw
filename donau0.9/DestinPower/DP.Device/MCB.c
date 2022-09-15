/*
 * MCB.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */


#include "odt.h"
#include "MCCB.h"
#include "MCB.h"
#include "fault.h"
#include "Event.h"
#include "GPIO.h"
#include "SYS_ESLV.h"
#include "SYSTEM.h"

typedef struct _M_CB
{
	MCB_STATUS 		State;
	MCB_STATUS 		PrevState;
	Odt 		*odtcbDelay;
	Odt 		ODT_CB_DELAY;
	Uns 		uHistoryIdPos;
	CBMC_CTRL 	idON_F;
	CBMC_CTRL 	idOFF_G;
	CBMC_CTRL 	idUVR;
	CBMC_STATUS idStatus;
	Uns idFault;
	e_CB_TYPE type;
	Odt *pOdtOffPulse;
	Odt ODT_OFF_PULSE;
	int iResetPulse;
} M_CB_STRUCT;



#if DBUG_MODE == 0
static M_CB_STRUCT _cbBatt;
M_CB_STRUCT _cbSSW;
static M_CB_STRUCT _cb3;
//+++by JCNET
static M_CB_STRUCT _cb1, _cb2;
//---
#else
M_CB_STRUCT _cbBatt;
M_CB_STRUCT _cbSSW;
M_CB_STRUCT _cb3;
//+++by JCNET
M_CB_STRUCT _cb1, _cb2;
//---
#endif
//+++ by JCNET
M_CB_STRUCT *SYS_GetMCbAHandle(M_CB_LIST cbId)
{
    switch(cbId)
    {
        case M_CB1_BATT: return &_cbBatt;
        case M_CB2_SSW:  return &_cbSSW;
        case M_CB1:      return &_cb1;
        case M_CB2:      return &_cb2;
        case M_CB3:      return &_cb3;
        default:
            return (M_CB_STRUCT *)0;
    }
}
//---

// CB1A Status 가 없어서 임시로.
Odt 		*odtCB1ADelay;
Odt 		ODT_CB1A_DELAY;

Odt 		*odtCB1AReOnOffDelay;
Odt 		ODT_CB1A_RE_ON_DELAY;

Odt 		*odtCB1AOffFaultDelay;
Odt 		ODT_CB1A_OFF_DELAY;

Odt 		*odtCB1FG_On_Delay;
Odt 		ODT_CB1FG_ON_DELAY;

#define RESET_DELAY		 5000	//5 second
#define RESET_ODT_PERIOD	5

//int iDsResetPulse = 0;

Bool MCB_SetType(M_CB_LIST cbId, e_CB_TYPE type )
{
	M_CB_STRUCT* cbHandle;
	cbHandle = SYS_GetMCbAHandle(cbId);
	if(!cbHandle) return FALSE;

	if( cbHandle->State != CB_OFF )
	{
		return FALSE;
	}

	if( (int)type >= (int)CBT_END )
	{
		return FALSE;
	}

	cbHandle->type = type;
	return TRUE;
}

void MCB_Create()
{
	odtCB1ADelay = Odt_(&ODT_CB1A_DELAY, 20, RESET_ODT_PERIOD);
	odtCB1AReOnOffDelay = Odt_(&ODT_CB1A_RE_ON_DELAY, 200, RESET_ODT_PERIOD);
	odtCB1AOffFaultDelay = Odt_(&ODT_CB1A_OFF_DELAY, 3100, RESET_ODT_PERIOD);
	odtCB1FG_On_Delay = Odt_(&ODT_CB1FG_ON_DELAY, 600, RESET_ODT_PERIOD);

	_cbBatt.odtcbDelay = Odt_(&_cbBatt.ODT_CB_DELAY, RESET_DELAY, RESET_ODT_PERIOD);
	_cbBatt.State=CB_RESET;
	_cbBatt.PrevState=CB_RESET;
	_cbBatt.uHistoryIdPos = EVT_STAT_BATTCB_RESET;
#if 0 // by JCNET
	_cbBatt.idON_F = CTRL_DS1_ON;
	_cbBatt.idOFF_G = CTRL_DS1_OFF;
	_cbBatt.idUVR = CTRL_DS1_UVR;
	_cbBatt.idStatus = STATUS_DS1;
	_cbBatt.idFault = FLTH_BATT_CB1;
#else
    _cbBatt.idON_F = CTRL_MC1_ON;
    _cbBatt.idOFF_G = CTRL_MC1_OFF;
    _cbBatt.idUVR = CTRL_NONE; // CTRL_DS1_UVR;
    _cbBatt.idStatus = STATUS_MC1;
    _cbBatt.idFault = FLTH_BATT_MC1;
#endif
#if USE_1000KW_FGDC_SWITCH == 1
	_cbBatt.type = CBT_F_G_UVR;
#else
	_cbBatt.type = CBT_NORMAL;
#endif
	_cbBatt.pOdtOffPulse = Odt_(&_cbBatt.ODT_OFF_PULSE, 50, RESET_ODT_PERIOD);
	_cbBatt.iResetPulse = 0;

	// by JCNET CB4 - > MC2
	_cbSSW.odtcbDelay = Odt_(&_cbSSW.ODT_CB_DELAY, RESET_DELAY, RESET_ODT_PERIOD);
	_cbSSW.State=CB_ON; /* Normal Mode */

	_cbSSW.PrevState=CB_RESET;
	_cbSSW.uHistoryIdPos = EVT_STAT_CB4_RESET;
	_cbSSW.idON_F = CTRL_MC2_ON; // CTRL_CB4_ON;
	_cbSSW.idOFF_G = CTRL_MC2_OFF; // CTRL_CB4_OFF;
	_cbSSW.idUVR = CTRL_NONE;
	_cbSSW.idStatus = STATUS_MC2;
	_cbSSW.idFault = FLTH_SSW_MC2;
	_cbSSW.type = CBT_NORMAL;
	_cbSSW.pOdtOffPulse = Odt_(&_cbSSW.ODT_OFF_PULSE, 50, RESET_ODT_PERIOD);
	_cbSSW.iResetPulse = 0;

#if 0
	_cb3.odtcbDelay = Odt_(&_cb3.ODT_CB_DELAY, RESET_DELAY, RESET_ODT_PERIOD);
	_cb3.State=CB_RESET;
	_cb3.PrevState=CB_RESET;
	_cb3.uHistoryIdPos = EVT_STAT_CB3_RESET;
	_cb3.idON_F = CTRL_CB3_ON;
	_cb3.idOFF_G = CTRL_CB3_OFF;
	_cb3.idUVR = CTRL_CB3_UVR;
	_cb3.idStatus = STATUS_CB3;
	_cb3.idFault = FLTH_GRID_CB3;
	_cb3.type = CBT_NORMAL;
	_cb3.pOdtOffPulse = Odt_(&_cb3.ODT_OFF_PULSE, 50, RESET_ODT_PERIOD);
	_cb3.iResetPulse = 0;
#endif
}

void MCB_CB1A_FaultReset()
{
	ODT_Initialize(odtCB1AOffFaultDelay);
}
static void BATTCB_StateTransition(M_CB_LIST cbId, Uns State)
{
	M_CB_STRUCT* cbHandle;

    cbHandle = SYS_GetMCbAHandle(cbId);
    if(!cbHandle) return;

	ODT_Initialize(odtCB1ADelay);
	ODT_Initialize(odtCB1AReOnOffDelay);
	ODT_Initialize(odtCB1AOffFaultDelay);
	ODT_Initialize(odtCB1FG_On_Delay);


	switch(State)
	{
		case CB_RESET:
			cbHandle->PrevState=cbHandle->State;
			cbHandle->State=CB_RESET;
			EVT_Store(cbHandle->uHistoryIdPos);
			ODT_Initialize(cbHandle->odtcbDelay);
			break;
		case CB_OFF:
			cbHandle->PrevState=cbHandle->State;
			cbHandle->State=CB_OFF;
			cbHandle->iResetPulse = 0;
			if( cbHandle->idUVR == CTRL_NONE )
			{
				MC_UpdateStatus(cbHandle->idON_F, OPEN);
			}

			if( cbHandle->type == CBT_F_G_UVR )
			{
				MC_UpdateStatus(cbHandle->idOFF_G,OPEN);
				MC_UpdateStatus(cbHandle->idON_F, OPEN);
			}
			else
			{
				MC_UpdateStatus(cbHandle->idON_F, OPEN);
				MC_UpdateStatus(cbHandle->idOFF_G,CLOSED); // TODO CHECK XXX
			}

			EVT_Store(cbHandle->uHistoryIdPos+1);
#if DBUG_MODE == 2
            if( cbId == M_CB1_BATT)
                MCCB.Status.BitVal.bDS1 = 0;
            if( cbId == M_CB4_SSW)
                MCCB.Status.BitVal.bCB4 = 0;
            if( cbId == M_CB3)
                MCCB.Status.BitVal.bCB3 = 0;
#endif
			break;
		case CB_ON:
			ODT_Initialize(cbHandle->pOdtOffPulse);
			cbHandle->PrevState=cbHandle->State;
			cbHandle->State=CB_ON;
			if( cbHandle->idUVR == CTRL_NONE )
			{
				MC_UpdateStatus(cbHandle->idOFF_G, OPEN);
			}

			if( cbHandle->type == CBT_F_G_UVR  )
			{
				MC_UpdateStatus(cbHandle->idON_F,CLOSED);
				MC_UpdateStatus(cbHandle->idOFF_G,CLOSED);
			}
			else
			{
				//-180211 MC_UpdateStatus(cbHandle->idON_F,CLOSED);
				//-180211 MC_UpdateStatus(cbHandle->idOFF_G, OPEN); // TODO CHECK XXX
				MC_UpdateStatus(cbHandle->idON_F,OPEN);
				MC_UpdateStatus(cbHandle->idOFF_G, OPEN);
			}

			EVT_Store(cbHandle->uHistoryIdPos+2);
#if DBUG_MODE == 2
            if( cbId == M_CB1_BATT)
                MCCB.Status.BitVal.bDS1 = 1;
            if( cbId == M_CB4_SSW)
                MCCB.Status.BitVal.bCB4 = 1;
            if( cbId == M_CB3)
                MCCB.Status.BitVal.bCB3 = 1;
#endif
			break;
		case CB_TRIP:
			cbHandle->PrevState=cbHandle->State;
			cbHandle->State=CB_TRIP;

			if( cbHandle->idUVR == CTRL_NONE )
			{
				cbHandle->State=CB_OFF;
				MC_UpdateStatus(cbHandle->idON_F, OPEN);
				MC_UpdateStatus(cbHandle->idOFF_G,CLOSED);
				EVT_Store(cbHandle->uHistoryIdPos+1);
				break;
			}

			if( cbHandle->type == CBT_F_G_UVR  )
			{
				MC_UpdateStatus(cbHandle->idUVR,CLOSED);
			}
			else
			{
				MC_UpdateStatus(cbHandle->idUVR,OPEN);
			}
			MC_UpdateStatus(cbHandle->idOFF_G,OPEN);
			MC_UpdateStatus(cbHandle->idON_F, OPEN);
			EVT_Store(cbHandle->uHistoryIdPos+3);
#if DBUG_MODE == 2
            if( cbId == M_CB1_BATT)
                MCCB.Status.BitVal.bDS1 = 0;
            if( cbId == M_CB4_SSW)
                MCCB.Status.BitVal.bCB4 = 0;
            if( cbId == M_CB3)
                MCCB.Status.BitVal.bCB3 = 0;
#endif
			break;
	}
}


/*
 * This function will be called periodic every 5ms to update the state of MCB module
 */
void SYS_UpdateMCbStatus(M_CB_LIST cbId)
{
	Uns		uStatus;
	M_CB_STRUCT* cbHandle;

    cbHandle = SYS_GetMCbAHandle(cbId);
    if(!cbHandle) return;

	switch (cbHandle->State)
	{
		case CB_RESET:
			MC_UpdateStatus(cbHandle->idON_F, OPEN);

			if( cbHandle->type == CBT_F_G_UVR  )
			{
				// G(OFF) 신호 OPEN
			}
			else
			{
				MC_UpdateStatus(cbHandle->idOFF_G,CLOSED);
			}

			if( cbHandle->idUVR != CTRL_NONE)// UVR 존재 타입.
			{
				if( cbHandle->type == CBT_F_G_UVR  )
				{
					MC_UpdateStatus(cbHandle->idUVR,OPEN);
				}
				else
				{
					MC_UpdateStatus(cbHandle->idUVR,CLOSED);
				}
			}
			else
			{
//				// UVR이 없는 타입이면
//				cbHandle->PrevState=cbHandle->State;
//				cbHandle->State=CB_OFF;
//				EVT_Store(cbHandle->uHistoryIdPos+1);

			}
			uStatus = ODT_Update( cbHandle->odtcbDelay, TRUE );
			if ( uStatus == ODT_FINISH )
			{
				if ( MC_GetStatus(cbHandle->idStatus) && (cbHandle->idStatus != STATUS_NONE) )
				{
					FLT_Raise(cbHandle->idFault);
				}
				else
				{
					cbHandle->PrevState=cbHandle->State;
					cbHandle->State=CB_OFF;
					MC_UpdateStatus(cbHandle->idOFF_G,OPEN);
					EVT_Store(cbHandle->uHistoryIdPos+1);
				}
			}
			break;
		case CB_OFF:
			//CB가 OFF && UVR 이 ON이면 모터 태엽이 다시 감긴다. 태엽이 다 감긴 후 ON 가능. 약 5초 이상 딜레이 필요.
				if( !MC_GetStatus(cbHandle->idStatus) ) // off
				{
					if( cbHandle->type == CBT_F_G_UVR  )
					{
						// Already Open
					}
					else
					{
						//180626 DS1 Reset 안됨 (계통 전압 없을 경우 220V는 빠져있고, DC SMPS전원만 들어가 있어서)
						//200205 CB3, CB4도 특정 상황에서 reset 안된다고 함. 일단 모두 off 신호 주기적으로 줘서 테스트
						//-if(cbId == M_CB1_BATT)
						if(InverterStateShadow != SYS_INV_RUN && InverterStateShadow != SYS_INV_ISLANDING && InverterStateShadow != SYS_INV_RE_SYNC)
						{
							cbHandle->iResetPulse++;

							if( cbHandle->iResetPulse < 600 )
							{
								MC_UpdateStatus(cbHandle->idOFF_G,CLOSED);
							}
							else
							{
								MC_UpdateStatus(cbHandle->idOFF_G,OPEN);
							}

							if( cbHandle->iResetPulse > 1200 )
								cbHandle->iResetPulse = 0;
						}
						else
						{
							MC_UpdateStatus(cbHandle->idOFF_G,OPEN);
						}
					}
				}
			break;
		case CB_ON:

			if( cbHandle->type == CBT_F_G_UVR  )
			{
				// odt가 끝나면 G를 OPEN
				if(ODT_Update( odtCB1FG_On_Delay, TRUE ) == ODT_FINISH)
				{
					MC_UpdateStatus(cbHandle->idOFF_G,OPEN);
				}
			}
			else
			{
				//+180211
				if( ODT_Update( cbHandle->pOdtOffPulse, TRUE ) == ODT_FINISH )
				{
					MC_UpdateStatus(cbHandle->idON_F,CLOSED);
					MC_UpdateStatus(cbHandle->idOFF_G, OPEN);
				}


				if( MC_GetStatus(cbHandle->idStatus) ) //CBMC_CB10 on
				{
					MC_UpdateStatus(cbHandle->idON_F,OPEN);
				}
			}
			break;
		default:
			break;
	}
}
/*
 * User will call this function to change the state of CB module
 */
void MCB_UpdateCmd(M_CB_LIST cbId, M_CB_CMD Cmd)
{
	M_CB_STRUCT* cbHandle;

    cbHandle = SYS_GetMCbAHandle(cbId);
    if(!cbHandle) return;

	switch (Cmd)
	{
		case CB_CMD_RESET:
			if(cbHandle->State==CB_TRIP)
			{
				BATTCB_StateTransition(cbId, CB_RESET);
			}
			break;
		case CB_CMD_OFF:
			if(cbHandle->State==CB_ON)
			{
				BATTCB_StateTransition(cbId, CB_OFF);
			}
			break;
		case CB_CMD_ON:
			if(cbHandle->State==CB_OFF)
			{
				BATTCB_StateTransition(cbId, CB_ON);
			}
			break;
		case CB_CMD_TRIP:
			if(cbHandle->State!=CB_TRIP)
				BATTCB_StateTransition(cbId, CB_TRIP);
			break;
	}
}

Uns MCB_GetStatus(M_CB_LIST cbId)
{
	M_CB_STRUCT* cbHandle;
    cbHandle = SYS_GetMCbAHandle(cbId);
    if(!cbHandle) return 99;

	return cbHandle->State;
}

