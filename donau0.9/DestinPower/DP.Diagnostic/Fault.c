/*
 * Fault.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#include <string.h>
#include "FAULT.h"
#include "Event.h"
#include "trace.h"
#include "PWM.h"
#include "CC.h"
#include "MCCB.h"
#include "SYSTEM.h"
#include "time.h"

#include "parameter.h"
#include "prm_pcs.h"
#include "Util.h"
#include "NvSRAM.h"
#include "CAN_GBI.h"
#include "MO.h"


typedef struct _GridFailureEventNVSRAM
{
	Uns	uB1;
	Uns	uB2;
	Uns	uB3;
	Uns	uB4;
} GridFailureEventNVSRAM;

#if DEBUG_MODE == 1
GridFailureEventNVSRAM FLT_GFE_NVSARM_ADDR;
#endif

#define FLT_TABLE_SIZE			80
#define FLT_LIGHT_START_NUM	    80

#define FLT_CLEAR_OFFSET	    400

#define NULL 0

#define FLT_IGBTL_OT_WARNING 75.
#define FLT_IGBTL_OT_HYSTERISIS 20.

Flt FLT;
FLT_PEBB_ST FLT_PEBB[PEBB_COUNT_MAX];
Uns FLT_HEAVY_TABLE[FLT_TABLE_SIZE];
Uns FLT_LIGHT_TABLE[FLT_TABLE_SIZE];

Uint32 uFault[FLT_BIT_NUM] = { 0, 0, 0, 0 }; // 0, 1 ==> �߰��� (32bit * 2)
                                             // 2, 3 ==> �����
char cFaultString[FLT_BIT_NUM * 8 + 1]; // FLT_BIT_NUM * 8 + 1(Null)

static inline void FLT_DecrementLightFlag( void )
{
	if (  FLT.uLightFlag > 0 )
		--FLT.uLightFlag;
}

static inline void FLT_DecrementHeavyFlag( void )
{
	if ( FLT.uHeavyFlag > 0 )
		--FLT.uHeavyFlag;
}

static inline void FLT_DecrementFlag( void )
{
	if ( FLT.uFlag > 0 )
		--FLT.uFlag;
}

static inline void FLT_IncrementLightFlag( void )
{
	if ( ++FLT.uLightFlag > FLT_TABLE_SIZE ) FLT.uLightFlag = FLT_TABLE_SIZE;
}

static inline void FLT_IncrementHeavyFlag( void )
{
	if ( ++FLT.uHeavyFlag > FLT_TABLE_SIZE ) FLT.uHeavyFlag = FLT_TABLE_SIZE;
}

static inline void FLT_IncrementFlag( void )
{
	if ( ++FLT.uFlag > 2*FLT_TABLE_SIZE ) FLT.uFlag = 2*FLT_TABLE_SIZE;
}


void FLT_IncreseClearDelayCnt(  )
{
	if ( ++FLT.uClearDelay >= 2 ) FLT.uClearDelay = 2;
}

void FLT_ResetClearDelayCnt(  )
{
	FLT.uClearDelay = 0;
}

void FLT_IncreseInitDelayCnt(  )
{
	if ( ++FLT.uInitDelay >= 5 ) FLT.uInitDelay = 5;
}

static void FLT_AutoResetStateTransition(Uns State)
{
	switch(State)
	{
		case FLT_STATE_AUTORESET_NORMAL:
			FLT.uAutoResetCnt = 0;
			FLT.bAutoResetInit = FALSE;
			FLT.uState = FLT_STATE_AUTORESET_NORMAL;
			EVT_Store(EVT_STATE_AUTORESET_NORMAL);
			break;
	}
}

// �� ���ʸ��� ����Ǿ�� ��
void FLT_UpdateAutoResetState()
{

	switch (FLT.uState)
	{
		case FLT_STATE_AUTORESET_NORMAL:
			if (PRM_PCS[CTRL_AUT_FAULT_RST].iValue == 0)
			{
				//if ( SYS_CheckInvCmdOn() && FLT.uHeavyFlag )
				// ON ���¿��� BATT OV�� GRID Fault�� ������ �ٸ� Heavy Fault�� �ִ°�?
				if ( SYS_CheckInvCmdOn() && FLT_HeavyFaultExceptGridFaults() && !FLT_GetEachStatus(FLTH_BATT_UV) )
				{
					// �ý��� OFF
					SYS_ClearCommand();
				}
			}
			else
			{
				FLT.uAutoResetDelayCnt = 0;
				FLT.uAutoResetIntervalCnt = 0;
				if ( FLT.bAutoResetCntClearTimerEnb )
				{
					if ( ++FLT.uAutoResetCntClearTimer > 3600 ) // 1hour
					{
						FLT.uAutoResetCnt = 0;
						FLT.uAutoResetCntClearTimer = 0;
						FLT.bAutoResetCntClearTimerEnb = FALSE;
					}
				}

				// ON ���¿��� BATT OV�� GRID Fault�� ������ �ٸ� Heavy Fault�� �ִ°�?
				if ( /*SYS_CheckInvCmdOn() &&*/ FLT_HeavyFaultExceptGridFaults() )
				{
					FLT.uState = FLT_STATE_AUTORESET_DELAY;
					EVT_Store(EVT_STATE_AUTORESET_DELAY);
				}
			}
			break;
		case FLT_STATE_AUTORESET_DELAY:
			if ( FLT.bAutoResetInit )
			{
				FLT_AutoResetStateTransition(FLT_STATE_AUTORESET_NORMAL);
			}
			else if ( FLT.uAutoResetCnt >= PRM_PCS[CTRL_AUT_FAULT_RST].iValue )
			{
				FLT.uState = FLT_STATE_AUTORESET_CLR_CMD;
				EVT_Store(EVT_STATE_AUTORESET_CLR_CMD);
			}
			else if ( ++FLT.uAutoResetDelayCnt > 2 )
			{
				FLT_Initialize();
				++FLT.uAutoResetCnt;
				FLT.uState = FLT_STATE_AUTORESET;
				EVT_Store(EVT_STATE_AUTORESET);
			}
			break;
		case FLT_STATE_AUTORESET:
			if (FLT.uAutoResetCnt==1)
				FLT.bAutoResetCntClearTimerEnb = TRUE;
			if ( FLT.bAutoResetInit )
			{
				FLT_AutoResetStateTransition(FLT_STATE_AUTORESET_NORMAL);
			}
			else if ( FLT.uAutoResetCnt > PRM_PCS[CTRL_AUT_FAULT_RST].iValue )
			{
				FLT.uState = FLT_STATE_AUTORESET_CLR_CMD;
				EVT_Store(EVT_STATE_AUTORESET_CLR_CMD);
			}
			//else if ( FLT.uHeavyFlag == 0 )
			else if ( FLT_HeavyFaultExceptGridFaults() == FALSE )
			{
				FLT.uState = FLT_STATE_AUTORESET_NORMAL;
				EVT_Store(EVT_STATE_AUTORESET_NORMAL);
			}
			else if ( ++FLT.uAutoResetIntervalCnt > 30 /* 100 sec -> 30 sec */ )
			{
				FLT_Initialize();
				++FLT.uAutoResetCnt;
				FLT.uAutoResetIntervalCnt = 0;
			}
			break;
		case FLT_STATE_AUTORESET_CLR_CMD:

			SYS_ClearCommand();
			if ( FLT.bAutoResetInit )
			{
				FLT_AutoResetStateTransition(FLT_STATE_AUTORESET_NORMAL);
			}
			break;
	}
}

void FLT_UpdateAutoResetInit()
{
	//- 140609 Posco
	// INVERTER.bGridFault_runState = FALSE;

	if ( FLT.uState != FLT_STATE_AUTORESET_NORMAL )
		FLT.bAutoResetInit = TRUE;
}

/*******************************************************************/
/* FLT_Create - FAULT ��ü �ʱ�ȭ                                  */
/* Parameters : void                                               */
/* Returns : void                                                  */
/*******************************************************************/
void FLT_Create( void )
{
	memset(&FLT, 0, sizeof(FLT));
	memset(&FLT_HEAVY_TABLE, 0, sizeof(FLT_HEAVY_TABLE));
	memset(&FLT_LIGHT_TABLE, 0, sizeof(FLT_LIGHT_TABLE));
	memset(&cFaultString, 0, sizeof(cFaultString));
	memset(&FLT_PEBB, 	  0, sizeof(FLT_PEBB));
	memset(&uFault, 	  0, sizeof(uFault));

}

/*******************************************************************/
/* FLT_Initialize - FAULT ��ü �ʱ�ȭ                              */
/* Parameters : void                                               */
/* Returns : void                                                  */
/*******************************************************************/
void FLT_Initialize( void )
{
	int i = 0;

	if( PRM_PCS[CTRL_TEST_MODE].iValue == 5 )
	{
		INVERTER.gridReconnect.bEnb = FALSE;
	}

	// GBI Fault Reset
	MO_PebbFaultReset();

	if ( FLT.uHeavyFlag ) //�߰����� �߻��� ��쿡 �ѿ�����
	{
		/* PWM�߻����� */
		PWM_Disable(PWM_CON);

		for(i=1; i<=63; i++)
		{
			if ( FLT_HEAVY_TABLE[i] )
				FLT_Clear(i);
		}
	}

	if ( FLT.uLightFlag ) //������� �߻��� ��쿡 �ѿ�����
	{
		for(i = 81; i<=143; i++)
		{
			if( i == FLTL_TEST_RUN)
			{
				if ( PRM_PCS[CTRL_TEST_MODE].iValue == 0)
						FLT_Clear(FLTL_TEST_RUN);
			}
			else
			{
				if ( FLT_LIGHT_TABLE[i - FLT_LIGHT_START_NUM] )
						FLT_Clear(i);
			}
		}
	}



	FLT.cNthStatus[0] = 0;

	//������ �����Ǹ� Ʈ���̽��� �����Ѵ�.
	if ( FLT.uHeavyFlag == FALSE )
	{
		//-180209 TRC_TraceReset();
	}

//    asm("	POP		IE		");
}

/*******************************************************************/
/* FLT_CheckHW - H/W Fault ����                                    */
/* Parameters : void                                               */
/* Returns : void                                                  */
/* Remark : �ݵ�� 5msec ���� ȣ��Ǿ�� ��                        */
/*******************************************************************/
#define FAULT_TICK		4 // 20msec/5msec
void FLT_CheckHW(void)
{
	static int tickEPO = 0;
	static int tickRTOT = 0;
	static int tickTROT = 0;
	//static int tickGFD = 0;
	static int tickSSWOT = 0;
	static int tickSSWFuse = 0;
	int i = 0;

	if ( FLT.uInitDelay < 5 ) return;

#if DBUG_MODE != 2
#if 0 //by JCNET
	if ( MC_GetStatus(STATUS_EPO) ) // ���� ����
		tickEPO++;
	else
		tickEPO = 0;

	if ( tickEPO>FAULT_TICK )
	{
		tickEPO = FAULT_TICK;
		FLT_Raise( FLTH_EPO );
	}
#endif
	if( PRM_PCS[CTRL_TEST_MODE].iValue == 1 || PRM_PCS[CTRL_TEST_MODE].iValue == 2)
		return;

	//���� STABLEEN#4(Y1-2) Door DI �̻���. ���� Y1 2�� Door ���� ��Ȱ��ȭ
#if 1 // by JCNET 1 -> 0 (������� ǰ)
	if( PRM_PCS[SYS_OPTION].iValue & 0x100 )
	{
		if( !MC_GetStatus(STATUS_CB2) || !MC_GetStatus(STATUS_DOOR) )
			FLT_Raise(FLTH_DOOR_OPEN);

		FLT_Clear(FLTL_DOOR_DC_OPEN);
		FLT_Clear(FLTL_DOOR_AC_OPEN);
	}
	else
	{
		if( !MC_GetStatus(STATUS_DOOR) )
			FLT_Raise(FLTL_DOOR_AC_OPEN);
		else
			FLT_Clear(FLTL_DOOR_AC_OPEN);
	}
#endif

#if 1 // by JCNET (���Ҵٰ� ������� ǰ)
#if 0 //  RETEMP, TRTEMP �� ���°� ���Ƽ� ����
#if DEBUG_MODE == NORMAL
	if ( !MC_GetStatus(STATUS_RETEMP) )
		tickRTOT++;
	else
		tickRTOT = 0;
	if ( tickRTOT>FAULT_TICK )
	{
		tickRTOT = FAULT_TICK;
		FLT_Raise( FLTH_RE_TEMP );
	}
#endif

	if ( !MC_GetStatus(STATUS_TRTEMP) )
		tickTROT++;
	else
		tickTROT = 0;

	if ( tickTROT>FAULT_TICK )
	{
		tickTROT = FAULT_TICK;
		FLT_Raise( FLTH_TR_TEMP );
	}

	if ( !MC_GetStatus(STATUS_SMPS) )
	{
		// FRŸ���� ��� �� ��ȣ�� DC, AC SMPS ��ȣ�� ��� ����.
		FLT_Raise( FLTL_SMPS );
	}
	else
		FLT_Clear( FLTL_SMPS );

	if ( !MC_GetStatus(STATUS_SPD1) )
	{
		//XXX TODO FLT? & hold time.
		FLT_Raise( FLTL_SPD1 );
	}
//	else
//	{
//		FLT_Clear( FLTL_SPD );
//	}
	/*
	if ( !MC_GetStatus(STATUS_SPD4) )
	{
		FLT_Raise( FLTL_SPD4 );
	}
	*/
//	else
//	{
//		FLT_Clear( FLTL_SPD4 );
//	}
#endif // ���̾��ݿ��� ����.. Ȯ�� �ʿ���

#if 0
	if ( !MC_GetStatus(STATUS_SSW_OT) )
		FLT_Raise( FLTH_SSW_OT );
#else
	if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
	{
//by JCNET		if ( !MC_GetStatus(STATUS_SSW_OT) )
	    if ( MC_GetStatus(STATUS_SSW_OT) )
			tickSSWOT++;
		else
			tickSSWOT = 0;

		if ( tickSSWOT > FAULT_TICK )
		{
			tickSSWOT = FAULT_TICK;
			FLT_Raise( FLTH_SSW_OT );
		}
	}
#endif

#if 0 //by JCNET
    if ( !MC_GetStatus(STATUS_SSW_FUSE) )
        tickSSWFuse++;
    else
        tickSSWFuse = 0;

    if ( tickSSWFuse > FAULT_TICK )
    {
        tickSSWFuse = FAULT_TICK;
        FLT_Raise( FLTH_SSW_FUSE );
    }
#endif

/*
	if( PRM_PCS[SYS_OPTION].iValue & 0x1 )
	{
		if ( MC_GetStatus(STATUS_GFD_DDIA_END) == FALSE )
		{
			if(INVERTER.uStatus != SYS_INV_INIT && INVERTER.uStatus != SYS_INV_FAULT
								&& INVERTER.uStatus != SYS_INV_STOP && (BATCTRL.fltDCLinkV.fOut > 400.0))
				tickGFD++;
			else
				tickGFD = 0;

			if( tickGFD > FAULT_TICK )
			{
				tickGFD = FAULT_TICK;
				FLT_Raise( FLTH_GFD );
			}
		}
		else
		{
			tickGFD = 0;
		}
	}*/ //2020. 10. 14 GFD Fault ����
#endif


	for(i = 0; i < PEBB_COUNT_MAX; i++ )
	{
		if( MO_FaultResetCheck() )
			break;

		if(     FLT_PEBB[i].data.STATE.BIT.IGBT_R_FAULT ||
				FLT_PEBB[i].data.STATE.BIT.IGBT_S_FAULT ||
				FLT_PEBB[i].data.STATE.BIT.IGBT_T_FAULT
				)
		{
			FLT_Raise(FLTH_PEBB_IGBT);
			PRM_PCS[BATT_PEBB_FAULT_ID].iValue |= 1 << i; // ��Ʈ Set

			if( FLT_PEBB[i].data.STATE.BIT.IGBT_R_FAULT )
				PRM_PCS[BATT_REBB_FAULT_RST].iValue = 1;
			else if( FLT_PEBB[i].data.STATE.BIT.IGBT_S_FAULT )
				PRM_PCS[BATT_REBB_FAULT_RST].iValue = 2;
			else if( FLT_PEBB[i].data.STATE.BIT.IGBT_T_FAULT )
				PRM_PCS[BATT_REBB_FAULT_RST].iValue = 3;
		}
	//		else
	//		{
	//		    Clear �� Fault Reset���� ����.
	//			PRM_PCS[BATT_PEBB_FAULT_ID].iValue &= ~(1 << i); // ��Ʈ Clear
	//		}

		if( FLT_PEBB[i].data.STATE.BIT.EMERGENCY )
		{
			FLT_Raise(FLTH_PEBB_OC);
			PRM_PCS[BATT_PEBB_FAULT_ID].iValue |= 1 << i; // ��Ʈ Set
		}

		if(FLT_PEBB[i].data.STATE.BIT.IGBT_OVERTEMP)
		{
			FLT_Raise(FLTH_PEBB_OVERTEMP);
			PRM_PCS[BATT_PEBB_FAULT_ID].iValue |= 1 << i; // ��Ʈ Set
		}

		if(FLT_PEBB[i].data.STATE.BIT.IGBT_FAN_FAULT)
		{
			FLT_Raise(FLTH_PEBB_FAN_FAULT);
			PRM_PCS[BATT_PEBB_FAULT_ID].iValue |= 1 << i; // ��Ʈ Set
		}
	#if DEBUG_MODE == NORMAL
		if(FLT_PEBB[i].data.STATE.BIT.IGBT_FUSE_FAULT)
		{
			FLT_Raise(FLTH_PEBB_FUSE_FAULT);
			PRM_PCS[BATT_PEBB_FAULT_ID].iValue |= 1 << i; // ��Ʈ Set
		}
	#endif

		if(FLT_PEBB[i].data.STATE.BIT.PARAMETER_ERR)
		{
			FLT_Raise(FLTL_GBI_PARAM_ERR);
			PRM_PCS[BATT_PEBB_FAULT_ID].iValue |= 1 << i; // ��Ʈ Set
		}

		if(FLT_PEBB[i].data.STATE.BIT.HEATSYNC_OT)
		{
			FLT_Raise(FLTH_PEBB_OVERTEMP);
			PRM_PCS[BATT_PEBB_FAULT_ID].iValue |= 1 << i; // ��Ʈ Set
		}

		/*
		 * MCU ��ü OT Fault �߻� ó��. 1�������� gbi ������ ó���ϳ�, MCU������ Level2 �뵵�� ó�� ��.
		 */
		if( g_gbiRxData[i].rxM4.tempHeatSync >= PARAM_VAL(CTRL_PEBB_HS_OVERTEMP))
		{
			FLT_Raise(FLTH_PEBB_OVERTEMP);
		}
	}
#endif // by JCNET
	if ( FLT.uHeavyFlag )
	{
		PWM_Disable(PWM_CON);
        PWM_Disable(PWM_SCC); //by JCNET
	}

}

/*******************************************************************/
/* FLT_Handling - ���� Fault �߻��� ó�� �� DCC, IGBT ���� ����    */
/* Parameters : void                                               */
/* Returns : void                                                  */
/*******************************************************************/
void FLT_Handling( void )
{
	FLT_CheckHW();
}

/*******************************************************************/
/* FLT_Raise - FAULT�� �߻��ϸ� �������̺� ��ȣ ��Ϲ� �÷��� ON */
/* Parameters : int ���� ����(ID)                                  */
/* Returns : void                                                  */
/*******************************************************************/
static Uns jcnet_heavyMask[FLT_HEAVY_MAX] = {
0, // �ε��� 0�� ���ڸ��� ��.. ��ȣ ���򰥸���..
0, //#define FLTH_START_FAILURE          1
0, //#define FLTH_BATT_OV                2
0, //#define FLTH_BATT_OC                3
0, //#define FLTH_BATT_UV                4
0, //#define FLTH_BATT_CB1               5
0, //#define FLTH_BATT_CB1A              6
0, //#define FLTH_BATT_POLARITY          7
0, // 8
0, //#define FLTH_DC_SIDE_ABNORMAL       9
0, //#define FLTH_INV_OV                 10
0, //#define FLTH_INV_UV                 11
0, //#define FLTH_INV_OF                 12
0, //#define FLTH_INV_UF                 13
0, //#define FLTH_INV_OC                 14
0, //#define FLTH_INV_PHASE              15
0, //#define FLTH_INV_PAHSE_JUMP         16
0, //#define FLTH_INV_CURR_UNBAL         17
0, //#define FLTH_BATT_I                 18
0, //#define FLTH_PEBB_OC                19
0, //#define FLTH_PEBB_OVERTEMP          20
0, //#define FLTH_PEBB_FAN_FAULT         21
0, //#define FLTH_PEBB_FUSE_FAULT        22
0, //#define FLTH_PEBB_IGBT              23

// MC1A, CB5~7
0, //#define FLTH_MC1A                   24
0, //#define FLTH_CB5                    25
0, //#define FLTH_CB6                    26
0, //#define FLTH_CB7                    27
0, //#define FLTH_EVE_FAULT              28
0, //#define FLTH_SSW_CB4                29 // SSW side CB
0, //#define FLTH_GRID_OV_LEVEL1         30
0, //#define FLTH_GRID_UV_LEVEL1         31
0, //#define FLTH_GRID_OF_LEVEL1         32
0, //#define FLTH_GRID_UF_LEVEL1         33
0, //#define FLTH_GRID_CB3               34
0, //#define FLTH_GRID_OV_LEVEL2         35
0, //#define FLTH_GRID_UV_LEVEL2         36
0, //#define FLTH_GRID_UF_LEVEL2         37
0, //#define FLTH_GRID_OF_LEVEL2         38
//MCB7~9
0, //#define FLTH_MCB7                   39
0, //#define FLTH_MC8                    40
0, //#define FLTH_MC9                    41
0, //#define FLTH_SSW_OT                 42
0, //#define FLTH_TR_TEMP                43
0, //#define FLTH_RE_TEMP                44

0, //#define FLTH_SSW_FUSE               45
0, //#define FLTH_ASYNC                  46
0, //#define FLTH_CAN                    47
0, // 48
0, //#define FLTH_EPO                    49 // emergency Stop
0, //#define FLTH_FIRE                   50 // ����
0, //#define FLTH_GFD                    51 // ���� (DC�� ����)
0, //#define FLTH_PARA_VERSION           52

/* FR */
0, //#define FLTH_DOOR_OPEN              53 // PCS �ǳ� ���� ����
0, //#define FLTH_DC_CHARGE_FAILED       54 // DC Link �ʱ� ���� ����
0, //55
0, //#define FLTH_AC_GFD                 56 // AC ����( PCS�� ���� �߿��� 51�� ���� �߻���Ŵ-DC�������� �� )
0, //#define FLTH_GRID_ZERO_VOLTAGE      57 // Grid Zero Voltage
0, //#define FLTH_AC_FUSE                58 // ��� �ȵ�
0, //#define FLTH_LPMS_ESTOP             59 // LPMS E-STOP ( ��� ��ɿ� ���� )

0, //#define FLTH_LOAD_OC                60 // LOAD OC

0, //#define FLTH_PM_OVERLOAD            61
0, //#define FLTH_ETC                    62
};
void FLT_Raise( int FltNum )
{
	Uns LightFltNum;

	if ( FltNum <= FLT_HEAVY_MAX ) // �߰��� �߻�
	{
//by JCNET : ���� heavyFault �߰��ϴ°ɷ�..
	if(jcnet_heavyMask[FltNum]) // �׽�Ʈ�� heavyFault mask off �� �� ���� �ʰ�.
	{
		PWM_Disable(PWM_CON);    // !! TODO by JCNET , inverter pwm ? converter ? or both ??
		PWM_Disable(PWM_SCC);    // by JCNET inverter side PWM
		GPIO_StaticSwitch(GPIO_STATIC_SW_ON);  // !! TODO by JCNET
		if ( !FLT_HEAVY_TABLE[FltNum] )
		{
			// �̹� �߻��Ǿ� �ִ� ������ �ƴ� ��쿡�� ������ �߻���Ŵ.
			//-IRQ_globalDisable();
			FLT_IncrementFlag();
			FLT_IncrementHeavyFlag();
			FLT_HEAVY_TABLE[FltNum] = TRUE;
			//+IRQ_globalEnable();

			// �̺�Ʈ ���ÿ� �����Ѵ�.
			EVT_Store(FltNum);
		}
    }
//	else return; // �߰����ϴ� ó������ ����.. �׽�Ʈ��.. JCNET
#if 0 //by JCNET
		//-130806 Delete.if ( !Model_hasManualDCSwitch() )
		{
			if ( PRM_PCS[CTRL_TEST_MODE].iValue != 3 )
			{
				MC_UpdateStatus(CTRL_DS1_UVR, OPEN);
			}
			else
			{
				if ( FltNum<FLTH_INV_OV || ( FltNum>FLTH_INV_UF && FltNum<FLTH_GRID_OV_LEVEL1) || FltNum>FLTH_GRID_OF_LEVEL2 )
				{
					MC_UpdateStatus(CTRL_DS1_UVR, OPEN);
				}
			}
		} //!Model_hasManualDCSwitch()
#endif
		// TODO FIX CB3
		//MC_UpdateStatus(CTRL_MC4, OPEN);

		// �߰��� ID�� 1���� ���۵ʿ� ���� bit operation�� ���Ͽ� -1 ó����
		FltNum--;
		if ( FltNum < 32)
		{
				uFault[0] |= ((Uint32)1<<FltNum);
		}
		else
		{
			uFault[1] |= ((Uint32)1<<(FltNum-32));
		}

		if( TRC_FAULT_ONLY_STOP == PRM_PCS[TRC_TRACE_MODE].iValue )
			TRC_StopTrace();

	}
	else if (FltNum < FLT_LIGHT_MAX)
	{
		LightFltNum = FltNum - FLT_LIGHT_START_NUM;

		if ( !FLT_LIGHT_TABLE[LightFltNum] )
		{
			// �̺�Ʈ ���ÿ� �����Ѵ�.
			EVT_Store(FltNum);
			/* Fault Raise Flag On */
			FLT_IncrementFlag();
			FLT_IncrementLightFlag();
			FLT_LIGHT_TABLE[LightFltNum] = TRUE;
		}

		if ( LightFltNum < 32)
			uFault[2] |= ((Uint32)1<<LightFltNum);
		else
			uFault[3] |= ((Uint32)1<<(LightFltNum-32));
	}
}

/******************************************************************/
/* FLT_Clear - FAULT ����                            				  */
/* Parameters : int ���� ����(ID)                                  */
/* Returns : void                                                 */
/******************************************************************/
void FLT_Clear( int FltNum )
{
	int LightFltNum;

	if ( FltNum <= FLT_HEAVY_MAX )
	{
		if ( FLT.uClearDelay < 2 ) return;

		// ���ſ� ������ �߻��Ǿ��� ��Ȳ�̶�� ������ �����ϰ� �̺�Ʈ�� �߻� ��Ŵ
		if ( FLT_HEAVY_TABLE[FltNum] )
		{
			// �̺�Ʈ ���ÿ� ���Ѵ�.
			EVT_Store( FLT_CLEAR_OFFSET + FltNum );
			FLT_DecrementFlag();
			FLT_DecrementHeavyFlag();
			FLT_HEAVY_TABLE[FltNum] = FALSE;
		}

		if ( FLT.uHeavyFlag == FALSE )
		{
			//TRC_TraceReset();
		}

		// �߰��� ID�� 1���� ���۵ʿ� ����
		// bit operation�� ���Ͽ� -1 ó����
		FltNum--;
		if ( FltNum < 32)
			uFault[0] &= ~((UInt32)1<<FltNum);
		else
			uFault[1] &= ~((UInt32)1<<(FltNum-32));
	}
	else if (FltNum < FLT_LIGHT_MAX)
	{
		LightFltNum = FltNum - FLT_LIGHT_START_NUM;
		if ( FLT_LIGHT_TABLE[LightFltNum] )
		{
			// �̺�Ʈ ���ÿ� �����Ѵ�.
			EVT_Store( FLT_CLEAR_OFFSET + FltNum );
			FLT_DecrementFlag();
			FLT_DecrementLightFlag();
			FLT_LIGHT_TABLE[LightFltNum] = FALSE;
		}

		if ( LightFltNum < 32)
			uFault[2] &= ~((UInt32)1<<LightFltNum);
		else
			uFault[3] &= ~((UInt32)1<<(LightFltNum-32));
	}

	// PEBB ���� Fault Clear

}

/*******************************************************************/
/* FLT_GetStatus - ����߻� ���� ����                              */
/* Parameters : void                                               */
/* Returns : Uns                                                   */
/*******************************************************************/
Uns FLT_GetStatus( void )
{
	return	FLT.uFlag;
}

Uns FLT_GetHeavyStatus( void )
{
	return	FLT.uHeavyFlag;
}

Uns FLT_GetLightStatus( void )
{
    //JCNET for test
 //   return 0;
	return	FLT.uLightFlag;
}

Uns FLT_GetEachStatus( int FltNum )
{
	if ( FltNum <= FLT_HEAVY_MAX )
	{
		return	FLT_HEAVY_TABLE[FltNum];
	}
	else if (FltNum < FLT_LIGHT_MAX)
	{
		return	FLT_LIGHT_TABLE[FltNum - FLT_LIGHT_START_NUM];
	}
	return FLT_HEAVY_MAX;
}

Bool FLT_ConVoltageFailure( void )
{
	if (FLT_GetEachStatus(FLTH_INV_OF) || FLT_GetEachStatus(FLTH_INV_UF) || FLT_GetEachStatus(FLTH_INV_OV)
			|| FLT_GetEachStatus(FLTH_INV_UV) || FLT_GetEachStatus(FLTH_INV_PHASE)/*//-130705Deleted@|| FLT_GetEachStatus(FLTH_INV_INSTANT_UV)*/ )
		return TRUE;
	else
		return FALSE;
}

Bool FLT_GridVoltageFailure( void )
{
	if (FLT_GetEachStatus(FLTH_GRID_OF_LEVEL1) || FLT_GetEachStatus(FLTH_GRID_OF_LEVEL2) ||
		FLT_GetEachStatus(FLTH_GRID_UF_LEVEL1) || FLT_GetEachStatus(FLTH_GRID_OV_LEVEL1) || FLT_GetEachStatus(FLTH_GRID_UV_LEVEL1) ||
		FLT_GetEachStatus(FLTH_GRID_UF_LEVEL2) || FLT_GetEachStatus(FLTH_GRID_OV_LEVEL2) || FLT_GetEachStatus(FLTH_GRID_UV_LEVEL2) ||
		FLT_GetEachStatus(FLTL_GRID_UV)
		//||
		//TODO Check (INVERTER.uStatus == SYS_INV_DISCONNECT && !SYS_ReConnectionCondition()))
		//-FIXMED (INVERTER.uStatus == SYS_INV_STOP && !SYS_ReConnectionCondition()))
		//-DELETED(INVERTER.uStatus == SYS_INV_STOP))
		)
		return TRUE;
	else
		return FALSE;
}

Bool FLT_BattVoltageFailure( void )
{
	if (FLT_GetEachStatus(FLTH_BATT_OV) || BATCTRL.fltDCBattV_2nd.fOut < PARAM_VAL(BATT_V_RANGE_MIN))
		return TRUE;
	else
		return FALSE;
}


/*******************************************************************/
/* FLT_GetStatusString                                             */
/* Parameters : void                                               */
/* Returns : String                                                */
/* Called : ESCI.c                                                 */
/*******************************************************************/
String FLT_GetStatusString(void)
{
	int j, i;

	for ( j = 0; j < FLT_BIT_NUM; j++ )
	{
		for ( i = 0; i < 8; i++ )
		{
			cFaultString[i+(8*j)] = Uns2Hex( (uFault[j]>>(4*i))&0xF );
		}
	}
	cFaultString[FLT_BIT_NUM * 8] = 0; // Null

	return cFaultString;
}

void FLT_CabinetTemp()
{
#if 0
	static Uns CntOT = 0;
	static Uns CntUT = 0;


	INVCTRL.TempCabinetMax = PRM_PCS[INV_CAB_T_MAX].iValue;
	if (INVCTRL.fltTempCabinet2.fOut > INVCTRL.TempCabinetMax)
	{
		if (++CntOT>=60)
		{
			FLT_Raise(FLTL_CABINET_OT_WARNING);
			CntOT = 0;
		}
	}
	else
	{
		CntOT = 0;
		if (FLT_GetEachStatus(FLTL_CABINET_OT_WARNING))
			FLT_Clear(FLTL_CABINET_OT_WARNING);
	}

	INVCTRL.TempCabinetMin = PRM_PCS[INV_CAB_T_MIN].iValue;
	if (INVCTRL.fltTempCabinet2.fOut < INVCTRL.TempCabinetMin)
	{
		if (++CntUT>=60)
		{
			FLT_Raise(FLTL_CABINET_UT);
			CntUT=0;
		}
	}
	else
	{
		CntUT = 0;
		if (FLT_GetEachStatus(FLTL_CABINET_UT))
		FLT_Clear(FLTL_CABINET_UT);
	}
#endif
}

void FLT_SetGridFailureEvent( Bool bSetVal )
{
	FLT.bGridFailureEvent = bSetVal;

	//TODO Something... ���� �ؾ� ���� ��Ȳ������ �ڵ� �� �⵿ ����
	//-NvSRAM_SetFaultCommand(bSetVal);
}

Bool  FLT_GetGridFailureEvent( void )
{
	Bool RetVal = OFF;

	if (FLT.bGridFailureEvent)
		RetVal = ON;
#if 0 // �� �Լ��� ��� ȣ�� �Ͽ�, �ٸ� ���� NvSRAM�� �۾��� ����.
	else
	{
		if(NvSRAM_CheckFaultCommand())
		{
			FLT.bGridFailureEvent = ON;
			RetVal = ON;
		}
	}
#endif

	return RetVal;
}

Bool FLT_HeavyFaultExceptGridFaults( void )
{
	int idx;
	Bool bRet = FALSE;

	//XXX TODO FIX

	if ( FLT.uHeavyFlag )
	{
		/*
		 * GRID�� ������ �̿��� �߰����� �ִ��� �˻�.
		 */
		for(idx=0; idx < FLT_HEAVY_MAX; idx++)
		{
			if( idx >= FLTH_GRID_OV_LEVEL1 && idx <= FLTH_GRID_OF_LEVEL2)
			{
				continue;
			}
			else if (FLT_HEAVY_TABLE[idx])
			{
				bRet = TRUE;
				break;
			}
		}


		// CB20 Trip, �������� �ʴ� ��Ȳ���� Fault reset count�� �ö��� �ʴ´�.
		//TODO Check if ( INVERTER.uStatus == SYS_INV_MPPT || INVERTER.uStatus == SYS_INV_START && FLT_HEAVY_TABLE[FLTH_GRID_CB_TRIP] )
		if ( INVERTER.uStatus == SYS_INV_RUN && FLT_HEAVY_TABLE[FLTH_GRID_CB3] )
		{
			bRet = TRUE;
		}
	}

	return bRet;
}
