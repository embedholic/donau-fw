/*
 * MCCB.c
 *
 *  Created on: 2013. 2. 15.
 *      Author: destinPower
 */

#include <string.h>
#include "MCCB.h"
#include "MCB.h"
#include "FAULT.h"
#include "Event.h"
#include "ODT.h"
#include "DIO.h"
//-#include "DBUG.h"
#include "trace.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "DBUG.h"

#if DBUG_MODE == 3
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#endif

#define CBMC_FAULT_CHECK_DELAY 	500 	// msec
#define CBMC_ODT_PERIOD 		5 		// msec
#define CBMC_CLOSED '1'
#define CBMC_OPEN '0'

Uint32 u32CBMCStatus; // Call: MCCB_Create(), MCCB_UpdateAllStatus(), ResDIOSTATUS()

#if DBUG_MODE == 0
Cbmc MCCB;
#else
Cbmc MCCB;
#endif
Cbmc MCCB_MIRROR;

static Odt *odtMC1AOn;
static Odt *odtMC1AOff;
static Odt *odtMC8On;
static Odt *odtMC8Off;
static Odt *odtMC9On;
static Odt *odtMC9Off;
//-static Odt *odtMC6On;
//-static Odt *odtMC6Off;

static Odt ODT_Module1A;
static Odt ODT_Module1B;
static Odt ODT_Module2A;
static Odt ODT_Module2B;
static Odt ODT_Module3A;
static Odt ODT_Module3B;
//-static Odt ODT_MC6_ON;
//-static Odt ODT_MC6_OFF;

void MCCB_Create( void )
{
	memset(&MCCB, 0, sizeof(MCCB));
	memset(&ODT_Module1A, 0, sizeof(ODT_Module1A));
	memset(&ODT_Module1B, 0, sizeof(ODT_Module1B));
	memset(&ODT_Module2A, 0, sizeof(ODT_Module2A));
	memset(&ODT_Module2B, 0, sizeof(ODT_Module2B));
	memset(&ODT_Module3A, 0, sizeof(ODT_Module3A));
	memset(&ODT_Module3B, 0, sizeof(ODT_Module3B));
//-	memset(&ODT_MC6_ON, 0, sizeof(ODT_MC6_ON));
//-	memset(&ODT_MC6_OFF, 0, sizeof(ODT_MC6_OFF));

	u32CBMCStatus = 0; //cCBMCStatus[]를 스트링으로 만듬

	odtMC1AOn = Odt_(&ODT_Module1A, CBMC_FAULT_CHECK_DELAY, CBMC_ODT_PERIOD);
	odtMC1AOff = Odt_(&ODT_Module1B, CBMC_FAULT_CHECK_DELAY, CBMC_ODT_PERIOD);
	odtMC8On = Odt_(&ODT_Module2A, CBMC_FAULT_CHECK_DELAY, CBMC_ODT_PERIOD);
	odtMC8Off = Odt_(&ODT_Module2B, CBMC_FAULT_CHECK_DELAY, CBMC_ODT_PERIOD);
	odtMC9On = Odt_(&ODT_Module3A, CBMC_FAULT_CHECK_DELAY, CBMC_ODT_PERIOD);
	odtMC9Off = Odt_(&ODT_Module3B, CBMC_FAULT_CHECK_DELAY, CBMC_ODT_PERIOD);
//-	odtMC6On = Odt_(&ODT_MC6_ON, CBMC_FAULT_CHECK_DELAY, CBMC_ODT_PERIOD);
//-	odtMC6Off = Odt_(&ODT_MC6_OFF, CBMC_FAULT_CHECK_DELAY, CBMC_ODT_PERIOD);

}

/*
 * Call: c_int25. 5ms 마다 호출 됨.
 */
void MCCB_UpdateAllStatus(void)
{
	static CbmcStatus Prev = {0};
	DI_STATE diData = {0};
	DIO_CTRL_STATUS res;

	u32CBMCStatus = MCCB.Status.UnsVal; // for communication
	MCCB_MIRROR.Status.UnsVal = MCCB.Status.UnsVal; // for communication

#if DEBUG_MODE == DEBUG_SIMULSATION
		MCCB_MIRROR.Status.BitVal.bCB4 = 1;
#endif

#if DBUG_MODE == 2 //-|| DBUG_MODE == 3
	return ;
#endif

#if 0
	if ( PRM_PCS[SYS_DPWM_OFFTEMP].iValue == 1 )
	{
		res = DIO_getStatus(DI_GROUP_B, &diData );

		if( res == dio_command_succ)
		{
			MCCB.Status.BitVal.bMC4 = diData.bit.PORT4;
		}

		res = DIO_getStatus(DI_GROUP_C, &diData );

		if( res == dio_command_succ)
		{
			MCCB.Status.BitVal.bTRTEMP = diData.bit.PORT1;
			MCCB.Status.BitVal.bCB4 = diData.bit.PORT2;
			MCCB.Status.BitVal.bMC8 = diData.bit.PORT3;
			MCCB.Status.BitVal.bMC9 = diData.bit.PORT4;
			MCCB.Status.BitVal.bCB4E = diData.bit.PORT5;
			MCCB.Status.BitVal.bADIB5 = diData.bit.PORT6;
			MCCB.Status.BitVal.bADIB6 = diData.bit.PORT7;
			MCCB.Status.BitVal.bADIB7 = diData.bit.PORT8;
		}
	}
	goto EVENT;
#endif

	//Read DDI
	res = DIO_getStatus(DI_GROUP_A, &diData );
#if DEBUG_MODE == 1
	if( res != dio_command_succ)
		error();
#endif


	if( res == dio_command_succ)
	{
#if 0 //by JCNET
		MCCB.Status.BitVal.bDS1 = 	 diData.bit.PORT1;
		MCCB.Status.BitVal.bCB2 = diData.bit.PORT2;
		MCCB.Status.BitVal.bSMPS = diData.bit.PORT3;
		MCCB.Status.BitVal.bSPD1 = 	 diData.bit.PORT4;
		MCCB.Status.BitVal.bEPO =    diData.bit.PORT5;
		MCCB.Status.BitVal.bSSW_OT =  diData.bit.PORT6;
		MCCB.Status.BitVal.bMC1A =	 	diData.bit.PORT7;
		MCCB.Status.BitVal.bGFD =   	diData.bit.PORT8;
#else
        MCCB.Status.BitVal.bMC1 =    diData.bit.PORT1;
        MCCB.Status.BitVal.bMC1A = diData.bit.PORT2;
        MCCB.Status.BitVal.bInductor = diData.bit.PORT3;
        MCCB.Status.BitVal.bFuseRec =   diData.bit.PORT4;
        MCCB.Status.BitVal.bPowerFault =    diData.bit.PORT5;
        MCCB.Status.BitVal.bFuseInv =  diData.bit.PORT6;
        MCCB.Status.BitVal.bDoor =      diData.bit.PORT7;
        MCCB.Status.BitVal.bMC2 =       diData.bit.PORT8;
#endif
	}

	//Read ADI
	res = DIO_getStatus(DI_GROUP_B, &diData );
#if DEBUG_MODE == 1
	if( res != dio_command_succ)
		error();
#endif

	if( res == dio_command_succ)
	{
#if 0 //by JCNET
		MCCB.Status.BitVal.bDoor = diData.bit.PORT1;
		MCCB.Status.BitVal.bCB3 = diData.bit.PORT2;
		MCCB.Status.BitVal.bExMCCB1 = diData.bit.PORT3;
		MCCB.Status.BitVal.bExMCCB2 = diData.bit.PORT4;
		MCCB.Status.BitVal.bExMCCB3 = diData.bit.PORT5;
		MCCB.Status.BitVal.bSSWFUSE = diData.bit.PORT6;
		MCCB.Status.BitVal.bMCB7 = diData.bit.PORT7;
		MCCB.Status.BitVal.bRETEMP = diData.bit.PORT8;
#else
        MCCB.Status.BitVal.bSSWOH = diData.bit.PORT1;
        MCCB.Status.BitVal.bREV1 = diData.bit.PORT2;
        MCCB.Status.BitVal.bREV2 = diData.bit.PORT3;
        MCCB.Status.BitVal.bCB1 = diData.bit.PORT4;
        MCCB.Status.BitVal.bCB2 = diData.bit.PORT5;
        MCCB.Status.BitVal.bCB3 = diData.bit.PORT6;
        MCCB.Status.BitVal.bREV3 = diData.bit.PORT7;
        MCCB.Status.BitVal.bREV4 = diData.bit.PORT8;
        MCCB.Status.BitVal.bSSWOH = 1; //by JCNET for debugging
#endif
	}

#if 0 //by JCNET
	res = DIO_getStatus(DI_GROUP_C, &diData );
#if DEBUG_MODE == 1
	if( res != dio_command_succ)
		error();
#endif

	//-MCCB.Status.BitVal.bMC4 = Current_B.bit.PORT4;
	if( res == dio_command_succ)
	{
		MCCB.Status.BitVal.bTRTEMP = diData.bit.PORT1;
		MCCB.Status.BitVal.bCB4  = diData.bit.PORT2;
		MCCB.Status.BitVal.bMC8   = diData.bit.PORT3;
		MCCB.Status.BitVal.bMC9   = diData.bit.PORT4;
		MCCB.Status.BitVal.bSPD4   = diData.bit.PORT5;
		MCCB.Status.BitVal.bADI_REV14 = diData.bit.PORT6;
		MCCB.Status.BitVal.bADI_REV15 = diData.bit.PORT7;
		MCCB.Status.BitVal.bADI_REV16 = diData.bit.PORT8;
	}
#endif //by JCNET
#if DBUG_MODE == 3
EVENT:
#endif
	/*
	 * Event.
	 * MCU가 제어하는 스위치에 대해서만 기록을 남김.
	 * XXX TODO FIX --------------------------------------------
	 */
#if 0
	if( Prev.BitVal.bDS1 == OFF && MCCB.Status.BitVal.bDS1 == ON )
		EVT_Store(EVT_DS1_ON);
	else if(Prev.BitVal.bDS1 == ON && MCCB.Status.BitVal.bDS1 == OFF)
		EVT_Store(EVT_DS1_OFF);

	if( Prev.BitVal.bMC1A == OFF && MCCB.Status.BitVal.bMC1A == ON )
		EVT_Store(EVT_MC1A_ON);
	else if(Prev.BitVal.bMC1A == ON && MCCB.Status.BitVal.bMC1A == OFF)
		EVT_Store(EVT_MC1A_OFF);

	if( Prev.BitVal.bCB3 == OFF && MCCB.Status.BitVal.bCB3 == ON )
		EVT_Store(EVT_CB3_ON);
	else if(Prev.BitVal.bCB3 == ON && MCCB.Status.BitVal.bCB3 == OFF)
		EVT_Store(EVT_CB3_OFF);

	if( Prev.BitVal.bExMCCB1 == OFF && MCCB.Status.BitVal.bExMCCB1 == ON )
		EVT_Store(EVT_EX_MCCB1_ON);
	else if(Prev.BitVal.bExMCCB1 == ON && MCCB.Status.BitVal.bExMCCB1 == OFF)
		EVT_Store(EVT_EX_MCCB1_OFF);

	if( Prev.BitVal.bExMCCB2 == OFF && MCCB.Status.BitVal.bExMCCB2 == ON )
		EVT_Store(EVT_EX_MCCB2_ON);
	else if(Prev.BitVal.bExMCCB2 == ON && MCCB.Status.BitVal.bExMCCB2 == OFF)
		EVT_Store(EVT_EX_MCCB2_OFF);

	if( Prev.BitVal.bExMCCB3 == OFF && MCCB.Status.BitVal.bExMCCB3 == ON )
		EVT_Store(EVT_EX_MCCB3_ON);
	else if(Prev.BitVal.bExMCCB3 == ON && MCCB.Status.BitVal.bExMCCB3 == OFF)
		EVT_Store(EVT_EX_MCCB3_OFF);

	if( Prev.BitVal.bMCB7 == OFF && MCCB.Status.BitVal.bMCB7 == ON )
		EVT_Store(EVT_MCB7_ON);
	else if(Prev.BitVal.bMCB7 == ON && MCCB.Status.BitVal.bMCB7 == OFF)
		EVT_Store(EVT_MCB7_OFF);

	if( Prev.BitVal.bCB4 == OFF && MCCB.Status.BitVal.bCB4 == ON )
		EVT_Store(EVT_CB4_ON);
	else if(Prev.BitVal.bCB4 == ON && MCCB.Status.BitVal.bCB4 == OFF)
		EVT_Store(EVT_CB4_OFF);

#if STABLEEN_HILL_ENABLE != 1

    if( Prev.BitVal.bCB2 == OFF && MCCB.Status.BitVal.bCB2 == ON )
        EVT_Store(EVT_CB2_ON);
    else if(Prev.BitVal.bCB2 == ON && MCCB.Status.BitVal.bCB2 == OFF)
        EVT_Store(EVT_CB2_OFF);

    if( Prev.BitVal.bMC8 == OFF && MCCB.Status.BitVal.bMC8 == ON )
        EVT_Store(EVT_MCB8_ON);
    else if(Prev.BitVal.bMC8 == ON && MCCB.Status.BitVal.bMC8 == OFF)
        EVT_Store(EVT_MCB8_OFF);

	if( Prev.BitVal.bMC9 == OFF && MCCB.Status.BitVal.bMC9 == ON )
		EVT_Store(EVT_MCB9_ON);
	else if(Prev.BitVal.bMC9 == ON && MCCB.Status.BitVal.bMC9 == OFF)
		EVT_Store(EVT_MCB9_OFF);

#endif
#else

#define M_EVT_STORE(event,flag)  if( Prev.BitVal.b##event != flag && MCCB.Status.BitVal.b##event == flag ) \
    EVT_Store(EVT_##event##_##flag)
	M_EVT_STORE(MC1A,ON);
	M_EVT_STORE(MC1A,OFF);
	M_EVT_STORE(MC1,ON);
	M_EVT_STORE(MC1,OFF);
	M_EVT_STORE(MC2,ON);
	M_EVT_STORE(MC2,OFF);

#endif
//BY JCNET, SYS_DPWM_OFFTEMP 파라메터 초기값을 1로 설정해서 체크하지 않도록 !!!

#if 0 //by JCNET
	if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
	{
		// MC1A
		if ( MCCB.Command.BitVal.bMC1A_ONOFF == TRUE )
		{
			ODT_Initialize( odtMC1AOff ); // 사용하지않는 Off Delay Timer는 초기화
			if ( ODT_Update( odtMC1AOn, TRUE ) == ODT_FINISH )
			{
				if ( MCCB.Status.BitVal.bMC1A != ON )
					FLT_Raise(FLTH_MC1A);
			}
		}
		else // 운전명령 Off
		{
			ODT_Initialize( odtMC1AOn ); // 사용하지않는 On Delay Timer는 초기화
			if ( ODT_Update( odtMC1AOff, TRUE ) == ODT_FINISH )
			{
				if ( MCCB.Status.BitVal.bMC1A != OFF )
					FLT_Raise(FLTH_MC1A);
			}
		}

#if 0 //by JCNET
		// MC8
		if ( MCCB.Command.BitVal.bMC8_ONOFF == TRUE )
		{
			ODT_Initialize( odtMC8Off ); // 사용하지않는 Off Delay Timer는 초기화
			if ( ODT_Update( odtMC8On, TRUE ) == ODT_FINISH )
			{
				if ( MCCB.Status.BitVal.bMC8 != ON )
					FLT_Raise(FLTH_MC8);
			}
		}
		else // 운전명령 Off
		{
			ODT_Initialize( odtMC8On ); // 사용하지않는 On Delay Timer는 초기화
			if ( ODT_Update( odtMC8Off, TRUE ) == ODT_FINISH )
			{
				if ( MCCB.Status.BitVal.bMC8 != OFF )
					FLT_Raise(FLTH_MC8);
			}
		}

		// MC9
		if ( MCCB.Command.BitVal.bMC9_ONOFF == TRUE )
		{
			ODT_Initialize( odtMC9Off ); // 사용하지않는 Off Delay Timer는 초기화
			if ( ODT_Update( odtMC9On, TRUE ) == ODT_FINISH )
			{
				if ( MCCB.Status.BitVal.bMC9 != ON )
					FLT_Raise(FLTH_MC9);
			}
		}
		else // 운전명령 Off
		{
			ODT_Initialize( odtMC9On ); // 사용하지않는 On Delay Timer는 초기화
			if ( ODT_Update( odtMC9Off, TRUE ) == ODT_FINISH )
			{
				if ( MCCB.Status.BitVal.bMC9 != OFF )
					FLT_Raise(FLTH_MC9);
			}
		}
#endif
	}
#endif


	/*
	 * Update PrevStatus
	 */
	MCCB.PrevStatus.UnsVal = MCCB.Status.UnsVal;
	Prev.UnsVal = MCCB.Status.UnsVal;
}

/*
 * Call: c_int25(), FLT_CheckHW(), SYS_UpdateInverterStatus(), SYS_UpdateMcbStatus()
 */
public UInt32 MC_GetStatus(CBMC_STATUS ID)
{
	// 150105 16비트 이상에서 계산되지 않아 Int32로 변경.
	return (MCCB.Status.UnsVal & ((UInt32)1 << ID));
}

#if DBUG_MODE == 2 || DBUG_MODE == 3
private void MC_SetStatusForDebug(CBMC_CTRL ID, int OnOff)
{
	switch( ID )
	{
    case CTRL_DS1_ON:
            MCCB.Status.BitVal.bDS1 = ON;
        break;
    case CTRL_DS1_OFF:
    	if( INVERTER.uStatus <= 2 )
            MCCB.Status.BitVal.bDS1 = OFF;
        break;
    case CTRL_MC1A_ONOFF:
            MCCB.Status.BitVal.bMC1A = OnOff;
        break;
    case CTRL_CB4_ON:
            MCCB.Status.BitVal.bCB4 = ON;
        break;
    case CTRL_CB4_OFF:
            MCCB.Status.BitVal.bCB4 = OFF;
        break;
    case CTRL_MC8_ONOFF:
            MCCB.Status.BitVal.bMC8 = OnOff;
        break;
    case CTRL_MC9_ONOFF:
            MCCB.Status.BitVal.bMC9 = OnOff;
        break;
    case CTRL_CB3_ON:
            MCCB.Status.BitVal.bCB3 = ON;
        break;
    case CTRL_CB3_OFF:
    	if( INVERTER.uStatus <= 2 )
            MCCB.Status.BitVal.bCB3 = OFF;
        break;
	}

	MCCB_MIRROR.Status.UnsVal = MCCB.Status.UnsVal;
	return ;
}
#endif

#if DBUG_MODE == 3
int iRt  = 0;
#endif
/*
 * Call: FAN_UpdateDuty(), FLT_Raise(), MVP_Process20msec(), Process1sec(), MCB_StateTransition(),
 * 		 SYS_InverterStateTransition(), SYS_UpdateInverterStatus(), SYS_UpdateMcbStatus()
 *
 * 		 CB Control은 MCB 객체를 통하여 할 것!. 그러지 아니하면, MCB State가 오염될 수 있음.
 */
public Bool MC_UpdateStatus(CBMC_CTRL ID, Bool OpenClosed)
{
	Bool bRet = FALSE;

#if DBUG_MODE == 2 || DBUG_MODE == 3
	//Operating 가능 상태로 설정
	MCCB.Status.BitVal.bCB2 = MCCB.Status.BitVal.bSMPS = 1;
	MCCB.Status.BitVal.bEPO = MCCB.Status.BitVal.bRETEMP = 0; // 비접점식, 상태 반대.
	MC_SetStatusForDebug(ID, OpenClosed);

#if DBUG_MODE == 3
	if ( PRM_PCS[SYS_DPWM_OFFTEMP].iValue == 1 )
	{
//		if( ID == CTRL_MC4 )
//		{
//			MCCB.Command.BitVal.bMC4 = OpenClosed;
//			DIO_setStatus(DO_8,OpenClosed);
//		}
//		else if( ID == CTRL_CB3_STATE )
//		{
//			MCCB.Command.BitVal.bREV_ADO_1 = OpenClosed;
//
//			for( iRt = 0; iRt<3; iRt++)
//			{
//				// HILL 모드에서는 1회 Write해서는 DO 제어가 안됨.
//				DIO_setStatus(DO_7,OpenClosed);
//			}
//		}
	}
#endif

	return 1;
#endif

	switch(ID)
	{
    case CTRL_MC1A_ONOFF:
        MCCB.Command.BitVal.bMC1A_ONOFF = OpenClosed;
        bRet = DIO_setStatus(DO_5,OpenClosed);
        break;
//by JCNET
    case CTRL_MC1_ON:
    case CTRL_MC1_OFF:
//            MCCB.Command.BitVal.bMC1A_ONOFF = OpenClosed;
//            bRet = DIO_setStatus(DO_5,OpenClosed);
        break;
    case CTRL_MC2_ON:
    case CTRL_MC2_OFF:
//            MCCB.Command.BitVal.bMC1A_ONOFF = OpenClosed;
//            bRet = DIO_setStatus(DO_5,OpenClosed);
        break;
//-- JCNET
// MC1A, MC1, MC2 만 일단 처리.. 나머지는 나중에... by JCNET
#if 0 // by JCNET
		case CTRL_DS1_ON:
			MCCB.Command.BitVal.bDS1_ON = OpenClosed; // command 상태 저장
			bRet = DIO_setStatus(DO_1,OpenClosed);
			break;
		case CTRL_DS1_OFF:
			MCCB.Command.BitVal.bDS1_OFF = OpenClosed;
			bRet = DIO_setStatus(DO_2,OpenClosed);
			break;
		case CTRL_DS1_UVR:
			MCCB.Command.BitVal.bDS1_UVR = OpenClosed;
			bRet = DIO_setStatus(DO_3,OpenClosed);
			break;
		case CTRL_DDO_REV4:
			MCCB.Command.BitVal.bDDO_REV4 = OpenClosed; // command 상태 저장
			bRet = DIO_setStatus(DO_4,OpenClosed);
			break;

		case CTRL_SCFAN_CTRL:
			MCCB.Command.BitVal.bSCFAN_CTRL = OpenClosed;
			bRet = DIO_setStatus(DO_6,OpenClosed);
			break;
		case CTRL_CB4_OFF:
			MCCB.Command.BitVal.bCB4_OFF = OpenClosed;
			bRet = DIO_setStatus(DO_7,OpenClosed);
			break;
		case CTRL_CB4_ON:
			MCCB.Command.BitVal.bCB4_ON = OpenClosed;
			bRet = DIO_setStatus(DO_8,OpenClosed);
			break;
		case CTRL_MC8_ONOFF:
			MCCB.Command.BitVal.bMC8_ONOFF = OpenClosed;
			bRet = DIO_setStatus(DO_9,OpenClosed);
			break;
		case CTRL_MC9_ONOFF:
			MCCB.Command.BitVal.bMC9_ONOFF = OpenClosed;
			bRet = DIO_setStatus(DO_10,OpenClosed);
			break;
		case CTRL_CB3_UVR:
			MCCB.Command.BitVal.bCB3_UVR = OpenClosed;
			bRet = DIO_setStatus(DO_11,OpenClosed);
			break;
		case CTRL_CB3_OFF:
			MCCB.Command.BitVal.bCB3_OFF = OpenClosed;
			bRet = DIO_setStatus(DO_12,OpenClosed);
			break;
		case CTRL_CB3_ON:
			MCCB.Command.BitVal.bCB3_ON = OpenClosed;
			bRet = DIO_setStatus(DO_13,OpenClosed);
			break;
		case CTRL_SSEFAN_CTRL:
			MCCB.Command.BitVal.bSSEFAN_CTRL = OpenClosed;
			bRet = DIO_setStatus(DO_14,OpenClosed);
			break;
		case CTRL_REV4:
			MCCB.Command.BitVal.bREV_2 = OpenClosed;
			bRet = DIO_setStatus(DO_15,OpenClosed);
			break;
		case CTRL_REV5:
			MCCB.Command.BitVal.bREV_3 = OpenClosed;
			bRet = DIO_setStatus(DO_16,OpenClosed);
			break;
#endif
	}

	return bRet;
}

/*
 * Call: ResDIOSET()
 */
public void MC_UpdateStatusForTest(CBMC_CTRL ID, Bool OpenClosed)
{
	MC_UpdateStatus(ID,OpenClosed);
}

#if 0 //[CO] by JCNET
void MCB_Abnormal_Operation_Check(void)
{
	switch(INVERTER.uStatus)
	{
		case SYS_INV_FAULT:
		{
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			{
				if(MC_GetStatus(STATUS_DS1))
					MCB_UpdateCmd(M_CB1_BATT, CB_CMD_OFF);
				if(MC_GetStatus(STATUS_CB3))
					MCB_UpdateCmd(M_CB3, CB_CMD_OFF);
			}
			else
			{
				if(MC_GetStatus(STATUS_DS1))
					MC_UpdateStatus(CTRL_DS1_ON, OPEN);
				if(MC_GetStatus(STATUS_CB3))
					MC_UpdateStatus(CTRL_CB3_ON, OPEN);
			}

//JCNET			if(MC_GetStatus(STATUS_MC1A2A))
				MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN);
//++ JCNET
			MC_UpdateStatus(CTRL_MC1_ONOFF, OPEN);
			MC_UpdateStatus(CTRL_MC2_ONOFF, CLOSED);
			GPIO_StaticSwitch(CLOSED);
//-- JCNET

			if(MC_GetStatus(STATUS_MC8))
				MC_UpdateStatus(CTRL_MC8_ONOFF, OPEN);
			if(MC_GetStatus(STATUS_MC9))
				MC_UpdateStatus(CTRL_MC9_ONOFF, OPEN);
		}
			break;
		case SYS_INV_STOP:
		{
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			{
				if(MC_GetStatus(STATUS_DS1))
					MCB_UpdateCmd(M_CB1_BATT, CB_CMD_OFF);
				if(MC_GetStatus(STATUS_CB3))
					MCB_UpdateCmd(M_CB3, CB_CMD_OFF);
			}
			else
			{
				if(MC_GetStatus(STATUS_DS1))
					MC_UpdateStatus(CTRL_DS1_ON, OPEN);
				if(MC_GetStatus(STATUS_CB3))
					MC_UpdateStatus(CTRL_CB3_ON, OPEN);
			}

#if 0 //by JCNET
			if(MC_GetStatus(STATUS_MC1A2A))
#endif
				MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN);
//++ JCNET
            MC_UpdateStatus(CTRL_MC1_ONOFF, OPEN);
            MC_UpdateStatus(CTRL_MC2_ONOFF, CLOSED);
            GPIO_StaticSwitch(CLOSED);
//-- JCNET
			if(MC_GetStatus(STATUS_MC8))
				MC_UpdateStatus(CTRL_MC8_ONOFF, OPEN);
			if(MC_GetStatus(STATUS_MC9))
				MC_UpdateStatus(CTRL_MC9_ONOFF, OPEN);
#if USE_MBP_PANNEL == 1
			if(!MC_GetStatus(STATUS_EX_MCCB1))
				FLT_Raise(FLTL_CB5);
			if(!MC_GetStatus(STATUS_EX_MCCB2))
				FLT_Raise(FLTL_CB6);
			if(MC_GetStatus(STATUS_EX_MCCB3))
				FLT_Raise(FLTL_CB7);
#endif
		}
			break;
		case SYS_INV_SCR_ON:
		{
			if(MC_GetStatus(STATUS_DS1))
				FLT_Raise(FLTH_BATT_CB1);
			if(MC_GetStatus(STATUS_CB3))
				FLT_Raise(FLTH_GRID_CB3);

			if(MC_GetStatus(STATUS_CB4))
			{
				if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
				{
					MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);
				}
				else
				{
					MC_UpdateStatus(CTRL_CB4_ON, OPEN);
				}
			}

			if(MC_GetStatus(STATUS_MC1A2A))
				FLT_Raise(FLTH_MC1A);
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			{
#if EVE_DC_CHARGER == 1
				if(MC_GetStatus(STATUS_MC8))
					FLT_Raise(FLTH_MC8);
				if(MC_GetStatus(STATUS_MC9))
					FLT_Raise(FLTH_MC9);
#endif
			}
#if USE_MBP_PANNEL == 1
			if(!MC_GetStatus(STATUS_EX_MCCB1))
				FLT_Raise(FLTH_CB5);
			if(!MC_GetStatus(STATUS_EX_MCCB2))
				FLT_Raise(FLTH_CB6);
			if(MC_GetStatus(STATUS_EX_MCCB3))
				FLT_Raise(FLTH_CB7);
#endif
		}
			break;
		case SYS_INV_DC_CHARGE:
		{
			if(MC_GetStatus(STATUS_CB3))
				FLT_Raise(FLTH_GRID_CB3);
			if(MC_GetStatus(STATUS_CB4))
				FLT_Raise(FLTL_CB4);
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			{
#if EVE_DC_CHARGER == 1
				if(MC_GetStatus(STATUS_MC8))
					FLT_Raise(FLTH_MC8);
				if(MC_GetStatus(STATUS_MC9))
					FLT_Raise(FLTH_MC9);
#endif
			}
#if USE_MBP_PANNEL == 1
			if(!MC_GetStatus(STATUS_EX_MCCB1))
				FLT_Raise(FLTH_CB5);
			if(!MC_GetStatus(STATUS_EX_MCCB2))
				FLT_Raise(FLTH_CB6);
			if(MC_GetStatus(STATUS_EX_MCCB3))
				FLT_Raise(FLTH_CB7);
#endif
		}
			break;
		case SYS_INV_EVE_DC_CHARGE:
		{
			if(!MC_GetStatus(STATUS_DS1))
				FLT_Raise(FLTH_BATT_CB1);
			if(MC_GetStatus(STATUS_CB3))
				FLT_Raise(FLTH_GRID_CB3);
			if(MC_GetStatus(STATUS_CB4))
				FLT_Raise(FLTL_CB4);
			if(MC_GetStatus(STATUS_MC1A2A))
				FLT_Raise(FLTH_MC1A);
#if USE_MBP_PANNEL == 1
			if(!MC_GetStatus(STATUS_EX_MCCB1))
				FLT_Raise(FLTH_CB5);
			if(!MC_GetStatus(STATUS_EX_MCCB2))
				FLT_Raise(FLTH_CB6);
			if(MC_GetStatus(STATUS_EX_MCCB3))
				FLT_Raise(FLTH_CB7);
#endif
		}
			break;
		case SYS_INV_AC_GENERATE:
		{
			if(!MC_GetStatus(STATUS_DS1))
				FLT_Raise(FLTH_BATT_CB1);
			if(MC_GetStatus(STATUS_CB3))
				FLT_Raise(FLTH_GRID_CB3);
			if(MC_GetStatus(STATUS_CB4))
				FLT_Raise(FLTL_CB4);
			if(MC_GetStatus(STATUS_MC1A2A))
				FLT_Raise(FLTH_MC1A);
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			{
#if EVE_DC_CHARGER == 1
				//if(!MC_GetStatus(STATUS_MC8))
				//	FLT_Raise(FLTH_MC8);
				if(MC_GetStatus(STATUS_MC9))
					FLT_Raise(FLTH_MC9);
#endif
			}
#if USE_MBP_PANNEL == 1
			if(!MC_GetStatus(STATUS_EX_MCCB1))
				FLT_Raise(FLTH_CB5);
			if(!MC_GetStatus(STATUS_EX_MCCB2))
				FLT_Raise(FLTH_CB6);
			if(MC_GetStatus(STATUS_EX_MCCB3))
				FLT_Raise(FLTH_CB7);
#endif
		}
			break;
		case SYS_INV_START_SYNC:
		{
			if(!MC_GetStatus(STATUS_DS1))
				FLT_Raise(FLTH_BATT_CB1);
			if(MC_GetStatus(STATUS_CB4))
				FLT_Raise(FLTL_CB4);
			if(MC_GetStatus(STATUS_MC1A2A))
				FLT_Raise(FLTH_MC1A);
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			{
#if EVE_DC_CHARGER == 1
				//if(!MC_GetStatus(STATUS_MC8))
				//	FLT_Raise(FLTH_MC8);
				if(MC_GetStatus(STATUS_MC9))
					FLT_Raise(FLTH_MC9);
#endif
			}
#if USE_MBP_PANNEL == 1
			if(!MC_GetStatus(STATUS_EX_MCCB1))
				FLT_Raise(FLTH_CB5);
			if(!MC_GetStatus(STATUS_EX_MCCB2))
				FLT_Raise(FLTH_CB6);
			if(MC_GetStatus(STATUS_EX_MCCB3))
				FLT_Raise(FLTH_CB7);
#endif
		}
			break;
		case SYS_INV_RUN:
		{
			if(!MC_GetStatus(STATUS_DS1))
				FLT_Raise(FLTH_BATT_CB1);
			if(!MC_GetStatus(STATUS_CB3))
				FLT_Raise(FLTH_GRID_CB3);
			if(MC_GetStatus(STATUS_CB4))
				FLT_Raise(FLTL_CB4);
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			{
				if(MC_GetStatus(STATUS_MC1A2A))
					FLT_Raise(FLTH_MC1A);
#if EVE_DC_CHARGER == 1
				//if(!MC_GetStatus(STATUS_MC8))
				//	FLT_Raise(FLTH_MC8);
				if(MC_GetStatus(STATUS_MC9))
					FLT_Raise(FLTH_MC9);
#endif
			}
#if USE_MBP_PANNEL == 1
			if(!MC_GetStatus(STATUS_EX_MCCB1))
				FLT_Raise(FLTH_CB5);
			if(!MC_GetStatus(STATUS_EX_MCCB2))
				FLT_Raise(FLTH_CB6);
			if(MC_GetStatus(STATUS_EX_MCCB3))
				FLT_Raise(FLTH_CB7);
#endif
		}
			break;
		case SYS_INV_ISLANDING:
		{
			if(!MC_GetStatus(STATUS_DS1))
				FLT_Raise(FLTH_BATT_CB1);
			if(!MC_GetStatus(STATUS_CB3))
				FLT_Raise(FLTH_GRID_CB3);
			if(MC_GetStatus(STATUS_CB4))
				FLT_Raise(FLTL_CB4);
			if(MC_GetStatus(STATUS_MC1A2A))
				FLT_Raise(FLTH_MC1A);
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			{
#if EVE_DC_CHARGER == 1
				//if(!MC_GetStatus(STATUS_MC8))
				//	FLT_Raise(FLTH_MC8);
				if(MC_GetStatus(STATUS_MC9))
					FLT_Raise(FLTH_MC9);
#endif
			}
#if USE_MBP_PANNEL == 1
			if(!MC_GetStatus(STATUS_EX_MCCB1))
				FLT_Raise(FLTH_CB5);
			if(!MC_GetStatus(STATUS_EX_MCCB2))
				FLT_Raise(FLTH_CB6);
			if(MC_GetStatus(STATUS_EX_MCCB3))
				FLT_Raise(FLTH_CB7);
#endif
		}
			break;
		case SYS_INV_RE_SYNC:
		{
			if(!MC_GetStatus(STATUS_DS1))
				FLT_Raise(FLTH_BATT_CB1);
			if(!MC_GetStatus(STATUS_CB3))
				FLT_Raise(FLTH_GRID_CB3);
			if(MC_GetStatus(STATUS_CB4))
				FLT_Raise(FLTL_CB4);
			if(MC_GetStatus(STATUS_MC1A2A))
				FLT_Raise(FLTH_MC1A);
			if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
			{
#if EVE_DC_CHARGER == 1
				//if(!MC_GetStatus(STATUS_MC8))
				//	FLT_Raise(FLTH_MC8);
				if(MC_GetStatus(STATUS_MC9))
					FLT_Raise(FLTH_MC9);
#endif
			}
#if USE_MBP_PANNEL == 1
			if(!MC_GetStatus(STATUS_EX_MCCB1))
				FLT_Raise(FLTH_CB5);
			if(!MC_GetStatus(STATUS_EX_MCCB2))
				FLT_Raise(FLTH_CB6);
			if(MC_GetStatus(STATUS_EX_MCCB3))
				FLT_Raise(FLTH_CB7);
#endif
		}
			break;
		case SYS_INV_BYP_EVT_OPERATION:
			break;
		case SYS_INV_TEST_MODE:
			break;
	}
}
#endif

