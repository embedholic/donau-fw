/*
 * Tasks.c
 *
 *  Created on: 2012. 11. 5.
 *      Author: destinPower
 */

#include "Task.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>
#include <string.h>

#include "DSP28x_Project.h"
#include "I2C.h"
#include "SCI.h"
#include "SPIa.h"
#include "CIRC.h"
#include "prm_pcs.h"
#include "RtcTime.h"
#include "CC.h"
#include "Fault.h"
#include "MVP.h"
#include "Event.h"
#include "Rms.h"
#include "VariablePeriod.h"

#include "SYSTEM.h" // call c_int25
#include "MVP.h" 	// call MVP_Process20msec
#include "FirmwareWriter.h"
#include "Version.h"
#include "History.h"
#include "Modbus232.h"
#include "CAN.h"
#include "CAN_GBI.h"
#include "History.h"
#include "DBUG.h"
#include "DIO.h"
//-#include "COSPHIControl.h"
#if DEBUG_MODE == 1
#include "DBUG.h"
#include "EADC.h"
#include "EPWM.h"
#endif
#include "NvSRAM.h"
#include "MO.h"
#include "CTRL_GEN.h"
#include "CTRL_INV_DROOP.h"
#include "EVE_DC_CHARGER.h"
#include "TRACE.h"
#include "FAN.h"

Uns uIgbtFaultClearSecond = 0;
//typedef struct _NvSRAM_BUF
//{
//	int addr;
//	int byte;
//}NVBUFS;
//NVBUFS NVBUF[100];
//int NVBUF_STX = -1;
//
//public void nvAdd(int addr, int byte)
//{
//	if( NVBUF_STX >= 99)
//	{
//		error();
//	}
//	NVBUF[NVBUF_STX+1].addr =addr;
//	NVBUF[NVBUF_STX++].byte =byte;
//}
//void nvProcessing()
//{
//	if( NVBUF_STX < 0 )
//		return;
//
//	MCBSP_SPI_byteWrite(NVBUF[NVBUF_STX].addr,NVBUF[NVBUF_STX--]);
//}

REQ_CMD_IDLE ReqCmdIdle;

private Void Process1sec(Void);
private Void Process100msec(Void);
private void InitWelcomMsg();
private Bool CheckSoftwareUpdate();
private Bool SendMessageToSerial(char* string );
/*
 *  ======== task_RxTx ========
 *  통신 데이터 송수신 Task
 *  Priority: 6
 */
extern volatile uint32_t my_flag;
Void _task_RxTx(void)
{
	char ch1;
	//-Uint16 i = 0;

#if DEBUG_MODE == 1
    System_printf("enter _task_RxTx()\n");
    System_flush();
#endif

    while(bUpdateFlag == TRUE)
    {
    	Task_sleep(100);
    }

#if USE_SCI_MODE == 1
#define SCI_REGS SciaRegs
#else
#define SCI_REGS ScibRegs
#endif

    while(TRUE)
    {

    	Task_sleep(1);

	   if(SCI_haveTxBufferChar(SCI_RS232))
	   {
			if(SCI_txReady(SCI_RS232))
			{
				SCI_getTxBufferChar(SCI_RS232, &ch1);
				SCI_sendChar(SCI_RS232,ch1);
			}
	   }

		/*
		 * For fixing error of connect different baudrate in RS232
		 */
		if(SCI_REGS.SCIRXST.bit.RXERROR!=0)
		{
			SCI_REGS.SCICTL1.bit.SWRESET=0;
			// Just a long delay loop
			//for (i = 0; i < 1000; i++) {}
			Task_sleep(100);

			SCI_create(SCI_RS232, SCI_BAUD,0);  // Init SCI-A
		}
    }

}

/*
 *  ======== task_MsgCore ========
 *  통신 데이터 처리
 *  Priority: 6
 */
Void _task_MsgCore(void)
{
#if DEBUG_MODE == 1
    System_printf("enter _task_MsgCore()\n");
    System_flush();
#endif

    //-SPIA_LoopBackTest();

	/* Check for update */
	InitWelcomMsg();

	if( CheckSoftwareUpdate() )
	{

		FW_UpdateViaSCI();
	}
	else
	{

		// Boot Flash disable. Data Flash Enable
		SPIA_dataFlash_ENB();
	}

	/*
	 * System Started!
	 */
	bSystemStarted = TRUE;

	FLT_Initialize();

	EPwm4Regs.ETSEL.bit.INTEN = 1;			/* Enable INT */

    bUpdateFlag = FALSE;

	Modbus232_Create();
	Modbus232_Start();//+


    while(1){
    	Modbus232_Process();
    }

}

/*
 *  ======== _task_CAN ========
 *  CAN 통신 데이터 처리
 *  Priority: 6
 */
Void _task_CAN(void)
{
	while(1){
		CAN_proc();
		CAN_GBI_proc();

		Task_sleep(10);
	}
}

#if DEBUG_MODE == DEBUG_DEVELOP
int traceBuffer[2][256];
#endif

/*
 *  ======== idleLoop ========
 *
 *  주의: Idle task에는 세마포어_pend() 또는 Task_sleep()을 넣으면 시스템이 종료되는 원인이 될 수 있음
 */

//int ffff = 7500;
//int gggg = 0;
//#include "MCB.h"
Void _idleLoop(void)
{
	memset(&ReqCmdIdle, 0, sizeof(ReqCmdIdle));

#if DEBUG_MODE == DEBUG_DEVELOP
	ERTM;   // Enable Global realtime interrupt DBGM
	TEST_Create();
#else

#endif

	while(1)
	{
		HISTORY_WriteToNvSRAM();

		if(!bSystemStarted)
			continue;

		if( ReqCmdIdle.bReqSetCmd )
		{
			ReqCmdIdle.bReqSetCmd = FALSE;
			if( ReqCmdIdle.SetCmdVal )
				NvSRAM_SetRunCommand(ON);
			else
				NvSRAM_SetRunCommand(OFF);
			
		}

		PARAM_UpdateAll();

		RMS_AllCalculate();

		#if PEAK_MODULE_ENB
		PEAK_OffsetCalc();
		#endif

		CVC_UpdateConEPeakInverse();

		Process100msec();
		Process1sec();
	}
}

/*
 *  ======== Periodic Timer for SYSTEM  ========
 *  Period: 2.5 ms
 *  Checked Period? : YES
 *
 */
Void _prd_System_2d5ms(void)
{
	static int cnt10ms = 0;
	//LED
	//-TEST_LedToggle(4);
	c_int25();

	cnt10ms++;
	if( cnt10ms == 4 )
	{
		cnt10ms = 0;
		MVP_Process10msec();
	}

}

/*
 *  ======== Periodic Timer for MVP ========
 *  Period: 20ms
 *  Checked Period? : YES
 */
Void _prd_MVP_20ms(void)
{
	//LED
	//-TEST_LedToggle(5);

	MVP_Process20msec();
}


private Void Process100msec(Void)
{
#if DEBUG_MODE == 1
	static Uns count = 0;
#endif

	if ( MVP_Get100msecTick() == FALSE ) return;

//	if ( PRM_PCS[CTRL_CC_PERIOD].iValue == 0 )
//		VPRD_UpdateReference();

	MVP_Clear100msecTick();

	CTRL_INV_DROOP_100ms_func();

	if(INVERTER.uStatus == SYS_INV_ISLANDING)
	{
		GenBlock.holdingLoadP = CC_GetActualPowerP_2nd();
		GenBlock.holdingLoadQ = CC_GetActualPowerQ_2nd();
	}

#if COSPHI_ENB == 1
	COSPHI_Run();
#endif

#if DEBUG_MODE == 1
	count++;
	if( (count % 10) == 0 )
	{
		//-System_printf("100 ms processing... 10count [ %d ]\n",count);
    	//-System_flush();
	}
	return;
#endif

}

#if DEBUG_MODE == DEBUG_DEVELOP
static int iPtemp = 0;
#endif
extern unsigned int iTraceAutoRstCnt;

UInt16 iCabTemp = 25;
private Void Process1sec(Void)
{
	static Uns u5Second = 0;
	static Uns u1min = 0;
	static Uns uInit10Second = 0;
	UInt16 retValue = 0;

#if DEBUG_MODE == 1
	static Uns count = 0;
#endif

	if ( MVP_GetOneSencondTick() == FALSE ) return;
	//-150212
	TEST_LedToggle(1);

#if 0
	if( iTraceAutoRstCnt++ >= 300 )
	{
		if( TRACE.iStop == 1 )
		{
			TRC_TraceReset();
		}
	}
#endif
	CTRL_INV_DROOP_1Sec_func();

	if( INVERTER.gridReconnect.bEnb )
		INVERTER.gridReconnect.uiProgressionSecond++;
	else
		INVERTER.gridReconnect.uiProgressionSecond = 0;

	if( INVERTER.mc4On_timer_start )
	{
		INVERTER.bRemove_Z_TR = ON;

		if( INVERTER.mc4On_timer++ >= 5 )
		{
			INVERTER.bRemove_Z_TR = OFF;
			 INVERTER.mc4On_timer_start = OFF;
		}
	}
	else
	{
		INVERTER.mc4On_timer = 0;
		INVERTER.bRemove_Z_TR = OFF;
	}
	//-151021 if( PRM_PCS[BYP_MODE].iValue == 20 )
	{
		if( INVERTER.uStatus == SYS_INV_RUN )
		{
			GenBlock.timer_hold_p_sec++;
			GenBlock.timer_decrease_p_sec++;
		}
		else
		{
			GenBlock.timer_hold_p_sec = 0;
			GenBlock.timer_decrease_p_sec = 0;
		}
	}

#if EVE_DC_CHARGER == 1
    EVE_CAN_Communication_TxChecking();
    EVE_CAN_Communication_RxChecking();
#endif

	/* RTC ----------------------------------------------------*/
	RTCTIME_Update();

#if DEBUG_MODE == DEBUG_DEVELOP
	extern TimePack TPackShadow;
	System_printf("TIME %d/%d/%d %d %d %d \n", TPackShadow.ByteVal.By3 >> 2,TPackShadow.BitVal.Month,TPackShadow.BitVal.Day,TPackShadow.BitVal.Hour,TPackShadow.BitVal.Minute,TPackShadow.BitVal.Second);
#endif

	/* TEMP ----------------------------------------------------*/
	if( I2C_GET_Temp( &retValue ) )
	{
#if DEBUG_MODE == DEBUG_DEVELOP
		System_printf("REQ SUCC. Temp : %d \n", retValue);
#endif
		iCabTemp = retValue;
	}
	else
	{
#if DEBUG_MODE == DEBUG_DEVELOP
		System_printf("REQ FAIL. Temp : %d \n", retValue);
#endif
	}

#if DBUG_MODE == 2
	System_flush();
#endif

	/* FR-BMS ----------------------------------------------------*/
	/* 1Sec Task.  */
	if( ++BATCTRL.bmsInfo.timeOutCnt_sec >= 10 )
	{
		BATCTRL.bmsInfo.timeOutCnt_sec++;
		 BATCTRL.bmsInfo.bActive = FALSE;
	}

	PRM_PCS[BATT_6].iValue = BATCTRL.bmsInfo.timeOutCnt_sec;

	// 1sec fan do ctrl

	FAN_Control_heatsink();
	if( ++uIgbtFaultClearSecond <= 15 && bSystemStarted && FLT_GetEachStatus(FLTH_PEBB_IGBT) )
	{
		FLT_Clear(FLTH_PEBB_IGBT);
		MO_PebbFaultReset();
	}
	else if( uIgbtFaultClearSecond > 15)
	{
		uIgbtFaultClearSecond = 16;
	}
	if ( ++u5Second >= 5 )
	{
		u5Second = 0;

		/*
		 * 1분 1회 파라미터 저장
		 */
		if( ++u1min >= 12 )
		{
			PARA_StoreToMemoryForTask(WH_CHG_HI, PRM_PCS[WH_CHG_HI].iValue);
			PARA_StoreToMemoryForTask(WH_CHG_LO, PRM_PCS[WH_CHG_LO].iValue);
			PARA_StoreToMemoryForTask(WH_DCHG_HI, PRM_PCS[WH_DCHG_HI].iValue);
			PARA_StoreToMemoryForTask(WH_DCHG_LO, PRM_PCS[WH_DCHG_LO].iValue);
			u1min = 0;
		}
	}

	FLT_CabinetTemp();

	FLT_UpdateAutoResetState();

	if ( FLT_GetHeavyStatus() )
		FLT_IncreseClearDelayCnt();
	else
		FLT_ResetClearDelayCnt();

	FLT_IncreseInitDelayCnt();

	if ( PRM_PCS[CTRL_TEST_MODE].iValue && INVERTER.uStatus == SYS_INV_TEST_MODE )
		FLT_Raise(FLTL_TEST_RUN);
	else
		FLT_Clear(FLTL_TEST_RUN);

	MVP_ClearOneSencondTick();

	if ( uInit10Second < 10 )
	{
		++uInit10Second;
		if (uInit10Second==10)
		{
			EVT_Store(EVT_BOOT_OK);
			SYS_CheckNVSRAMRun();
		}
	}

	// CAN DEBUG
	//-PRM_PCS[SYS_REV_4].iValue = ECanaRegs.CANES.all >> 16 &0xFFFF;
	//-PRM_PCS[SYS_REV_5].iValue = ECanaRegs.CANES.all & 0xFFFF;

#if DEBUG_MODE == DEBUG_DEVELOP
	extern unsigned int iCallCount;
	iPtemp++;
	if( iPtemp == 10)
	{
		iPtemp = 0;
		System_printf("10 Sec processing... [ %d ]\n",count++);
		System_printf("cc count [ %d ]\n",iCallCount);
		iCallCount = 0;
		System_flush();
	}
    return;
#endif
}



/*
 * Program Update Check.
 */

/*
 * 주의: 문자열 뒤에 '\0' 을 넣어 안전 확보.
 */
private Bool SendMessageToSerial(char* string )
{
	SCI_writeString(SCI_RS232, string, strlen(string));
	SendFlush();
	return TRUE;
}


private void InitWelcomMsg()
{
	int i = 0;

	SendMessageToSerial("\r\n=========================\r\n");
	SendMessageToSerial("DestinPower LGS Software\r\n");
	SendMessageToSerial("Version :");
	SendMessageToSerial(VER_getString());
	SendMessageToSerial("\r\n=========================\r\n");
	SendMessageToSerial("PRESS BACK-SPACE KEY TO DOWNLOAD:1\r\n");

	for(i = 0; i<10; i++)
	{
		SendMessageToSerial(".");
		Task_sleep(300);
	}
	SendMessageToSerial("\r\n");
}

private Bool CheckSoftwareUpdate()
{
	//PWM_Disable
	char ch = 0;
	int i = 0;

	for(i = 0; i<3; i++)
	{
		ch = 0;
		if(SCI_read(SCI_RS232, &ch) == FALSE)
		{
			return FALSE;
		}
		if( ch != 0x8 )
			return FALSE;
	}

	SendMessageToSerial("READY TO DOWNLOAD\r\n");

	return TRUE;
}




