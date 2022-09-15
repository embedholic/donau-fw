/*
 * DIO.c
 *
 *  Created on: 2013. 1. 23.
 *      Author: destinPower
 */


#include "DIO.h"
#include "SPId.h"
#include "DSP28x_Project.h"						// Device Headerfile and Examples Include File
#include <xdc/runtime/System.h>
/*
 * DIO 이상 유무 확인 : DIO_DIAG()
 */

typedef struct _DITIGAL
{
	DI_STATE Port;
	DI_STATE dummy;
	void (*pCS_Set)(); // Chip Select Enable function pointer
	void (*pCS_Clr)(); // Chip Select Disable function pointer
}DIGITAL_CTRL;

static int bSPI_D_working = FALSE;

static DIGITAL_CTRL DI[DI_ISO_END];
static DIGITAL_CTRL DO[DO_ISO_END];

private void DIO_Item_Init(DIGITAL_CTRL* const dioObj, Uint16 chipId, void (*pCsSetFunc)(), void (*pCsClrFunc)());
private Bool DIO_DO_Clear(DIGITAL_CTRL* const isoObj);
private Bool DIO_DI_Load(DIGITAL_CTRL* isoObj);
private DIO_CTRL_STATUS DIO_output(const DIGITAL_CTRL* const doObj);
private DIO_CTRL_STATUS DIO_input(DIGITAL_CTRL* diObj);


private void DIO_Item_Init(DIGITAL_CTRL* const dioObj, Uint16 chipId, void (*pCsSetFunc)(), void (*pCsClrFunc)())
{
	dioObj[chipId].Port.all = 0;
	dioObj[chipId].pCS_Clr = pCsClrFunc;
	dioObj[chipId].pCS_Set = pCsSetFunc;
}
private Bool DIO_DO_Clear(DIGITAL_CTRL* const isoObj)
{
	Uint16 failSafeCount = 0;
	DIO_CTRL_STATUS dioRes = dio_none;

	isoObj->Port.all = 0x00;
	do
	{
		dioRes = DIO_output(isoObj);
		if(dioRes == dio_command_succ)
		{
			return TRUE;
		}
		DELAY_US(1);
	}while(failSafeCount++ <= 3);

	return FALSE;
}

/*
 * DI 상태 값을 메모리에 저장한다.
 */
private Bool DIO_DI_Load(DIGITAL_CTRL* isoObj)
{
	Uint16 failSafeCount = 0;
	DIO_CTRL_STATUS dioRes = dio_none;

	do
	{
		dioRes = DIO_input(isoObj);
		if(dioRes == dio_command_succ)
		{
			return TRUE;
		}
		DELAY_US(1);
	}while(failSafeCount++ <= 3);

	return FALSE;
}
/*
 * 함수 수행 시간(다른 인터럽트에 간섭 받지 않았을 경우) : 약 150ns + 8bit 전송시간
 */
private DIO_CTRL_STATUS DIO_output(const DIGITAL_CTRL* const doObj)
{
	volatile int dummy, failSafeCount=0;

	if( bSPI_D_working )
	{
		//-error();
#if DEBUG_MODE == DEBUG_DEVELOP
				System_printf("\n\n SET_dio_device_busy 1");
#endif
		return dio_device_busy;

	}

	// Chip Select가 이미 되어 있는 것이 있는지 확인하여 다른 작업 중일 경우 리턴.
	if( DIO_CS_STATE() == 1)
	{
		//-error();
#if DEBUG_MODE == DEBUG_DEVELOP
				System_printf("\n\n SET_dio_device_busy 2");
#endif
		return dio_device_busy;
	}

	/*=================== D0 ===================*/
	//-SpidRegs.SPICCR.bit.SPISWRESET=0;				// Reset SPI
	SpidRegs.SPICCR.bit.CLKPOLARITY = 1;			//
	//-SpidRegs.SPICTL.bit.CLK_PHASE  = 0;				// with Delay
	//-SpidRegs.SPICCR.bit.SPISWRESET=1;
	//-DELAY_US(1);

	bSPI_D_working = TRUE;

	if(doObj == 0)
	{
		//-error();
#if DEBUG_MODE == DEBUG_DEVELOP
				System_printf("\n\n SET_dio_unknown_device");
#endif
		bSPI_D_working = FALSE;
		return dio_unknown_device;
	}

	if(doObj->pCS_Set != 0 || doObj->pCS_Clr != 0)
	{
		doObj->pCS_Set();
	}
	else
	{
		bSPI_D_working = FALSE; // Added. 130515@
#if DEBUG_MODE == DEBUG_DEVELOP
				System_printf("\n\n SET_dio_dio_device_busy 3");
#endif
		return dio_device_busy;
	}

	DELAY_NS(30);
	SpidRegs.SPIDAT = ((doObj->Port.all)<<8)& 0xFF00; // 5Mhz
	while(!SpidRegs.SPISTS.bit.INT_FLAG); // FIXME while문으로 남겨둘 것인지. counting 할 것인지, while 없이 일단 cs_clear 해도 되는지.
										  // counting으로 할 경우에는 기존 상태가 동일할 경우 write하지 않고 return 하는 시퀀스는 삭제되어야 한다.
	DELAY_NS(30); // b:10
	// reset int flag
	dummy = SpidRegs.SPIRXBUF;

	doObj->pCS_Clr();

	if(SpidRegs.SPISTS.bit.OVERRUN_FLAG)
	{
#if DEBUG_MODE == DEBUG_DEVELOP
				System_printf("\n\n SET_dio_OVERRUN_FLAG ");
#endif
		error();
	}
	DELAY_NS(100);
	bSPI_D_working = FALSE;

	return dio_command_succ;
}

/*
 * 함수 수행 시간(다른 인터럽트에 간섭 받지 않았을 경우) : 약 8.17us + 8bit 전송시간
 */
private DIO_CTRL_STATUS DIO_input(DIGITAL_CTRL* diObj)
{
	volatile int failSafeCount=0,temp,i;

	if( bSPI_D_working )
	{
		return dio_device_busy;
	}

	// Chip Select가 이미 되어 있는 것이 있는지 확인하여 다른 작업 중일 경우 리턴.
	if( DIO_CS_STATE() == 1)
	{
#if DEBUG_MODE == DEBUG_DEVELOP
		error();
		System_printf("\n\n SET_dio_OVERRUN_FLAG ");
#endif
		return dio_device_busy;
	}

	/*=================== DI ===================*/
	//-SpidRegs.SPICCR.bit.SPISWRESET=0;				// Reset SPI
	SpidRegs.SPICCR.bit.CLKPOLARITY = 0;			//
	//-SpidRegs.SPICTL.bit.CLK_PHASE  = 0;				// with Delay
	//-SpidRegs.SPICCR.bit.SPISWRESET=1;

	//-------------------------->
	bSPI_D_working = TRUE;

	DELAY_US(5);

	if(diObj == 0){
		error();
		bSPI_D_working = FALSE; // Added. 130515@

#if DEBUG_MODE == DEBUG_DEVELOP
		System_printf("\n\n dio_unknown_device 1 ");
#endif
		return dio_unknown_device;
	}

	if(diObj->pCS_Set != 0 || diObj->pCS_Clr != 0)
	{
		diObj->pCS_Set();
	}
	else
	{
		bSPI_D_working = FALSE; // Added. 130515@
#if DEBUG_MODE == DEBUG_DEVELOP
		System_printf("\n\n dio_unknown_device  2 ");
#endif
		return dio_unknown_device;
	}

	if(SpidRegs.SPISTS.bit.INT_FLAG)
	{
#if DEBUG_MODE == DEBUG_DEVELOP
		error();
		System_printf("\n\n dio_unknown_device  2 ");
#endif
		temp = (SpidRegs.SPIRXBUF) & 0xFF;
		diObj->pCS_Clr();
		return dio_device_error;
	}

	DELAY_NS(400);//400->500
	SpidRegs.SPIDAT = 0x00; // send dummy data
	//temp = 0;
	//-DELAY_NS(400);// 150515@Added
	while(!SpidRegs.SPISTS.bit.INT_FLAG);
	/*{
		temp++;
		DELAY_NS(10);
		if( temp > 20 ) // 150515@Fixed 11 -> 20
		{
#if DEBUG_MODE == 1
			//FIXME Delete!!
			System_printf("DI Input error\n");
			System_flush();
#endif
			diObj->pCS_Clr();
			bSPI_D_working = FALSE; // Added. 130515@
			return dio_device_error;

		}
	}*/

	temp = (SpidRegs.SPIRXBUF) & 0xFF;

	//FIXME 행태 수정.
	diObj->Port.all = 0;
	if(temp & 0x1)
		diObj->Port.bit.PORT8 = 1;
	if(temp & 0x2)
			diObj->Port.bit.PORT7 = 1;
	if(temp & 0x4)
			diObj->Port.bit.PORT6 = 1;
	if(temp & 0x8)
			diObj->Port.bit.PORT5 = 1;
	if(temp & 0x10)
			diObj->Port.bit.PORT4 = 1;
	if(temp & 0x20)
			diObj->Port.bit.PORT3 = 1;
	if(temp & 0x40)
			diObj->Port.bit.PORT2 = 1;
	if(temp & 0x80)
			diObj->Port.bit.PORT1 = 1;

	DELAY_NS(70);
	diObj->pCS_Clr();

	//dummy = (SpidRegs.SPIRXBUF) & 0xFF; // (수신 인터럽트가 클리어 되지가 않을 경우를 위한 테스트 코드.
	if(SpidRegs.SPISTS.bit.OVERRUN_FLAG)
	{
#if DEBUG_MODE == DEBUG_DEVELOP
		error();
		System_printf("\n\n OVERRUN_FLAG ");
#endif
	}

	bSPI_D_working = FALSE;
	//<--------------------------
	DELAY_US(3);
	return dio_command_succ;
}

public DIO_CTRL_STATUS DIO_getStatus(DI_LIST diGroup, DI_STATE* pIsoface)
{
	DIO_CTRL_STATUS status = dio_none;
	if( diGroup < DI_STX || diGroup > DI_END )
	{
		status = dio_unknown_port;
		return status;
	}
	if(diGroup < DI_GROUP_B)
	{
		status = DIO_input(&DI[DI_ISO_A]);
		pIsoface->all = DI[DI_ISO_A].Port.all;
		return status;
	}
	else if(diGroup < DI_GROUP_C)
	{
		status = DIO_input(&DI[DI_ISO_B]);
		pIsoface->all = DI[DI_ISO_B].Port.all;
		return status;
	}
	else
	{
		status = DIO_input(&DI[DI_ISO_C]);
		pIsoface->all = DI[DI_ISO_C].Port.all;
		return status;
	}
}

//주의: 기존 상태는 유지 되지 않고 새로 들어온 데이터로 모두 갱신함.
//임시 함수임.
#if 0
public DIO_CTRL_STATUS DIO_setStatusGroup(DO_CHIP groupId, DI_STATE diState)
{
	DIO_CTRL_STATUS status = dio_none;
	DIGITAL_CTRL* itsObj = 0;

	if( groupId < DO_ISO_STX || groupId > DO_ISO_END )
	{
		status = dio_unknown_port;
		return status;
	}
	itsObj = &DO[groupId];


	if(diState.bit.PORT1)
		DO[groupId].Port.all |= 1<<0;
	else
		DO[groupId].Port.all &= ~(1<<0);
	if(diState.bit.PORT2)
		DO[groupId].Port.all |= 1<<1;
	else
		DO[groupId].Port.all &= ~(1<<1);
	if(diState.bit.PORT3)
		DO[groupId].Port.all |= 1<<2;
	else
		DO[groupId].Port.all &= ~(1<<2);
	if(diState.bit.PORT4)
		DO[groupId].Port.all |= 1<<3;
	else
		DO[groupId].Port.all &= ~(1<<3);
	if(diState.bit.PORT5)
		DO[groupId].Port.all |= 1<<4;
	else
		DO[groupId].Port.all &= ~(1<<4);
	if(diState.bit.PORT6)
		DO[groupId].Port.all |= 1<<5;
	else
		DO[groupId].Port.all &= ~(1<<5);
	if(diState.bit.PORT7)
		DO[groupId].Port.all |= 1<<6;
	else
		DO[groupId].Port.all &= ~(1<<6);
	if(diState.bit.PORT8)
		DO[groupId].Port.all |= 1<<7;
	else
		DO[groupId].Port.all &= ~(1<<7);

#if DEBUG_MODE == DEBUG_DEVELOP
	if(DO[groupId].Port.all == 0xFF)
	{
		error();
	}
#endif

//	diState.bit.PORT1 ? DO[groupId].Port.all |= 1<<0 :DO[groupId].Port.all &= ~(1<<0);
//	diState.bit.PORT2 ? DO[groupId].Port.all |= 1<<1 :itsObj->Port.all &= ~(1<<1);
//	diState.bit.PORT3 ? DO[groupId].Port.all |= 1<<2 :itsObj->Port.all &= ~(1<<2);
//	diState.bit.PORT4 ? DO[groupId].Port.all |= 1<<3 :itsObj->Port.all &= ~(1<<3);
//	diState.bit.PORT5 ? DO[groupId].Port.all |= 1<<4 :itsObj->Port.all &= ~(1<<4);
//	diState.bit.PORT6 ? DO[groupId].Port.all |= 1<<5 :itsObj->Port.all &= ~(1<<5);
//	diState.bit.PORT7 ? DO[groupId].Port.all |= 1<<6 :itsObj->Port.all &= ~(1<<6);
//	diState.bit.PORT8 ? DO[groupId].Port.all |= 1<<7 :itsObj->Port.all &= ~(1<<7);
	return DIO_output(itsObj);
}
#endif


/* 주의: 시스템 초기화 시에 호출 되면 안됨. 초기 값과 다를 때만 ON/OFF를 수행하기 때문에
 * 최초에 변수에는 OFF로 인식되어 있는데 초기화 목적으로 OFF했을 경우 실제로 명령을 수행하지 않을 것임.
 * 따라서, 최초에는 DIO_Item_Init 로 초기화 함.
 */
public DIO_CTRL_STATUS DIO_setStatus(volatile DO_LIST doID, Bool bOnOff)
{
	//-static int bFirst = FALSE;
	DIGITAL_CTRL* itsObj = 0;
	DIO_CTRL_STATUS status = dio_none;
	UInt16 iPosition = 0;

	if( doID < DO_STX || doID > DO_END )
	{
		status = dio_unknown_port;
		return status;
	}

	if(doID < DO_GROUP_B)
	{
		itsObj = &DO[DO_ISO_A];
		iPosition = DO_ISO_A;
	}
	else
	{
		itsObj = &DO[DO_ISO_B];
		iPosition = DO_ISO_B;
	}

	iPosition = doID - (8 * iPosition) -1;


#if 1
	/* 141223 항시 Write 하도록 수정. HILL에서는 DIO 포트가 아닌 다른 보드라서 적용이 제대로 안됨. */
	if(bOnOff >= 1)
		itsObj->Port.all |= 1<<iPosition;
	else
		itsObj->Port.all &= ~(1<<iPosition);

	return DIO_output(itsObj);
#else

	// 141201 사용. HILL 에서 되는지 확인. => HILL 에서 MC 신호가 적용되지 않을 경우 발생.
	if( itsObj->Port.all & bOnOff<<iPosition )
	{
		status = dio_command_succ;
		return status;
	}

	// dummy 에 현재 상태를 백업한 후 성공했을 경우에만 Port data Update
	itsObj->dummy.all = itsObj->Port.all;

	if(bOnOff >= 1)
		itsObj->Port.all |= 1<<iPosition;
	else
		itsObj->Port.all &= ~(1<<iPosition);

	status = DIO_output(itsObj);
	if( status != dio_command_succ )
	{
		itsObj->Port.all = itsObj->dummy.all;
	}

	return status;
#endif
}

void testCode();
public void DIO_Create()
{
	DIO_Item_Init(DI, DI_ISO_A, DI_CS_A_SET, DI_CS_A_CLE);
	DIO_Item_Init(DI, DI_ISO_B, DI_CS_B_SET, DI_CS_B_CLE);
	DIO_Item_Init(DI, DI_ISO_C, DI_CS_C_SET, DI_CS_C_CLE);
	DIO_Item_Init(DO, DO_ISO_A, DO_CS_A_SET, DO_CS_A_CLE);
	DIO_Item_Init(DO, DO_ISO_B, DO_CS_B_SET, DO_CS_B_CLE);

	/*
	 * Digital Out A,B를 Open 상태로 초기화 한다.
	 */
	if(!DIO_DO_Clear(&DO[DO_ISO_A]))
	{
		//Fault Raise
#if DEBUG_MODE == 1
		error();
#endif
	}
	if(!DIO_DO_Clear(&DO[DO_ISO_B]))
	{
		//Fault Raise
#if DEBUG_MODE == 1
		error();
#endif
	}

	/*
	 * Digital Input A,B,C 상태를 메모리에 저장한다.
	 */
	if(!DIO_DI_Load(&DI[DI_ISO_A]) )
	{
		//Fault Raise
#if DEBUG_MODE == 1
		error();
#endif
	}
	if(!DIO_DI_Load(&DI[DI_ISO_B]) )
	{
		//Fault Raise
#if DEBUG_MODE == 1
		error();
#endif
	}
	if(!DIO_DI_Load(&DI[DI_ISO_C]) )
	{
		//Fault Raise
#if DEBUG_MODE == 1
		error();
#endif
	}

#if 0 /* TEST CORD */
testCode();
#endif
}



#if 0
void testCode()
{
	DI_STATE inputInfo1;
	DI_STATE inputInfo2;
	DI_STATE inputInfo3;
	DI_STATE inputInfo4;

	DIO_setStatus(1,ON);
	DIO_setStatus(1,OFF);
	DIO_setStatus(2,ON);
	DIO_setStatus(2,OFF);
	DIO_setStatus(3,ON);
	DIO_setStatus(3,OFF);
	DIO_setStatus(4,ON);
	DIO_setStatus(4,OFF);
	DIO_setStatus(5,ON);
	DIO_setStatus(5,OFF);
	DIO_setStatus(6,ON);
	DIO_setStatus(6,OFF);
	DIO_setStatus(7,ON);
	DIO_setStatus(7,OFF);
	DIO_setStatus(8,ON);
	DIO_setStatus(8,OFF);

	DIO_setStatus(9,ON);
	DIO_setStatus(9,OFF);
	DIO_setStatus(10,ON);
	DIO_setStatus(10,OFF);
	DIO_setStatus(11,ON);
	DIO_setStatus(11,OFF);
	DIO_setStatus(12,ON);
	DIO_setStatus(12,OFF);

	DIO_setStatus(1,ON);
	DIO_setStatus(3,ON);
	DIO_setStatus(5,ON);
	DIO_setStatus(7,ON);

	DIO_setStatus(9,ON);
	DIO_setStatus(11,ON);
	DIO_setStatus(13,ON);

	DIO_getStatus(DI_GROUP_A, &inputInfo1);
	DIO_getStatus(DI_GROUP_B, &inputInfo2);
	DIO_getStatus(DI_GROUP_C, &inputInfo3);


#if 0
	Uint16 RxData = 0;
	/*---------------------------------------------------------------------------
		SPI 데이터 송신
	---------------------------------------------------------------------------*/
	DI_CS_A_CLE();
	DI_CS_B_CLE();
	DI_CS_C_CLE();
	DO_CS_A_CLE();
	DO_CS_B_CLE();

	DO_CS_A_SET();
	DELAY_NS(30);
	SpidRegs.SPIDAT = 0x0100;
	while(!SpidRegs.SPISTS.bit.INT_FLAG);
	DO_CS_A_CLE();
	// reset int flag
	RxData = SpidRegs.SPIRXBUF;

	DELAY_NS(100);

	DO_CS_B_SET();
	DELAY_NS(30);
	SpidRegs.SPIDAT = 0x0100;
	while(!SpidRegs.SPISTS.bit.INT_FLAG);
	DO_CS_B_CLE();
	// reset int flag
	RxData = SpidRegs.SPIRXBUF;

	/*=================== DI ===================*/
	SpidRegs.SPICCR.bit.SPISWRESET=0;				// Reset SPI
	SpidRegs.SPICCR.bit.CLKPOLARITY = 0;			// RISING EDGE
	SpidRegs.SPICTL.bit.CLK_PHASE  = 0;				// with Delay
	SpidRegs.SPICCR.bit.SPISWRESET=1;

	DELAY_NS(100);
	DI_CS_A_SET();
	DELAY_NS(100);
	SpidRegs.SPIDAT = 0x00; // send dummy data
	while(!SpidRegs.SPISTS.bit.INT_FLAG);
	RxData = (SpidRegs.SPIDAT ) & 0xFF;
	DELAY_NS(70);
	DI_CS_A_CLE();

	DELAY_NS(2500);
	DI_CS_B_SET();
	DELAY_NS(100);
	SpidRegs.SPIDAT = 0x00; // send dummy data
	while(!SpidRegs.SPISTS.bit.INT_FLAG);
	RxData = (SpidRegs.SPIDAT  ) & 0xFF;
	DELAY_NS(70);
	DI_CS_B_CLE();

	DELAY_NS(2500);
	DI_CS_C_SET();
	DELAY_NS(100);
	SpidRegs.SPIDAT = 0x00; // send dummy data
	while(!SpidRegs.SPISTS.bit.INT_FLAG);
	RxData = (SpidRegs.SPIDAT  ) & 0xFF;
	DELAY_NS(70);
	DI_CS_C_CLE();
#endif
}
#endif

