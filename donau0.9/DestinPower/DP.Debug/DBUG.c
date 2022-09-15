/*
 * TEST_delfino.c
 *
 *  Created on: 2012. 11. 1.
 *      Author: destinPower
 */

#include <ti/sysbios/knl/Task.h>
#include <Math.h>
#include <String.h>
#include "LGS_Common.h"
#include "DBUG.h"
#include "MathConst.h"
#include "McBSP_SPI.h"
#include "DSP28x_Project.h"
#include "slld.h"
#include "DIO.h"
#include "NvSRAM.h"
#include "prm_pcs.h"
#include "History.h"
#include "SPIa.h"
#include "CC.h"
#include "MCCB.h"

#if DEBUG_MODE == DEBUG_SIMULSATION
#define DBUG_VDC_MIN 100
extern float TccIVdc, CC_tsCC, CC_fDTheta;

DIO_CTRL_STATUS diRes = dio_none;
#endif

Debug DBUG;
extern float TccIVdc, CC_tsCC, CC_fDTheta;

#if DBUG_MODE == 1 || DBUG_MODE == 2

UInt16 retValue = 0;
UInt16 iVerify = 0;
UInt32 lTemp = 0;
UInt32 wirteAddr = 0;
UInt32 readAddr = 0;
UInt32 lEraseSector = 0;
#endif

void TEST_Create()
{
#if DEBUG_MODE == DEBUG_DEVELOP

	SLLD_STATUS dfStatus= SLLD_ERROR;
	DEVSTATUS dfDevStatus = dev_status_unknown;
	DI_STATE isoFace;
	int retArray[10];
	Uint16 i = 0;
#if 0
	// I2C TEMP
	// 141230 Tested

	while(1)
	{
		DELAY_US(625000); // 1Sec
		//DELAY_US(625000); // 1Sec
		//DELAY_US(625000); // 1Sec

		if( I2C_GET_Temp( &retValue ) )
		{
			System_printf("REQ SUCC. Temp : %d \n", retValue);
		}
		else
		{
			System_printf("REQ FAIL. Temp : %d \n", retValue);
		}
		System_flush();
	}
	error();


#endif

#if 0 // MRAM TEST
   MCBSP_SPI_WREN();
   MCBSP_SPI_statusWrite(0);
   int k = 0;
    while(1)
    {
		int testValue;
#if 0 // 활성화 하면 write후 read값이 잘못된 값이 읽혀질 때가 있음을 확인.
		testValue = MCBSP_SPI_statusRead();

		System_printf("MRAM Status is %d\n", testValue);
		System_flush();

		//asm("     ESTOP0");

		testValue = MCBSP_SPI_statusRead();
		System_printf("MRAM Status is %d\n", testValue);
		System_flush();
#endif

#if 1
		if( NvSRAM_Write(0, k++, FALSE ) )
		{
			System_printf("MRAM Write Succ\n");
			System_flush();
		}
		else
		{
			System_printf("MRAM Status Fail\n");
			System_flush();
		}

		asm("     ESTOP0");

		if( NvSRAM_Read(0, &testValue) )
		{
			System_printf("MRAM Read Succ : %d\n", testValue);
			System_flush();
		}
		else
		{
			System_printf("MRAM Read Fail\n");
			System_flush();
		}

		//-MCBSP_SPI_WREN();
#endif

    }
#endif

#if 0 // DIO GROUP SET TEST

	DIO_setStatus(DO_1,TRUE);
	DIO_setStatus(DO_2,TRUE);
	DIO_setStatus(DO_3,TRUE);
	DIO_setStatus(DO_4,TRUE);
	DIO_setStatus(DO_5,TRUE);
	DIO_setStatus(DO_6,TRUE);
	DIO_setStatus(DO_7,TRUE);

	//public DIO_CTRL_STATUS DIO_getStatus(DI_LIST diGroup, DI_STATE* pIsoface)
	DIO_getStatus(DI_GROUP_A,&isoFace );
	DIO_getStatus(DI_GROUP_B,&isoFace );
	DIO_getStatus(DI_GROUP_C,&isoFace );

//	DIO_setStatus(DO_1,TRUE);
//	DIO_setStatus(DO_2,TRUE);
//	DIO_setStatus(DO_3,TRUE);
//	DIO_setStatus(DO_4,TRUE);
//	DIO_setStatus(DO_5,TRUE);
//	DIO_setStatus(DO_6,TRUE);


#endif
#if 0 // History Queue Test
	HISTORY_STORE data;

	data.hisId = 1;
	QUEUE_enqueHistory(data);

	data.hisId = 2;
	QUEUE_enqueHistory(data);

	data.hisId = 3;
	QUEUE_enqueHistory(data);

	while(QUEUE_dequeHistory(&data))
	{
		System_printf("Res Data %d\r\n", data.hisId);
		System_flush();
	}

#endif

#if 0
	// I2C TX/RX FIFO 통신 테스트 완. (RTC)
	RtcTime TI;

	for(;;){
		//-DS1743HW_get(TI);
		memset(&TI, 0, sizeof(TI));
		if( I2C_GET_RTC_Time(&TI) == FALSE )
		{
			error();
		}

		TI.Date += 1;
		if( I2C_SET_RTC_Time(&TI) == FALSE )
		{
			error();
		}
	}
#endif


#if 0 /* LED Test Code */
	TEST_DELFINO_LedOnOff(0,TRUE); //OFF
	TEST_DELFINO_LedOnOff(0,FALSE);//ON
	TEST_DELFINO_LedOnOff(1,TRUE); //OFF
	TEST_DELFINO_LedOnOff(1,FALSE); //ON
#endif

#if 0
#if McBSP_SPI_GPIO_MODE == 0
	/*
	 * SRAM Test Code
	 * 13.01.29 : 최저 속도로 테스트 완료.
	 */
//
//	 // Write Test
	iVerify = 10;
	for( i=1; i<= 50; i++)
	{
		if(iVerify >= 255)
			iVerify = 0;

		/*
		while( 1 )
		{
			if( NvSRAM_Write(i, iVerify++,TRUE) == FALSE )
				continue;
			else
				break;
		}
		*/
		iVerify++;
		if( !NvSRAM_Write(i, iVerify,FALSE) )
		{
			error();
		}
	}

	System_printf("NvSRAM Write Tested.\n");
	System_flush();
	error();

	while(1)
	{
		for( i=1; i<= 50; i++)
		{
			if( !NvSRAM_Read(i, &iVerify) )
			{
				error();
			}
			else
			{
				System_printf("%d,",iVerify);
			}

			if( !(i % 20) )
			{
				System_printf("\n");
				System_flush();
			}
		}
		DELAY_US(625000); // 1Sec
	}

	System_printf("NvSRAM Read Tested. \n");
	System_flush();
	error();

#else
/// 테스트 완료.
 	 // Write Test
	for( i=0; i<= 32767; i++)
	{
		if(iVerify > 255)
			iVerify = 0;

		MCBSP_SPI_byteWrite(i, iVerify++);
	}

	iVerify = 0;
	// Read only Test
	for( i=0; i<= 32767; i++)
	{
		if(iVerify > 255)
			iVerify = 0;

		MCBSP_SPI_byteRead(i,&retValue);

		if( retValue != iVerify++)
		{
			error();
		}
	}
#endif
#endif

#if 0 // SPI-DATA FLASH TESTED
	BYTE dataArray[512] = {0};
	DEVSTATUS s;
	int iNum =0;
	iVerify = 10;
	wirteAddr = 0;
	readAddr = 0;
	lEraseSector = 0;
	/* DATA FLASH TEST */
	SPIA_dataFlash_ENB();

	//dfStatus = slld_ClearStatusRegisterCmd();
	while(1)
	{
		asm("     ESTOP0");

		dfStatus = slld_StatusGet(&dfDevStatus);

		System_printf("slld_StatusGet [ %d  %d]\n", dfDevStatus, dfStatus);
		System_flush();
		slld_SEOp(lEraseSector*0x10000, &s); /* Erase (Sector Erase. 64KB를 삭제한다.) 섹터 단위 시작 addr을 입력. */

		//while(1)
		{

#if 0
			for(iNum =0; iNum< 512; iNum++)
				dataArray[iNum] = iVerify; // iNum+1;

            // 함수의 원형부가 없으면 파라미터 전달 시 스택을 기본 TYPE 크기 잡음.
			// 최대 256 바이트 까지만 쓰기 가능.


			/* 기본 입출력 테스트. 정상 작동 함. */
			if( !SPIA_dataFlash_writeByte(wirteAddr, dataArray, 256) )
			{
				System_printf("write failed\n");
				System_flush();
			}

			System_printf("read status: %d \n",SPIA_dataFlash_ReadStatus());


			for(iNum =0; iNum< 512; iNum++)
				dataArray[iNum] = 0;

			if( SPIA_dataFlash_readByte(readAddr, dataArray, 256) )
			{
				System_printf("read val: %d \n", dataArray[255]);
				System_flush();
			}
			else
			{
				System_printf("read failed\n");
				System_flush();
			}
#else
			for(iNum =0; iNum< 256; iNum++)
				dataArray[iNum] = iVerify; // iNum+1;

			//public Bool SPIA_dataFlash_writeInt16(UInt32 address/*24bit*/,char* const pData, Uint16 size, int isHigh)
			asm("     ESTOP0");
			if( !SPIA_dataFlash_writeInt16(wirteAddr, dataArray, 256 ) )
			{
				System_printf("write failed\n");
				System_flush();
			}
			asm("     ESTOP0");
			for(iNum =0; iNum< 512; iNum++)
				dataArray[iNum] = 0;
			asm("     ESTOP0");
			if( SPIA_dataFlash_readByteInt16(readAddr, dataArray, 256) )
			{
				System_printf("read val: %d \n", dataArray[255]);
				System_flush();
			}
			else
			{
				System_printf("read failed\n");
				System_flush();
			}

#endif

 			asm("     ESTOP0");
		}
	}
#elif 0
	volatile int RxData;
	int iNum =0;
	DEVSTATUS s;
	BYTE dataArray[256] = {0};
	SPIA_dataFlash_ENB();
	asm("     ESTOP0");

	/* 디바이스 드라이버 포팅 코드 - 동작 잘 됨. DATA FLASH 테스트 완료 */
		slld_SEOp(0x000000, &s); /* Erase */
		asm("     ESTOP0");

		for(iNum =0; iNum< 256; iNum++)
			dataArray[iNum] = 7; // iNum+1;

		asm("     ESTOP0");
		slld_PPOp(0x500000, dataArray, 256, &s);
		asm("     ESTOP0");

		for(iNum =0; iNum< 256; iNum++)
			dataArray[iNum] = 0;

		asm("     ESTOP0");

		slld_ReadOp(0x500000, dataArray, 256);
		asm("     ESTOP0");

#endif

#if 0
	NvSRAM_SetRunCommand(ON);
	System_printf("SNvSRAM_CheckRunCommand [ %d ]\n", NvSRAM_CheckRunCommand());
	System_flush();

	NvSRAM_SetRunCommand(OFF);
	System_printf("SNvSRAM_CheckRunCommand [ %d ]\n", NvSRAM_CheckRunCommand());
	System_flush();
#endif
#endif

//#if DBUG_MODE == 1 || DBUG_MODE == 2
	TEST_Led(3, 0);
	TEST_Led(4, 0);

	memset(&DBUG, 0, sizeof(DBUG));

	DBUG.Theta = 0;
	DBUG.SinTheta = 0;
	DBUG.CosTheta = 0;
	DBUG.Vds = 0;
	DBUG.Vqs = 0;
	DBUG.Va = 0;
	DBUG.Vb = 0;
	DBUG.Vc = 0;
//#endif
}
#if DBUG_MODE == 1 || DBUG_MODE == 2
float fPhaseJump = 0;
int bPhaseJump = 0;
int bGridOff = 1;
#endif

void TEST_UpdateEVAConvResult(void)
{
	float Ratio;

	DBUG.Theta += CC_fDTheta;

#if DBUG_MODE == 1 || DBUG_MODE == 2
	// 141201 June
	if( bPhaseJump )
	{
		bPhaseJump = FALSE;
		DBUG.Theta += fPhaseJump * 0.0174; // degree to radian
	}

	MCCB.Status.BitVal.bExMCCB1 = ON;
	MCCB.Status.BitVal.bExMCCB2 = ON;
	MCCB.Status.BitVal.bExMCCB3 = OFF;
#endif

	if ( DBUG.Theta > PI )
	{
		DBUG.Theta += -TWO_PI;
	}
	else if ( DBUG.Theta < -PI ) DBUG.Theta += TWO_PI;

	if(PRM_PCS[INV_TRANSFORMER].iValue == 1)
	    DBUG.Theta_TR = DBUG.Theta - PI_6;
	else
	    DBUG.Theta_TR = DBUG.Theta;

	DBUG.SinTheta = sin(DBUG.Theta); 	DBUG.CosTheta = cos(DBUG.Theta);
	DBUG.SinTheta_TR = sin(DBUG.Theta_TR);    DBUG.CosTheta_TR = cos(DBUG.Theta_TR);

	DBUG.Vds = - DBUG.SinTheta;
	DBUG.Vqs = DBUG.CosTheta;
    DBUG.Vds_TR = - DBUG.SinTheta_TR;
    DBUG.Vqs_TR = DBUG.CosTheta_TR;
//	DBUG.Ids = - DBUG.SinTheta;
//	DBUG.Iqs = DBUG.CosTheta;

	TRANSFORM_dq_abc(DBUG.Vds, DBUG.Vqs, DBUG.Va, DBUG.Vb, DBUG.Vc);
	TRANSFORM_dq_abc(DBUG.Vds_TR, DBUG.Vqs_TR, DBUG.Va_TR, DBUG.Vb_TR, DBUG.Vc_TR);

#if DBUG_MODE == 1 || DBUG_MODE == 2
	/*
	 * Vbyp 전압, 실제 계통 전압
	 */
//	if( GenBlock.bBypV == FALSE )
//	{
//		// 그리드 전압
//		DBUG.Vbyp_a = 0.;
//		DBUG.Vbyp_b = 0.;
//		DBUG.Vbyp_c = 0.;
//	}
//	else if( GenBlock.bBypV == TRUE )
//	{
	if( bGridOff >= 1 ){
		// 그리드 전압
		DBUG.Vbyp_a = DBUG.Va_TR * PRM_PCS[ANL_AI1_OFFSET].iValue * PRM_PCS[ANL_AI1_OFFSET].fIncDec;
		DBUG.Vbyp_b = DBUG.Vb_TR * PRM_PCS[ANL_AI1_GAIN].iValue * PRM_PCS[ANL_AI1_GAIN].fIncDec;
		DBUG.Vbyp_c = DBUG.Vc_TR * PRM_PCS[ANL_AI2_OFFSET].iValue * PRM_PCS[ANL_AI2_OFFSET].fIncDec;
	}
//	}

    /*
     * Vpcc 전압, PCC 지점 전압 GC: 계통 전압, GI: 인버터 전압
     */
//	if( (GenBlock.bypSw == TRUE) && ((INVERTER.uStatus == SYS_INV_FAULT) || (INVERTER.uStatus == SYS_INV_STOP)
//	        || (INVERTER.uStatus == SYS_INV_SCR_ON) || (INVERTER.uStatus == SYS_INV_DC_CHARGE)
//	        || (INVERTER.uStatus == SYS_INV_EVE_DC_CHARGE) || (INVERTER.uStatus == SYS_INV_AC_GENERATE)
//	        || (INVERTER.uStatus == SYS_INV_START_SYNC) || (INVERTER.uStatus == SYS_INV_RUN)))
	if( bGridOff == 2 )
	{
        DBUG.Vg_a = DBUG.Vbyp_b;
        DBUG.Vg_b = DBUG.Vbyp_a;
        DBUG.Vg_c = DBUG.Vbyp_c;
	}
	else if( bGridOff == 3 )
	{
		DBUG.Vg_a = DBUG.Vbyp_a;
		DBUG.Vg_b = DBUG.Vbyp_c;
		DBUG.Vg_c = DBUG.Vbyp_b;
	}
	else
	{
		DBUG.Vg_a = DBUG.Vbyp_a;
		DBUG.Vg_b = DBUG.Vbyp_b;
		DBUG.Vg_c = DBUG.Vbyp_c;
	}
//	else if( (GenBlock.bypSw == FALSE) && ((INVERTER.uStatus == SYS_INV_ISLANDING) || (INVERTER.uStatus == SYS_INV_RE_SYNC)) )
//	{
//        DBUG.Vg_a = DBUG.Vinv_a * TR_RATIO_INVERSE;
//        DBUG.Vg_b = DBUG.Vinv_b * TR_RATIO_INVERSE;
//        DBUG.Vg_c = DBUG.Vinv_c * TR_RATIO_INVERSE;
//	}
//	else
//	{
//        DBUG.Vg_a = 0;
//        DBUG.Vg_b = 0;
//        DBUG.Vg_c = 0;
//	}
#endif
    /*
     * Vinv 전압, 인버터 전압
     */
//	if((INVERTER.uStatus == SYS_INV_AC_GENERATE) || (INVERTER.uStatus == SYS_INV_START_SYNC)
//	        || (INVERTER.uStatus == SYS_INV_RUN) || (INVERTER.uStatus == SYS_INV_ISLANDING)
//	        || (INVERTER.uStatus == SYS_INV_RE_SYNC) || (INVERTER.uStatus == SYS_INV_BYP_EVT_OPERATION))
//	{
        Ratio = PRM_PCS[ANL_AI2_GAIN].iValue * PRM_PCS[ANL_AI2_GAIN].fIncDec;
        DBUG.Vinv_a = DBUG.Va * Ratio;
        DBUG.Vinv_b = DBUG.Vb * Ratio;
        DBUG.Vinv_c = DBUG.Vc * Ratio;
//	}
//	else
//	{
//        DBUG.Vinv_a = 0;
//        DBUG.Vinv_b = 0;
//        DBUG.Vinv_c = 0;
//	}

    DBUG.Iqe = (PARAM_VAL(BATT_REMOTE_PCC_P_REF) * 1000.) * INV_SQRT3 /  PARAM_VAL(GRID_RATED_VOLT);

	DBUG.Iinv_a = DBUG.Va * DBUG.Iqe *  PRM_PCS[ANL_AI4_OFFSET].iValue * PRM_PCS[ANL_AI4_OFFSET].fIncDec * 0.001;
	DBUG.Iinv_b = DBUG.Vb * DBUG.Iqe *  PRM_PCS[ANL_AI4_OFFSET].iValue * PRM_PCS[ANL_AI4_OFFSET].fIncDec * 0.001;
	DBUG.Iinv_c = DBUG.Vc * DBUG.Iqe *  PRM_PCS[ANL_AI4_OFFSET].iValue * PRM_PCS[ANL_AI4_OFFSET].fIncDec * 0.001;

	if(PRM_PCS[INV_TRANSFORMER].iValue == 0)
	{
        DBUG.Itr_a = DBUG.Va * DBUG.Iqe;
        DBUG.Itr_b = DBUG.Vb * DBUG.Iqe;
        DBUG.Itr_c = DBUG.Vc * DBUG.Iqe;
	}
	else
	{
        DBUG.Itr_a = DBUG.Va * DBUG.Iqe * TR_RATIO;
        DBUG.Itr_b = DBUG.Vb * DBUG.Iqe * TR_RATIO;
        DBUG.Itr_c = DBUG.Vc * DBUG.Iqe * TR_RATIO;
	}
	//DBUG.Ipv = PRM_PCS[ANL_AI2_GAIN].iValue * PRM_PCS[ANL_AI2_GAIN].fIncDec;
	DBUG.Ipv = 0;

#if 0 //by JCNET
	if(MCCB.Status.BitVal.bDS1 || MCCB.Status.BitVal.bMC1A)
#else
	if(MCCB.Status.BitVal.bMC1 || MCCB.Status.BitVal.bMC1A)
#endif
	    DBUG.Vdc = PRM_PCS[ANL_AI3_OFFSET].iValue * PRM_PCS[ANL_AI3_OFFSET].fIncDec;
	else
	    DBUG.Vdc = 0;

	DBUG.Vdc_batt = PRM_PCS[ANL_AI3_GAIN].iValue * PRM_PCS[ANL_AI3_GAIN].fIncDec;

	//Ratio = ( (-PARAM_VAL(BATT_REMOTE_PCC_P_REF) * INV_3 / ACP.PCC.RATE.Vph) + (GenBlock.SET_loadPowerMeterP * INV_3 / (ACP.PCC.RATE.Vph * 1000.)) ) * SQRT3;
	Ratio = DBUG.Iqe * TR_RATIO_320_208;
	DBUG.Ibyp_a = DBUG.Va*Ratio/**0.2*/;
	DBUG.Ibyp_b = DBUG.Vb*Ratio/**0.2*/;
	DBUG.Ibyp_c = DBUG.Vc*Ratio/**0.2*/;

}
void TEST_LedToggle(int ledNum)
{
	//return;
//#if DBUG_MODE == 2
//	if( ledNum == 3)
//		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
//	else if(ledNum == 4)
//		GpioDataRegs.GPBTOGGLE.bit.GPIO35 = 1;
//#else
	if( ledNum == 0)
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
	else if(ledNum == 1)
		GpioDataRegs.GPBTOGGLE.bit.GPIO35 = 1;
//#endif
}

void TEST_Led(int ledNum, int onoff)
{
	onoff = !onoff;

//#if DBUG_MODE == 2
//	if( ledNum == 3)
//		GpioDataRegs.GPBDAT.bit.GPIO34 = onoff;
//	else if(ledNum == 4)
//		GpioDataRegs.GPBDAT.bit.GPIO35 = onoff;
//#else
	if( ledNum == 0)
		GpioDataRegs.GPBDAT.bit.GPIO34 = onoff;
	else if(ledNum == 1)
		GpioDataRegs.GPBDAT.bit.GPIO35 = onoff;
//#endif
}
