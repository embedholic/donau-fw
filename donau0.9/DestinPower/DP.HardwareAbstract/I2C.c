/*
 * I2C.c
 *
 *  Created on: 2012. 11. 8.
 *      Author: destinPower
 */

/*
 * 13-02-06 JUNE 수신 인터럽트 처리시 제대로 수신하지 못하여, 수신 방식을 FIFO로 수정한 후 정상 동작 확인 함.
 */


#include "I2C.h"
#include "DSP28x_Project.h"

#define I2C_RTC 0
#define I2C_TEMP 1
#define I2C_CHIP I2C_TEMP
#define IN_DATA_BUF_MAX 10


#define I2C_SLAVE_ADDR_TEMP 0x18
#define I2C_SLAVE_ADDR_RTC 0x68


volatile RtcTime rtcBuffer;
RtcTime rtcBufferShadow;

volatile Uint16 InData[IN_DATA_BUF_MAX]={0};

//150430 June
static int bI2cWorking = FALSE;

Bool I2C_Wait(void)
{
	// Wait until the STP bit is cleared from any previous master communication.
	// Clearing of this bit by the module is delayed until after the SCD bit is
	// set. If this bit is not checked prior to initiating a new message, the
	// I2C could get confused.
	unsigned int I2cResetDelay = 0;

	//+ FIXED 140219
	while (I2caRegs.I2CMDR.bit.STP == 1)
	{ // Wait for Stop condition bit to be zero.
		I2cResetDelay++;
		if(I2cResetDelay >= 65530)
		{
			I2cResetDelay = 0;
			I2caRegs.I2CMDR.bit.IRS = 0;
			I2caRegs.I2CMDR.bit.IRS = 1;
			return FALSE;
		}
	}

	I2cResetDelay = 0;

	while (I2caRegs.I2CSTR.bit.BB == 1)
	{  // Wait for Bus Busy to be zero.
		if(I2cResetDelay++ >= 65530)
		{
			I2cResetDelay = 0;
			I2caRegs.I2CMDR.bit.IRS = 0;
			I2caRegs.I2CMDR.bit.IRS = 1;
			return FALSE;
		}
	}

	return TRUE;
}

void I2C_Init(void)
{
	int i;
	// Initialize I2C
	I2caRegs.I2CSAR = I2C_SLAVE_ADDR_RTC;		// Slave Address.
	I2caRegs.I2COAR = 0x0001;       		// address as Master.

#if I2C_CHIP == I2C_TEMP
	// TEMP 동작 확인.
	I2caRegs.I2CPSC.bit.IPSC = 29;		// 300Mhz / 30 = 10Mhz, Tmod = 100NS
	I2caRegs.I2CCLKL = 47-5;			// (30 * 100NS = 3us;OK) - RTC는 최소 1.3us - Temp는 최소 4.7us
	I2caRegs.I2CCLKH = 40-5;			// (20 * 100NS = 2us;OK) - RTC는 최소 1.3us - Temp는 최소 4us  114kHz

#else
	// RTC 동작 잘됨
	I2caRegs.I2CPSC.bit.IPSC = 29;		// 300Mhz / 30 = 10Mhz, Tmod = 100NS
	I2caRegs.I2CCLKL = 30-5;			// (30 * 100NS = 3us;OK) - RTC는 최소 1.3us - Temp는 최소 4.7us
	I2caRegs.I2CCLKH = 20-5;			// (20 * 100NS = 2us;OK) - RTC는 최소 1.3us - Temp는 최소 4us  114kHz

	//more fast
	//-I2caRegs.I2CPSC.bit.IPSC = 29;		// 300Mhz / 30 = 10Mhz, Tmod = 100NS
	//-I2caRegs.I2CCLKL = 10-5;			// (10 * 100NS = 1us;OK) - RTC는 최소 1.3us - Temp는 최소 4.7us
	//-I2caRegs.I2CCLKH = 7-5;				// (7 * 100NS = 0.7 us;OK) - RTC는 최소 1.3us - Temp는 최소 4us  114kHz
#endif

	//-I2caRegs.I2CIER.all = 0x2C;	// Enable SCD & ARDY interrupts & 수신 인터럽트
	I2caRegs.I2CIER.all = 0x24;		// Enable SCD & ARDY interrupts - 수신 FIFO 사용을 위해 수신 인터럽트 삭제
	I2caRegs.I2CMDR.all = 0;
	I2caRegs.I2CMDR.bit.FREE = 1;
	I2caRegs.I2CMDR.bit.IRS = 1;	// Take I2C out of reset
									// Stop I2C when suspended

	I2caRegs.I2CFFTX.all = 0x6000;	// Enable FIFO mode and TXFIFO
	I2caRegs.I2CFFRX.all = 0x2040;	//+ Enable RXFIFO, clear RXFFINT,


#if I2C_CHIP == I2C_TEMP
	GpioDataRegs.GPBDAT.bit.GPIO43 = 1;
#else

	// RTC CS ACTIVE!
	GpioDataRegs.GPBDAT.bit.GPIO43 = 0;
#endif

	for(i =0; i < IN_DATA_BUF_MAX; i++)
		InData[i] = 0;

	rtcBuffer.Year = 1;
	rtcBuffer.Month = 1;
	rtcBuffer.Date = 1;
	rtcBuffer.Hour = 1;
	rtcBuffer.Minute = 1;
	rtcBuffer.Second = 1;
	//-rtcBuffer.bUpdating = TRUE;

	rtcBufferShadow = rtcBuffer;

	bI2cWorking = FALSE;
}

//BOOL I2CA_SetTime(Uint16 seconds,Uint16 minutes, Uint16 hours, Uint16 date, Uint16 month, Uint16 year)
Bool I2C_SET_RTC_Time(RtcTime* pTime)
{
	UInt16 temp, temp1;

	if( bI2cWorking )
		return FALSE;

	bI2cWorking = TRUE;

	I2caRegs.I2CSAR = I2C_SLAVE_ADDR_RTC;		// Slave Address.
	I2caRegs.I2CMDR.bit.IRS = 0;	//
	I2caRegs.I2CMDR.bit.IRS = 1;	//

	// Slave Address info gets passed with Start Condition
	I2caRegs.I2CCNT = 8; 			// 3 Additional Bytes being tranferred.
	I2caRegs.I2CDXR = 0x00;     // Send Register to be updated.

	//Write seconds
	if (pTime->Second > 59)						//Ensure value is in range
		goto rtc_set_time_fail;
	temp1 = (pTime->Second / 10);
	temp = (pTime->Second - (temp1 * 10)) + (temp1 << 4);
	temp &= 0x7f;							//Bit7 = enable oscillator
	I2caRegs.I2CDXR = temp;

	//Write minutes
	if (pTime->Minute > 59)						//Ensure value is in range
		goto rtc_set_time_fail;

	temp1 = (pTime->Minute / 10);
	temp = (pTime->Minute - (temp1 * 10)) + (temp1 << 4);
	I2caRegs.I2CDXR = temp;

	//Write hours
	if (pTime->Hour > 23)					//Ensure value is in range
		goto rtc_set_time_fail;

	temp1 = (pTime->Hour / 10);
	temp = (pTime->Hour - (temp1 * 10)) + (temp1 << 4);
	temp &= 0x3f;						//Bit6 low = set format to 24 hour
	I2caRegs.I2CDXR = temp;

	//Write day
	I2caRegs.I2CDXR = 1; // day는 1로 고정

	//Write date
	if (pTime->Date > 31)						//Ensure value is in range
		goto rtc_set_time_fail;

	temp1 = (pTime->Date / 10);
	temp = (pTime->Date - (temp1 * 10)) + (temp1 << 4);
	I2caRegs.I2CDXR = temp;

	//Write month
	if (pTime->Month > 12)					//Ensure value is in range
		goto rtc_set_time_fail;

	temp1 = (pTime->Month / 10);
	temp = (pTime->Month - (temp1 * 10)) + (temp1 << 4);
	I2caRegs.I2CDXR = temp;

	//Write year
	if(pTime->Year >= 2000)
	{
		pTime->Year -= 2000;
	}

	if (pTime->Year > 99)						//Ensure value is in range
		goto rtc_set_time_fail;

	temp1 = (pTime->Year / 10);
	temp = (pTime->Year - (temp1 * 10)) + (temp1 << 4);
	I2caRegs.I2CDXR = temp;

	I2caRegs.I2CMDR.all = 0x6E20;   // Set up the control register:
	// bit 14 FREE = 1
	// bit 13 STT = 1  (Start condition)

	// bit 11 STP = 1  (Stop condition after
	//                transfer of bytes.)
	// bit 10 MST = 1  Master
	// bit  9 TRX = 1  Transmit

	// bit  5 IRS = 1 to Reset I2C bus.
	if( I2C_Wait() == FALSE )
	{
		bI2cWorking = FALSE;
		return FALSE;
	}

	bI2cWorking = FALSE;
	return TRUE;


//----- I2C COMMS FAILED -----
rtc_set_time_fail:
	bI2cWorking = FALSE;
	return FALSE;
}

/*
 * RTC의 데이터를 요청한다. 요청 후 받은 데이터는 수신 인터럽트에서 pTime 객체에 저장된다.
 *
 */
Bool I2C_GET_RTC_Time(RtcTime* pTime)
{
	int temp;
	//-int dummy;
	Bool bInit = 0;
	bInit = FALSE;

	if( bI2cWorking )
	 	return FALSE;

	bI2cWorking = TRUE;

	I2caRegs.I2CSAR = I2C_SLAVE_ADDR_RTC;		// Slave Address.
	I2caRegs.I2CMDR.bit.IRS = 0;	//
	I2caRegs.I2CMDR.bit.IRS = 1;	//

	int I2cIndex = 0;                   // Reset value for ISR.
	InData[0] = InData[1] = InData[2] = InData[3] = InData[4]= InData[5]=InData[6]=InData[7]=0;
	// Slave Address info gets passed with Start Condition
	I2caRegs.I2CCNT = 1; 			// 1 Additional Byte being tranferred.
	I2caRegs.I2CDXR = 0x0;     // Send Register to be updated.
	I2caRegs.I2CMDR.all = 0x6620  ;   // Set up the control register:


//	I2caRegs.I2CMDR.bit.FREE = 1;
//	I2caRegs.I2CMDR.bit.STT = 1; // b:13 Start Condition
//	I2caRegs.I2CMDR.bit.STP = 0; // b:11 No Stop condition after transfer of bytes
//	I2caRegs.I2CMDR.bit.MST = 1; // b:10 Master
//	I2caRegs.I2CMDR.bit.TRX = 1; // b:9 Transmitter mode
//	I2caRegs.I2CMDR.bit.IRS = 1; // b:5 Reset I2C bus.

	DELAY_US(50);                   // Delay (50 ~ 10)
									// Delay 함수를 쓰면 안되나, 쓰지 않으면 데이터를 잘 못 가져올 경우가 많다.
	//-while( I2caRegs.I2CSTR.bit.ARDY == 0){};

	//-rtcBuffer.bUpdating = TRUE;
	I2caRegs.I2CCNT = 7;            // Set up receive of 7 bytes.
	I2caRegs.I2CMDR.all = 0x6C20;	// Send "repeated" Start with Read (TRX off)
//	I2caRegs.I2CMDR.bit.FREE = 1;
//	I2caRegs.I2CMDR.bit.STT = 1; // b:13 Start Condition
//	I2caRegs.I2CMDR.bit.STP = 1; // b:11 Stop condition after transfer of bytes
//	I2caRegs.I2CMDR.bit.MST = 1; // b:10 Master
//	I2caRegs.I2CMDR.bit.TRX = 0; // b:9 Receiver mode
//	I2caRegs.I2CMDR.bit.IRS = 1; // b:5 Reset I2C bus.
	if( I2C_Wait() == FALSE )
	{
		bI2cWorking = FALSE;
		return FALSE;
	}

	//+ FIFO 수신
	while(I2caRegs.I2CFFRX.bit.RXFFST != 0)
	{
		if(I2cIndex >= 10)
			break;

		temp = I2caRegs.I2CDRR & 0x7F;
		switch(I2cIndex++)
		{
			case 0: // second
				//-rtcBuffer.bUpdating = TRUE;
				rtcBuffer.Second = (temp & 0x0f) + (((temp & 0x70)>> 4) * 10);
				break;
			case 1: // minutes
				rtcBuffer.Minute = (temp & 0x0f) + (((temp & 0x70)>> 4) * 10);
				break;
			case 2: // hours
				rtcBuffer.Hour = (temp & 0x0f) + (((temp & 0x30)>> 4) * 10);

				break;
			case 3: // day
				//dummy = (temp & 0x07);
				break;
			case 4: // date
				rtcBuffer.Date = (temp & 0x0f) + (((temp & 0x30) >> 4) * 10);
				break;
			case 5: // month
				rtcBuffer.Month = (temp & 0x0f) + (((temp & 0x10) >> 4) * 10);

				break;
			case 6: // year
				rtcBuffer.Year = (temp & 0x0f) + (((temp)>> 4) * 10) + 2000;

				//-rtcBuffer.bUpdating = FALSE;
				rtcBufferShadow = rtcBuffer;
				break;
		}
	}

	if(rtcBufferShadow.Second > 59)
	{
		bInit = TRUE;
	}
	if(rtcBufferShadow.Minute > 59)
	{
		bInit = TRUE;
	}
	if(rtcBufferShadow.Hour > 23)
	{
		bInit = TRUE;
	}
	if(rtcBufferShadow.Date > 31)
	{
		bInit = TRUE;
	}
	if(rtcBufferShadow.Month > 12)
	{
		bInit = TRUE;
	}
	if(rtcBufferShadow.Year > 2099 )
	{
		bInit = TRUE;
	}

	if( bInit)
	{
		rtcBufferShadow.Year = 2000;
		rtcBufferShadow.Month = 1;
		rtcBufferShadow.Date = 1;
		rtcBufferShadow.Hour = 1;
		rtcBufferShadow.Minute = 1;
		rtcBufferShadow.Second = 1;

		// for SET RTC TIME
		bI2cWorking = FALSE;
		I2C_SET_RTC_Time(&rtcBufferShadow);
	}

	*pTime = rtcBufferShadow;

	bI2cWorking = FALSE;
	return TRUE;

}


/*
 * Temp 요청
 *
 */
Bool I2C_GET_Temp(UInt16* pRetData)
{
	int I2cIndex = 0;                   // Reset value for ISR.
	InData[0] = InData[1] = InData[2] = InData[3] = InData[4]= InData[5]=InData[6]=InData[7]=0;

	if( bI2cWorking )
		return FALSE;

	bI2cWorking = TRUE;

	I2caRegs.I2CSAR = I2C_SLAVE_ADDR_TEMP;		// Slave Address.
	I2caRegs.I2CMDR.bit.IRS = 0;	//
	I2caRegs.I2CMDR.bit.IRS = 1;	//

	// Slave Address info gets passed with Start Condition
	I2caRegs.I2CCNT = 1; 				// 1 Additional Byte being tranferred. 141230:2->1
	I2caRegs.I2CDXR = 0x5;     			// Send Register to be updated.
	//I2caRegs.I2CDXR = 0x31;     		// Send Register to be updated. 141230:delete
	//I2caRegs.I2CMDR.all = 0x6620;   	// Set up the control register: 141230:6E20->6620
	I2caRegs.I2CMDR.bit.BC = 0;  	// [2:0] Bit count	0: 8bit
	I2caRegs.I2CMDR.bit.FDF = 0;	// [3]Free data format

	I2caRegs.I2CMDR.bit.STB = 0;    // [4]Start byte. Master일 때 start dummy를 보낸다.
	I2caRegs.I2CMDR.bit.IRS = 1;	// [5]I2C Reset not. I2C 활성화.
	I2caRegs.I2CMDR.bit.DLB = 0;	// [6]Digital loopback
	I2caRegs.I2CMDR.bit.RM = 0;		// [7]Repeat mode . FIFO모드에서 FIFO에 값이 없을 때까지 씀. CNT값은 무시.

	I2caRegs.I2CMDR.bit.XA = 0;		// [8]Expand address
	I2caRegs.I2CMDR.bit.TRX = 1;	// [9]Transmitter/reciever 1:Trans
	I2caRegs.I2CMDR.bit.MST = 1;	// [10]Master/slave		 1:Master
	I2caRegs.I2CMDR.bit.STP = 0;	// [11]Stop condition		0 마스터 모드 일 때 STOP상태(P)를 발생 시킴.

	I2caRegs.I2CMDR.bit.rsvd1 = 0;	// [12]reserved			0
	I2caRegs.I2CMDR.bit.STT = 1;	// [13]Start condition		1 마스터 모드 일 때 Start상태(S)를 발생 시킴.
	I2caRegs.I2CMDR.bit.FREE = 1;	// [14]Emulation mode		1 디버깅시에 I2C는 자유롭게 동작.
	I2caRegs.I2CMDR.bit.NACKMOD = 0;// [15]No Ack mode			0 수신 모드 일 때 0이면 보낼 때 마다 ack를 발생 시키고 CNT가 0일 떄 NACK 발생.
																// 1일 경우 다음 확인주기에 NACK를 보내고 0로 clear됨.


	DELAY_US(50);                   // Delay (50 ~ 10)
	//while( I2caRegs.I2CSTR.bit.ARDY == 0){};

	I2caRegs.I2CCNT = 2;            // Set up receive of 2 bytes.
	//I2caRegs.I2CMDR.all = 0x6C20;	// Send "repeated" Start with Read (TRX off) 141230:6620-> 6C20
	I2caRegs.I2CMDR.bit.BC = 0;  	// [2:0] Bit count	0: 8bit
	I2caRegs.I2CMDR.bit.FDF = 0;	// [3]Free data format

	I2caRegs.I2CMDR.bit.STB = 0;    // [4]Start byte
	I2caRegs.I2CMDR.bit.IRS = 1;	// [5]I2C Reset not
	I2caRegs.I2CMDR.bit.DLB = 0;	// [6]Digital loopback
	I2caRegs.I2CMDR.bit.RM = 0;		// [7]Repeat mode

	I2caRegs.I2CMDR.bit.XA = 0;		// [8]Expand address
	I2caRegs.I2CMDR.bit.TRX = 0;	// [9]Transmitter/reciever 0:Recv
	I2caRegs.I2CMDR.bit.MST = 1;	// [10]Master/slave		 1:Master
	I2caRegs.I2CMDR.bit.STP = 1;	// [11]Stop condition	 1: SR

	I2caRegs.I2CMDR.bit.rsvd1 = 0;	// [12]reserved			0
	I2caRegs.I2CMDR.bit.STT = 1;	// [13]Start condition
	I2caRegs.I2CMDR.bit.FREE = 1;	// [14]Emulation mode		1
	I2caRegs.I2CMDR.bit.NACKMOD = 0;// [15]No Ack mode			0

	if( I2C_Wait() == FALSE )
	{
		bI2cWorking = FALSE;
		return FALSE;
	}


	//+ FIFO 수신
	while(I2caRegs.I2CFFRX.bit.RXFFST != 0)
	{
		if(I2cIndex++ >= 5)
			break;

		if( I2cIndex == 1 ) // MSB
			*pRetData = (I2caRegs.I2CDRR & 0xFF) << 8;
		else if( I2cIndex == 2 ) // LSB
			*pRetData |= (I2caRegs.I2CDRR & 0xFF);
	}

	I2cIndex = *pRetData;

	*pRetData = 0;

	// 소수점 Bit0~3은 무시.

	if( I2cIndex & 0x10 ) // Bit4
		*pRetData += 1;
	if( I2cIndex & 0x20 ) // Bit5
		*pRetData += 2;
	if( I2cIndex & 0x40 ) // Bit6
		*pRetData += 4;
	if( I2cIndex & 0x80 ) // Bit7
		*pRetData += 8;
	if( I2cIndex & 0x100 ) // Bit8
		*pRetData += 16;
	if( I2cIndex & 0x200 ) // Bit9
		*pRetData += 32;
	if( I2cIndex & 0x400 ) // Bit10
		*pRetData += 64;
	if( I2cIndex & 0x800 ) // Bit11
		*pRetData += 128;

	if( I2cIndex & 0x1000 ) // Bit12 Sign
			*pRetData = (unsigned short)( ((int)*pRetData) * -1 );


	bI2cWorking = FALSE;
	return TRUE;

}


interrupt void i2c_int1a_isr(void)     // I2C-A
{
	Uint16 IntSource;

	// Read interrupt source
	IntSource = I2caRegs.I2CISRC.bit.INTCODE & 0x7;

	switch(IntSource)
	{
	case I2C_NO_ISRC:   // =0
		break;

	case I2C_ARB_ISRC:  // =1
		break;

	case I2C_NACK_ISRC: // =2
		break;

	case I2C_ARDY_ISRC: // =3
	  if(I2caRegs.I2CSTR.bit.NACK == 1)
	  {
		 I2caRegs.I2CMDR.bit.STP = 1;
		 I2caRegs.I2CSTR.all = I2C_CLR_NACK_BIT;
	  }
		break;

	case I2C_RX_ISRC:   // =4
#if DEBUG_MODE == 1
		//-temp = I2caRegs.I2CDRR & 0x7F;
		error(); // 수신 인터럽트 사용 안함.
#endif
		break;

	case I2C_TX_ISRC:   // =5
		break;

	case I2C_SCD_ISRC:  // =6
		break;

	case I2C_AAS_ISRC:  // =7
		break;
#if DEBUG_MODE == 1
	default:
		asm("   ESTOP0"); // Halt on invalid number.
#endif
	}

	// Enable future I2C (PIE Group 8) interrupts
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

