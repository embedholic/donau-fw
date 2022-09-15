/*
 * McBSP_SPI.c (NvSRAM)
 *
 *  Created on: 2013. 1. 11.
 *      Author: destinPower
 */

#include "McBSP_SPI.h"
#include "DSP28x_Project.h"
#include <ti/sysbios/knl/Task.h>

void MCBSP_SPI_WREN();
void MCBSP_SPI_WRDIS();
void MCBSP_SPI_statusWrite(int value);

void MCBSP_SPI_init(void)
{
	int i;
	// McBSP-A register settings
	McbspbRegs.SPCR2.all=0x0000;		// Reset FS generator, sample rate generator & transmitter
	McbspbRegs.SPCR1.all=0x0000;		// Reset Receiver, Right justify word, Digital loopback dis.
	//-McbspbRegs.PCR.all=0x0F08;       // (CLKXM=CLKRM=FSXM=FSRM= 1, FSXP = 1)

	//-McbspbRegs.SPCR1.bit.DLB = 1; // DEBUG MODE
#if 1 // TRUE
	McbspbRegs.SPCR1.bit.CLKSTP = 3;    // Clock Delay
	//McbspbRegs.SPCR1.bit.CLKSTP = 2;    // No Clock Delay
	McbspbRegs.PCR.bit.CLKXP = 0;		// CPOL = 0, CPHA = 0 rising edge no delay
	McbspbRegs.PCR.bit.CLKRP = 1;		// MRAM에서 0으로도 동작은 됨.
#else // TEST
	McbspbRegs.SPCR1.bit.CLKSTP = 3;    // Clock Delay
	//McbspbRegs.SPCR1.bit.CLKSTP = 2;    // No Clock Delay
	McbspbRegs.PCR.bit.CLKXP = 0;		// CPOL = 0, CPHA = 0 rising edge no delay
	McbspbRegs.PCR.bit.CLKRP = 1;		// MRAM에서 0으로도 동작은 됨.

#endif
#if 1 // TRUE
	McbspbRegs.PCR.bit.CLKXM = 1; 		// MCLKX pin is an output pin
	McbspbRegs.PCR.bit.SCLKME = 0;
	McbspbRegs.PCR.bit.FSXM = 1; 		// FSX(CS) pin is an output ping driven according to the FSGM bit.
	McbspbRegs.PCR.bit.FSRM = 1; 		// +
	McbspbRegs.PCR.bit.FSXP = 1; 		// FSX pin is active low
	McbspbRegs.PCR.bit.CLKRM = 1;       // The sample rate generator clock drives the transmit and receive clocking (CLKRM = CLKXM = 1 in PCR).
#else // MRAM TEST
	McbspbRegs.PCR.bit.CLKXM = 1; 		// MCLKX pin is an output pin
	McbspbRegs.PCR.bit.SCLKME = 0;
	McbspbRegs.PCR.bit.FSXM = 1; 		// FSX(CS) pin is an output ping driven according to the FSGM bit.
	McbspbRegs.PCR.bit.FSRM = 1; 		// + 추가해봄.
	McbspbRegs.PCR.bit.FSXP = 1; 		// FSX pin is active low
	McbspbRegs.PCR.bit.FSRP = 1; 		// FSX pin is active low
	McbspbRegs.PCR.bit.CLKRM = 1;       // The sample rate generator clock drives the transmit and receive clocking (CLKRM = CLKXM = 1 in PCR).
#endif

	McbspbRegs.RCR2.bit.RDATDLY= 1;     // FSX setup time 1 in master mode. 0 for slave mode (Receive)
	McbspbRegs.XCR2.bit.XDATDLY= 1;     // FSX setup time 1 in master mode. 0 for slave mode (Transmit)

	//8bit mode
	McbspbRegs.RCR1.bit.RWDLEN1=0;     	// 0: 8bit, 5:32-bit - Read
	McbspbRegs.XCR1.bit.XWDLEN1=0;     	// 0: 8bit, 5:32-bit - Send

	McbspbRegs.SRGR2.all=0x2000; 	 	// CLKSM=1, FPER = 1 CLKG periods
	McbspbRegs.SRGR2.bit.FSGM = 0;		// The transmitter drives a frame-synchronization pulse on the FSX pin every time data is
										// transferred from DXR1 to XSR1.
	// Frame Width = 1 CLKG period, CLKGDV=16 (*LSPCLK 300 MHz . 20MHz=0xE, 10Mhz = 0x1D,0x12B 1Mhz)
	McbspbRegs.SRGR1.bit.FWID = 0x0;
	//-McbspbRegs.SRGR1.bit.CLKGDV = 0xFF; //-최저 속도.
	//-McbspbRegs.SRGR1.bit.CLKGDV = 0x3B; // 5MHz
#if MCU_PCB_REV_VER >= MCU_PCB_REV_J
	McbspbRegs.SRGR1.bit.CLKGDV = 0xE; // 30MHz
#else
	McbspbRegs.SRGR1.bit.CLKGDV = 0x1D; // 10MHz
#endif
	McbspbRegs.SRGR2.bit.CLKSM = 1;// The input clock for the sample rate generator is taken from the LSPCLK.
	for (i = 0; i < 50; i++) {}

	McbspbRegs.SPCR2.bit.GRST=1;        // Enable the sample rate generator
	McbspbRegs.SPCR2.bit.FREE = 1;
	for (i = 0; i < 50; i++) {}
	McbspbRegs.SPCR2.bit.XRST=1;        // Release TX from Reset
	McbspbRegs.SPCR1.bit.RRST=1;        // Release RX from Reset
	McbspbRegs.SPCR2.bit.FRST=1;        // Frame Sync Generator reset

#if MCU_PCB_REV_VER >= MCU_PCB_REV_J
   MCBSP_SPI_WREN();
   MCBSP_SPI_statusWrite(1);

#endif
}
void MCBSP_SPI_xmit(int a)
{
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
    McbspbRegs.DXR1.all=a;
}
void MCBSP_SPI_read(int* a)
{
	// Master waits until RX data is ready
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {}
    *a = McbspbRegs.DRR1.all;                      // Then read DRR1 to complete receiving of data
}

Bool bReiteration = FALSE;
Bool MCBSP_SPI_byteWrite(UInt16 address,UInt16 data)
{
	//-XXX T int dummy;

#if DEBUG_MODE == 1
	if( bReiteration == TRUE )
	{
		error();
	}
	bReiteration = TRUE;
#endif

#if MCU_PCB_REV_VER >= MCU_PCB_REV_J
   MCBSP_SPI_WREN();
   MCBSP_SPI_statusWrite(1);
#endif

#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPBDAT.bit.GPIO40 == 0)
	{
		Task_sleep(1);
	}

	GpioDataRegs.GPBDAT.bit.GPIO40 = 0; // cs
	DELAY_NS(50);
#else
	//MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPADAT.bit.GPIO27 == 0)
	{
		Task_sleep(1);
	}

	GpioDataRegs.GPADAT.bit.GPIO27 = 0; // cs
	DELAY_NS(50);
	#endif

#endif


#if 0
	// 32bit mode
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR2.all = (0x2<<8&0xFF00) | (address>>8&0x00FF);
	McbspbRegs.DXR1.all = ((address)<<8&0xFF00) | (data & 0xFF);
#else
	//8bit mode
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = (0x2);
#endif

#if MCU_PCB_REV_VER >= MCU_PCB_REV_J
	// 24bit address mode
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = 0;
#endif

	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = (address>>8)&0xFF;

	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = (address)&0xFF;

	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = (data & 0xFF);

	while( McbspbRegs.SPCR2.bit.XEMPTY) {} // wait for any previous SPI transactions to clear

	/*
	 * RFULL 상태를 막기 위해 쓰레기 값을 읽는다.
	 * RFULL 상태가 되면 새로 들어오는 수신 값들이 무시 되기 때문.
	 */
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {} //
	McbspbRegs.DRR1.all;               			// read dummy


#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	DELAY_NS(50);
	GpioDataRegs.GPBDAT.bit.GPIO40 = 1; // cs clear
	DELAY_NS(50);
#else
	//MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	DELAY_NS(50);
	GpioDataRegs.GPADAT.bit.GPIO27 = 1; // cs clear
	DELAY_NS(30);
	#endif
#endif

#if MCU_PCB_REV_VER >= MCU_PCB_REV_J
   MCBSP_SPI_WRDIS();
#endif

	bReiteration = FALSE;
	return TRUE;
}

Bool MCBSP_SPI_byteRead(UInt16 address,UInt16* data)
{
#if DEBUG_MODE == 1
	if( bReiteration == TRUE )
	{
		error();
	}
	bReiteration = TRUE;
#endif

#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPBDAT.bit.GPIO40 == 0){
		Task_sleep(1);
	}
	GpioDataRegs.GPBDAT.bit.GPIO40 = 0; // cs
	DELAY_NS(50);
#else
	// MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPADAT.bit.GPIO27 == 0){
		Task_sleep(1);
	}
	GpioDataRegs.GPADAT.bit.GPIO27 = 0; // cs
	DELAY_NS(50);
	#endif
#endif

	//32bit mode
	//-while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	//-McbspbRegs.DXR2.all = ((address&0xFF)<<8);
	//-McbspbRegs.DXR1.all = (0x3<<8) | (address>>8);

	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = (0x3);
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {} // FRULL 상태를 막기 위해 쓰레기 값을 읽는다.
	McbspbRegs.DRR1.all;               			// read dummy
#if MCU_PCB_REV_VER >= MCU_PCB_REV_J
	// 24bit address mode
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = 0;
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {}
	McbspbRegs.DRR1.all;
#endif
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {}
	McbspbRegs.DXR1.bit.LWLB = (address>>8)&0xFF;
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {}
	McbspbRegs.DRR1.all;
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {}
	McbspbRegs.DXR1.bit.LWLB = (address&0xFF);
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {}
	McbspbRegs.DRR1.all;
	// send dummy
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {}
	McbspbRegs.DXR1.bit.LWLB = 0xFF;

	while(McbspbRegs.SPCR2.bit.XEMPTY){};
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {}	// 실제 데이터를 기다린다.
	*data = McbspbRegs.DRR1.all &0xFF;			// 데이터를 읽는다.

	while(McbspbRegs.SPCR2.bit.XEMPTY){};

#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	GpioDataRegs.GPBDAT.bit.GPIO40 = 1;
	DELAY_NS(50);
#else
	//MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	GpioDataRegs.GPADAT.bit.GPIO27 = 1;
	DELAY_NS(50);
	#endif
#endif
	bReiteration = FALSE;
	return TRUE;
}

void MCBSP_SPI_WREN()
{
	//-XXX T int data;
#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPBDAT.bit.GPIO40 == 0){
		Task_sleep(1);
	}
	GpioDataRegs.GPBDAT.bit.GPIO40 = 0; // cs
	DELAY_NS(50);
#else
	// MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPADAT.bit.GPIO27 == 0){
		Task_sleep(1);
	}
	GpioDataRegs.GPADAT.bit.GPIO27 = 0; // cs
	DELAY_NS(50);
	#endif
#endif

	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = (0x6);
	while(McbspbRegs.SPCR2.bit.XEMPTY){};

	/*
	 * RFULL 상태를 막기 위해 쓰레기 값을 읽는다.
	 * RFULL 상태가 되면 새로 들어오는 수신 값들이 무시 되기 때문.
	 */
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {} //
	McbspbRegs.DRR1.all;               			// read dummy

#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	GpioDataRegs.GPBDAT.bit.GPIO40 = 1;
	DELAY_NS(50);
#else
	//MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	GpioDataRegs.GPADAT.bit.GPIO27 = 1;
	DELAY_NS(50);
	#endif
#endif

	return;
}

void MCBSP_SPI_WRDIS()
{
	//-XXX T int data;
#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPBDAT.bit.GPIO40 == 0){
		Task_sleep(1);
	}
	GpioDataRegs.GPBDAT.bit.GPIO40 = 0; // cs
	DELAY_NS(50);
#else
	// MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPADAT.bit.GPIO27 == 0){
		Task_sleep(1);
	}
	GpioDataRegs.GPADAT.bit.GPIO27 = 0; // cs
	DELAY_NS(50);
	#endif
#endif

	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = (0x4);
	while(McbspbRegs.SPCR2.bit.XEMPTY){};

	/*
	 * RFULL 상태를 막기 위해 쓰레기 값을 읽는다.
	 * RFULL 상태가 되면 새로 들어오는 수신 값들이 무시 되기 때문.
	 */
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {} //
	McbspbRegs.DRR1.all;               			// read dummy

#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	GpioDataRegs.GPBDAT.bit.GPIO40 = 1;
	DELAY_NS(50);
#else
	//MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	GpioDataRegs.GPADAT.bit.GPIO27 = 1;
	DELAY_NS(50);
	#endif
#endif

	return;
}
void MCBSP_SPI_statusWrite(int value)
{

#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPBDAT.bit.GPIO40 == 0)
	{
		Task_sleep(1);
	}

	GpioDataRegs.GPBDAT.bit.GPIO40 = 0; // cs
	DELAY_NS(50);
#else
	//MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPADAT.bit.GPIO27 == 0)
	{
		Task_sleep(1);
	}

	GpioDataRegs.GPADAT.bit.GPIO27 = 0; // cs
	DELAY_NS(50);
	#endif

#endif

	//8bit mode
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = (0x1);
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {} //
	McbspbRegs.DRR1.all;               			// read dummy

#if MCU_PCB_REV_VER >= MCU_PCB_REV_J
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = value&0xFF;
#endif

	while( McbspbRegs.SPCR2.bit.XEMPTY) {} // wait for any previous SPI transactions to clear

	/*
	 * RFULL 상태를 막기 위해 쓰레기 값을 읽는다.
	 * RFULL 상태가 되면 새로 들어오는 수신 값들이 무시 되기 때문.
	 */
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {} //
	McbspbRegs.DRR1.all;               			// read dummy

#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	DELAY_NS(50);
	GpioDataRegs.GPBDAT.bit.GPIO40 = 1; // cs clear
	DELAY_NS(50);
#else
	//MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	DELAY_NS(50);
	GpioDataRegs.GPADAT.bit.GPIO27 = 1; // cs clear
	DELAY_NS(50);
	#endif
#endif

	return;
}

/*
 * FIXME 검증 안됨. 이 함수를 사용할 경우 Read function 읽기 실패 할 수 있음.
 * - 사용 안됨.
 */
int MCBSP_SPI_statusRead()
{
	int data;
#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPBDAT.bit.GPIO40 == 0){
		Task_sleep(1);
	}
	GpioDataRegs.GPBDAT.bit.GPIO40 = 0; // cs
	DELAY_NS(50);
#else
	// MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	// CS 중이면 다른 작업이 종료될 때까지 대기.
	while( GpioDataRegs.GPADAT.bit.GPIO27 == 0){
		Task_sleep(1);
	}
	GpioDataRegs.GPADAT.bit.GPIO27 = 0; // cs
	DELAY_NS(50);
	#endif
#endif

	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {} // wait for any previous SPI transactions to clear
	McbspbRegs.DXR1.bit.LWLB = (0x5);
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {} // FRULL 상태를 막기 위해 쓰레기 값을 읽는다.
	McbspbRegs.DRR1.all;               			// read dummy

	// send dummy
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {}
	McbspbRegs.DXR1.bit.LWLB = 0xFF;
	while( McbspbRegs.SPCR2.bit.XRDY == 0 ) {}

	while(McbspbRegs.SPCR2.bit.XEMPTY){};
	while( McbspbRegs.SPCR1.bit.RRDY == 0 ) {}	// 실제 데이터를 기다린다.
	data = McbspbRegs.DRR1.all &0xFF;				// 데이터를 읽는다.

	//-DELAY_NS(50);
	while(McbspbRegs.SPCR2.bit.XEMPTY){};

#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	GpioDataRegs.GPBDAT.bit.GPIO40 = 1;
	DELAY_NS(50);
#else
	//MFSXB
	#if MCBSP_B_CS_USE_MFSX == 0
	GpioDataRegs.GPADAT.bit.GPIO27 = 1;
	DELAY_NS(50);
	#endif
#endif

	return data;
}



#if McBSP_SPI_GPIO_MODE == 1

#define SPISCK_CLE() GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
#define SPISCK_SET() GpioDataRegs.GPASET.bit.GPIO26 = 1;
#define SIMO_CLE() GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;
#define SPI_CS_SET() GpioDataRegs.GPBDAT.bit.GPIO40 = 0;
#define SPI_CS_CLE() GpioDataRegs.GPBDAT.bit.GPIO40 = 1;

#define MOSI GpioDataRegs.GPADAT.bit.GPIO24
#define MISO GpioDataRegs.GPADAT.bit.GPIO25


public void MCBSP_SPI_init(void)
{
}

public void MCBSP_SPI_xmit(int a)
{
}

public void MCBSP_SPI_read(int* a)
{

}

private void MCBSP_SPI_CLOCK()
{
	// 16Mhz 정도로 동작 예상.
	DELAY_NS(20);
	SPISCK_SET();
	DELAY_NS(20);
	SPISCK_CLE();
	asm(" nop");
}
private void MCBSP_SPI_CS_ENB()
{
	SPI_CS_SET();
	DELAY_NS(50);
}
private void MCBSP_SPI_CS_DIS()
{
	SIMO_CLE();
	DELAY_NS(45);
	SPI_CS_CLE();
	DELAY_NS(50);
}
public void MCBSP_SPI_byteWrite(UInt16 address,UInt16 data)
{
	Uint16 i;

	MCBSP_SPI_CS_ENB();
	for(i = 8; i >= 1; i--)
	{
		MOSI = ((0x02 & (1<<(i-1))) >> (i-1));
		MCBSP_SPI_CLOCK();
	}
	for(i = 16; i >= 1; i--)
	{
		MOSI = ((address & (1<<(i-1))) >> (i-1));
		MCBSP_SPI_CLOCK();
	}
	for(i = 8; i >= 1; i--)
	{
		MOSI = ((data & (1<<(i-1))) >> (i-1));
		MCBSP_SPI_CLOCK();
	}
	MCBSP_SPI_CS_DIS();
}

public void MCBSP_SPI_byteRead(UInt16 address,UInt16* data)
{
	Uint16 i;

	MCBSP_SPI_CS_ENB();

	for(i = 8; i >= 1; i--)
	{
		MOSI = ((0x03 & (1<<(i-1))) >> (i-1));
		MCBSP_SPI_CLOCK();
	}
	for(i = 16; i >= 1; i--)
	{
		MOSI = ((address & (1<<(i-1))) >> (i-1));
		MCBSP_SPI_CLOCK();
	}
	DELAY_NS(25);
	for(i = 8; i >= 1; i--)
	{
	   *data |= (MISO << (i-1));
	   //-DELAY_NS(25);
	   SPISCK_SET();
	   DELAY_NS(25);
	   SPISCK_CLE();
	   DELAY_NS(25);
	   asm(" nop");
	}
	MCBSP_SPI_CS_DIS();
	return;                 // Then read DRR1 to complete receiving of data
}
#endif
