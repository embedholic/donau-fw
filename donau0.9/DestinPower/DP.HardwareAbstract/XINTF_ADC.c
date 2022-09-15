/*
 * ExADC.c
 *
 *  Created on: 2012. 12. 17.
 *      Author: destinPower
 */

#include "XINTF_ADC.h"
#include "DSP28x_Project.h"

//#define ADC_CH_CNT 22

#define ADC_RESET()	\
			GpioDataRegs.GPBDAT.bit.GPIO47 = 1;\
			DELAY_US(1);\
			GpioDataRegs.GPBDAT.bit.GPIO47 = 0;\

#define ADC_AB_BUSY()	GpioDataRegs.GPBDAT.bit.GPIO46
#define ADC_C_BUSY() 	GpioDataRegs.GPCDAT.bit.GPIO80

#define ADC_A (Uint16 volatile *)0x4000			//Zone 0 address for ADC A
#define ADC_B (Uint16 volatile *)0x100000		//Zone 6 address for ADC B
#define ADC_C (Uint16 volatile *)0x200000		//Zone 7 address for ADC C

volatile int ADC_A_buf[8] = {0};
volatile int ADC_B_buf[8] = {0};
volatile int ADC_C_buf[6] = {0};
#if 0
float fADC_A_res[8] = {0};
float fADC_B_res[8] = {0};
float fADC_C_res[6] = {0};
#endif


private void XINTF_ADC_HW_init();


public Void XINTF_ADC_Create()
{
	XINTF_ADC_HW_init();
	ADC_RESET();
}

private void XINTF_ADC_HW_init()
{
	EALLOW;

	// XCLKOUT to SYSCLKOUT ratio.
	// XTIMCLK = SYSCLKOUT/2 = (150 MHz) : 0과 1만 선택가능. 1 선택시 시스템 클럭 나누기 2.
	XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
	// XCLKOUT = XTIMCLK/2 = (75 MHz)
	XintfRegs.XINTCNF2.bit.CLKMODE = 1;
	// XCLKOUT = XTIMCLK/4 = (37 MHz) - TODO 37Mhz 로 동작 가능한지 ADC 스펙 확인.
	XintfRegs.XINTCNF2.bit.BY4CLKMODE = 1;
	// Enable XCLKOUT - 클럭 오프 해지
	XintfRegs.XINTCNF2.bit.CLKOFF = 0;

	// XRD LEAD,ACTIVE, TRAIN 의 타이밍을 2배로 만든다.
	XintfRegs.XTIMING0.bit.X2TIMING = 1;
	XintfRegs.XTIMING6.bit.X2TIMING = 1;
	XintfRegs.XTIMING7.bit.X2TIMING = 1;

#if MCU_PCB_REV_VER >= MCU_PCB_REV_B_G
	// Zone0 read timing - AD7606
	XintfRegs.XTIMING0.bit.XRDLEAD = 1; 	// 13.32ns
	XintfRegs.XTIMING0.bit.XRDACTIVE = 5; 	// 66.6ns
	XintfRegs.XTIMING0.bit.XRDTRAIL = 0; 	// 0ns
	// Zone6 read timing
	XintfRegs.XTIMING6.bit.XRDLEAD = 1; 	// 13.32ns
	XintfRegs.XTIMING6.bit.XRDACTIVE = 5; 	// 66.6ns
	XintfRegs.XTIMING6.bit.XRDTRAIL = 0; 	// 0ns
#else
	// Zone0 read timing - AD7606
	XintfRegs.XTIMING0.bit.XRDLEAD = 1; 	// 13.32ns
	XintfRegs.XTIMING0.bit.XRDACTIVE = 3; 	// 39.96ns
	XintfRegs.XTIMING0.bit.XRDTRAIL = 0; 	// 0ns
	// Zone6 read timing
	XintfRegs.XTIMING6.bit.XRDLEAD = 1; 	// 13.32ns
	XintfRegs.XTIMING6.bit.XRDACTIVE = 3; 	// 39.96ns
	XintfRegs.XTIMING6.bit.XRDTRAIL = 0; 	// 0ns
#endif
	// Zone7 read timing - AD7656-1
	XintfRegs.XTIMING7.bit.XRDLEAD = 1; 	// 13.32ns
	XintfRegs.XTIMING7.bit.XRDACTIVE = 5; 	// 66.6ns
	XintfRegs.XTIMING7.bit.XRDTRAIL = 0; 	// 0ns

	// ADC에 레이디 신호선이 없으므로 사용 안함 설정
	XintfRegs.XTIMING0.bit.USEREADY = 0;
	XintfRegs.XTIMING0.bit.READYMODE = 0;
	XintfRegs.XTIMING6.bit.USEREADY = 0;
	XintfRegs.XTIMING6.bit.READYMODE = 0;
	XintfRegs.XTIMING7.bit.USEREADY = 0;
	XintfRegs.XTIMING7.bit.READYMODE = 0;

	// To switch to 32-bit writes, make sure to also enable 32-bit XINTF using the XSIZE bits.
	XintfRegs.XTIMING0.bit.XSIZE = 3;	// Size for x16 is 3 , Size for x32 is 1
	XintfRegs.XTIMING6.bit.XSIZE = 3;	// Size for x16 is 3 , Size for x32 is 1
	XintfRegs.XTIMING7.bit.XSIZE = 3;	// Size for x16 is 3 , Size for x32 is 1

	EDIS;
}

//public inline UInt16 XINTF_ADC_GetValue(Uint15 itemId)
//{
//
//}

/*
 * SOC는 PWM count 0이 되었을 때 발생하므로
 * PWM 인터럽트에서 ADC값을 읽어야 함.
 */
public Void XINTF_ADC_READ_ALL()
{
	Uint16 ndx;

	/* PWM에 의해 SOC명령이 전달되고, ADC가 BUSY 상태가 된다.
	 * BUSY 상태로 변화하기전을 대기 후 BUSY상태가 되면 BUSY상태가 종료되는 것을 기다린다.
	 * 1. wait before busy
	 * 2. wait busy
	 * TODO 테스트 필요.
	 */
	//LGS SW에서는 busy 신호가 active된 후 이 루틴에 들어오는 것으로 보인다.
	//while(!ADC_AB_BUSY()) asm(" NOP");
	while(ADC_AB_BUSY()) asm(" NOP");

#if MCU_PCB_REV_VER >= MCU_PCB_REV_B_G
	// ADC-A zone0; AD7656
	for (ndx=0; ndx<6; ndx++)
	{
		ADC_A_buf[ndx] = *ADC_A;
		//-fADC_A_res[ndx] = ADC_A_buf[ndx] * 3.051757e-4; //디버그용 변수
	}

	// ADC-B zone6; ADC7656
	for (ndx=0; ndx<6; ndx++)
	{
		ADC_B_buf[ndx] = *ADC_B;
		//-fADC_B_res[ndx] = (ADC_B_buf[ndx] * 3.051757e-4);
	}
#else
	// ADC-A zone0; AD7606
	for (ndx=0; ndx<8; ndx++)
	{
		ADC_A_buf[ndx] = *ADC_A;
	}

	// ADC-B zone6; ADC7606
	for (ndx=0; ndx<8; ndx++)
	{
		ADC_B_buf[ndx] = *ADC_B;
	}
#endif

	// wait busy
	while(ADC_C_BUSY()) asm(" NOP");

	// ADC-C zone7; ADC7656
	for (ndx=0; ndx<6; ndx++)
	{
		ADC_C_buf[ndx] = *ADC_C;
		//-fADC_C_res[ndx] = ADC_C_buf[ndx] * 3.051757e-4;
	}
}


/* ADC에 입력 되는 실제 전압을 리턴 한다. */
public Uint16 XINTF_ADC_getVLevel(Uint16 adcCh)
{
	return 0;
}

UInt16 bToggle = 1;
public void XINTF_ADC_MUX_TOGGLE()
{
	//-GpioDataRegs.GPATOGGLE.bit.GPIO15 = 1;
	bToggle = !bToggle;
	GpioDataRegs.GPADAT.bit.GPIO15 = bToggle;
}

public Uint16 XINTF_ADC_MUX_STATE()
{
	return bToggle;
}



