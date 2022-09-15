/******************************************************************************
 * Destin Power
 *
 * 파일명    : MCU_BSP_Init.c
 * 작성자    : 신요섭
 * 목적      : TI 제공 BSP Source 초기화 매니져
 * 사용방식  : Public 함수 호출
 * 사용파일  : Manager_DP_Source.c
 * 제한사항  :
 * 오류처리  :
 * 이력사항
 * 			2012. 11/01 신요섭
 * 			1. 최초 생성
 */

#include "MCU_BSP_Init.h"
#include "LGS_Common.h"
#include "DSP28x_Project.h"
#include <ti/sysbios/knl/Clock.h>
#include "I2C.h"
#include "SCI.h"
#include "EPWM.h"
#include "SPIa.h"
#include "SPId.h"
#include "McBSP_SPI.h"
#include "XINTF_ADC.h"
#include "CAN.h"
#include "CAN_GBI.h"
#include "Modbus232.h"

#include "Timer_ISR.h"

#if DEBUG_MODE == 1
#include "DBUG.h"
#endif

private void MBI_InitGpio(void);
extern unsigned int hwi_vec_loadstart;
extern unsigned int hwi_vec_loadend;
extern unsigned int hwi_vec_runstart;

extern unsigned int econst_loadstart;
extern unsigned int econst_loadend;
extern unsigned int econst_runstart;

public void MBI_Create(void)
{
	DINT;
	IER = 0x0000;
	IFR = 0x0000;

	//-InitSysCtrl(); BIOS 설정으로 수행
	InitPeripheralClocks();
	MBI_InitGpio();

	InitPieCtrl(); // BIOS의 HWI 설정으로 I2C 인터럽트가 수행되지 않아서, BSP code 이용.
	//-InitPieVectTable();

	//==================================================================
	EALLOW;
	/* for I2C Enable */
	PieVectTable.I2CINT1A = &i2c_int1a_isr;
	/* for SCI Enable */
#if USE_SCI_MODE == 1
	PieVectTable.SCITXINTA = &SCITXINTA_ISR;
	PieVectTable.SCIRXINTA = &SCIRXINTA_ISR;
#else
	PieVectTable.SCITXINTB = &SCITXINTA_ISR;
	PieVectTable.SCIRXINTB = &SCIRXINTA_ISR;
#endif
	/* for EPWM Enable */
	PieVectTable.EPWM1_INT = &EPWM1_INT_ISR;
#if DOUBLE_CONTROL == 1
	PieVectTable.EPWM2_INT = &EPWM2_INT_ISR;
#endif
	/* for Timer ISR */
	PieVectTable.TINT0 = &cpu_timer0_isr;
	EDIS;

	//==================================================================
	XINTF_ADC_Create(); // XINF INIT
	EPWM_Init();
#if 0 //by JCNET
	EPWM_Disable();
#else
    EPWM_Converter_Disable();
    EPWM_Inverter_Disable();
#endif
	I2C_Init();
	SPIA_init();
	SPID_init();
	MCBSP_SPI_init();
	SCI_create(SCI_RS232, SCI_BAUD,0);

	Timer_ISR_init();

	//==================================================================
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;      // EPWM1 interrupt Enable
#if DOUBLE_CONTROL == 1
	PieCtrlRegs.PIEIER3.bit.INTx2 = 1;      // EPWM2 interrupt Enable
#endif
	PieCtrlRegs.PIEIER8.bit.INTx1 = 1;		// Enable I2C interrupt 1 in the PIE: Group 8 interrupt 1
	PieCtrlRegs.PIEIER9.bit.INTx4 = 1;		// PIE 인터럽트(SCITXINT B) : Enable, data manual 50page
	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;		// PIE 인터럽트(SCIRXINT B) : Enable
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	IER |= (M_INT3 | M_INT8 | M_INT9 | M_INT1);		// Enable CPU INT
//    IER |= (M_INT3 | M_INT8 | M_INT9 );     // Enable CPU INT
	bUpdateFlag = TRUE; // 업데이트 대기 flag

	CAN_Create();
	CAN_GBI_Create();

#if DEBUG_MODE == DEBUG_DEVELOP
	ERTM;   // Enable Global realtime interrupt DBGM
#else

#endif

	//-	EINT;
}

private void MBI_InitGpio(void)
{
	/*===================================================================*/
	//EPWM
	/*===================================================================*/
	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio(); // <--- 2 Level

#if IGBT_LEVEL == 3
	InitEPwm4Gpio();
	InitEPwm5Gpio();
	InitEPwm6Gpio(); // <--- 3 Level
#endif

#if STATIC_SWITCH_GPIO_PWM6_9 != 0
	InitEPwm6Gpio(); // <--- 3 Level
	InitEPwm7Gpio(); // Not Used
	InitEPwm8Gpio(); // Not Used
#endif

#if !STATIC_SWITCH_GPIO
	InitEPwm9Gpio();
#endif

	EALLOW;
	// PWM SYNC ( 62번 GPIO를 통해 PWM Sync Signal을 전송 )
	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;   	// Enable pull-up on GPIO27
	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0; 	// Configure GPIO27 as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO62 = 1;  	// GPIO27 is output
	GpioDataRegs.GPBDAT.bit.GPIO62 = 0;

#if STATIC_SWITCH_GPIO
	// GPIO 63번을 Static Switch 사용
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;   	// Enable pull-up on GPIO27
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0; 	// Configure GPIO27 as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO63 = 1;  	// GPIO27 is output
	GpioDataRegs.GPBDAT.bit.GPIO63 = 0;

#if 0
	//6 7 8 9 10 11 -> New Static Switch
	GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
	GpioDataRegs.GPADAT.bit.GPIO6 = 0;

	GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;
	GpioDataRegs.GPADAT.bit.GPIO7 = 0;

	GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;
	GpioDataRegs.GPADAT.bit.GPIO8 = 0;

	GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
	GpioDataRegs.GPADAT.bit.GPIO9 = 0;

	GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
	GpioDataRegs.GPADAT.bit.GPIO10 = 0;

	GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
	GpioDataRegs.GPADAT.bit.GPIO11 = 0;
#endif
#endif
	EDIS;


	/*===================================================================*/
	// ADC MUX
	/*===================================================================*/
#if MCU_PCB_REV_VER >= MCU_PCB_REV_B_G
	EALLOW;
	//MUX Enable
	GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;
	GpioDataRegs.GPASET.bit.GPIO14 = 1; // Load output latch
	GpioDataRegs.GPADAT.bit.GPIO14 = 0; // Active Low : Enable

	//MUX SEL. HIGH: SxA ON      Low: SxB ON
	GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;
	GpioDataRegs.GPASET.bit.GPIO15 = 1; // Load output latch
	GpioDataRegs.GPADAT.bit.GPIO15 = 1; // Active High
	EDIS;

	/*
	 * Control comment
	 * sequence :
	 * 1. Enable MUX(gpio 14)
	 * 2. Mux sel : active high
	 * 3. soc
	 * 4. adc get data, IN VT, IN VS, IN VR    OUT IR, OUT IS, OUT IT
	 * 5. gpio15 toggle in cc_int()
	 * 6. soc
	 * 7. adc get data, GEN VT, GEN VS, GEN VR     DCLINK VP, BAT VP, BAT I
	 * 8. gpio15 toggle in cc_int()
	 * 9. goto line 3
	 */
#endif

	/*===================================================================*/
	// I2C
	/*===================================================================*/
	EALLOW;
	/* Enable internal pull-up for the selected pins */
	// Pull-ups can be enabled or disabled disabled by the user.
	// This will enable the pullups for the specified pins.
	GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;    // Enable pull-up for GPIO32 (SDAA)
	GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;	   // Enable pull-up for GPIO33 (SCLA)

	/* Set qualification for selected pins to asynch only */
	// This will select asynch (no qualification) for the selected pins.
	GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;  // Asynch input GPIO32 (SDAA)
	GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;  // Asynch input GPIO33 (SCLA)

	// This specifies which of the possible GPIO pins will be I2C functional pins.
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;   // Configure GPIO32 for SDAA operation
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;   // Configure GPIO33 for SCLA operation

	//CS - RTC
	GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO43 = 1;
	GpioDataRegs.GPBDAT.bit.GPIO43 = 1;

	EDIS;

	/*===================================================================*/
	//SCI
	/*===================================================================*/
	InitScibGpio();

	/*===================================================================*/
	//SPI A - BOOT FLASH, DATA FLASH
	/*===================================================================*/
	EALLOW;
	//SPISIMOA
	GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   	// Enable pull-up on GPIO16 (SPISIMOA)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; 	// Asynch input GPIO16 (SPISIMOA)
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; 	// Configure GPIO16 as SPISIMOA

	//SPISOMIA
	GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   	// Enable pull-up on GPIO17 (SPISOMIA)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; 	// Asynch input GPIO17 (SPISOMIA)
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; 	// Configure GPIO17 as SPISOMIA

	//SPICLKA
	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   	// Enable pull-up on GPIO18 (SPICLKA)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; 	// Asynch input GPIO18 (SPICLKA)
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; 	// Configure GPIO18 as SPICLKA

	//(CS) - BOOT FLASH
	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   	// Enable pull-up on GPIO19 (SPISTEA)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; 	// Asynch input GPIO19 (SPISTEA)
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0; 	// Configure GPIO19 as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;  	// GPIO19 is output
	GpioDataRegs.GPADAT.bit.GPIO19 = 1; 	//+

#if	MCU_PCB_REV_VER < MCU_PCB_REV_A_F
	//(CS) - DATA FLASH
	GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;   	// Enable pull-up on GPIO27
	GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 0; 	// Asynch input GPIO27 (CS)
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0; 	// Configure GPIO27 as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;  	// GPIO27 is output
	GpioDataRegs.GPADAT.bit.GPIO27 = 1; 	//+
#else
	//(CS) - DATA FLASH - GPIO 40으로 개정 됨 (핀 번호만 40으로 수정되며 기존과 같이 GPIO 모드로 CS 사용)
	GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;   	// Enable pull-up on GPIO27
	GpioCtrlRegs.GPBQSEL1.bit.GPIO40 = 0; 	// Asynch input GPIO27 (CS)
	GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0; 	// Configure GPIO27 as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;  	// GPIO27 is output
	GpioDataRegs.GPBDAT.bit.GPIO40 = 1; 	// +
#endif

	//(WP) - DATA FLASH
	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;   	// Enable pull-up on GPIO29
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0; 	// Configure GPIO29 as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;  	// GPIO29 is output
	GpioDataRegs.GPADAT.bit.GPIO29 = 0; 	// GPIO29 (/WP) - PROTECT ON

	//(WP) - BOOT FLASH
	GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0;   	// Enable pull-up on GPIO39
	GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0; 	// Configure GPIO39 as GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;  	// GPIO39 is output
	GpioDataRegs.GPBDAT.bit.GPIO39 = 0; 	// GPIO39 (/WP) - PROTECT ON
	EDIS;

	/*===================================================================*/
	// SPI D
	/*===================================================================*/
	/* DIGITAL OUT GPIO SETTING -----------------------------------------*/
	EALLOW;
	//CS A
	GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO54 = 1;
	GpioDataRegs.GPBDAT.bit.GPIO54 = 1;

	//CS B
	GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO55 = 1;
	GpioDataRegs.GPBDAT.bit.GPIO55 = 1;

	//MOSI
	GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;  // Enable pull-up
	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 3; // Configure GPIO19 as MOSI
	GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 3; // Asynch input

	//CLk
	GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;  // Enable pull-up
	GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 3; // Configure GPIO19 as SPISTEA
	GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 3; // Asynch input

	//DIAG - ISOFACE - A - OUT
	GpioCtrlRegs.GPBPUD.bit.GPIO41 = 0;  // Enable pull-up
	GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0; // Configure GPIO19
	GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0;
	GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 3; // Asynch input

	//DIAG - ISOFACE - B - OUT
	GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0;  // Enable pull-up
	GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0; // Configure GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO42 = 0;
	GpioCtrlRegs.GPBQSEL1.bit.GPIO42 = 3; // Asynch input GPIO42
	/*-------------------------------------------------------------------*/

	/* DIGITAL IN GPIO SETTING -----------------------------------------*/
	//CS A
	GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;
	GpioDataRegs.GPBDAT.bit.GPIO51 = 1;
	//CS B
	GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;
	GpioDataRegs.GPBDAT.bit.GPIO52 = 1;
	//CS C
	GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO53 = 1;
	GpioDataRegs.GPBDAT.bit.GPIO53 = 1;

	//MISO
	GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 3;
	//-GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 3; // async input

	//CLK
	GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 3;
	//-GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 3; // async input

	//DIAG A
	GpioCtrlRegs.GPCPUD.bit.GPIO81 = 0;
	GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 0;
	GpioCtrlRegs.GPCDIR.bit.GPIO81 = 0;

	//DIAG B
	GpioCtrlRegs.GPCPUD.bit.GPIO82 = 0;
	GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;
	GpioCtrlRegs.GPCDIR.bit.GPIO82 = 0;

	//DIAG C
	GpioCtrlRegs.GPCPUD.bit.GPIO83 = 0;
	GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 0;
	GpioCtrlRegs.GPCDIR.bit.GPIO83 = 0;
	/*-------------------------------------------------------------------*/
	EDIS;

	/*===================================================================*/
	//McBSP B - SRAM(23K256T-I/SN)
	/*===================================================================*/
	EALLOW;
#if McBSP_SPI_GPIO_MODE == 1
	//SPISIMOA
	GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;
	GpioDataRegs.GPADAT.bit.GPIO24 = 0;

	//SPISOMIA
	GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;
	GpioDataRegs.GPADAT.bit.GPIO25 = 0;

	//SPICLKA
	GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;
	GpioDataRegs.GPADAT.bit.GPIO26 = 0;

	//SPISTEA
	GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;
	GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;
	GpioDataRegs.GPBDAT.bit.GPIO40 = 1;
#else
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 3;	// GPIO24 is MDXB pin (Comment as needed)
	GpioCtrlRegs.GPAPUD.bit.GPIO24 = 1;	    // Disable pull-up on GPIO24 (MDXB) (Comment as needed)

	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 3;	// GPIO25 is MDRB pin (Comment as needed)
	GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;	    // Enable pull-up on GPIO25 (MDRB) (Comment as needed)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 3;   // Asynch input GPIO25 (MDRB) (Comment as needed)

	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 3;	// GPIO26 is MCLKXB pin (Comment as needed)
	GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;	    // Enable pull-up on GPIO26 (MCLKXB) (Comment as needed)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 3;   // Asynch input GPIO26(MCLKXB) (Comment as needed)

#if	MCU_PCB_REV_VER < MCU_PCB_REV_A_F
	GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;	// Chip select for GPIO
	GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;	    // Enable pull-up
	GpioCtrlRegs.GPBQSEL1.bit.GPIO40 = 3;   // Asynch input
	GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;  	// output
	GpioDataRegs.GPBSET.bit.GPIO40 = 1;
#else
	// GPIO 40 => 27로 수정 되고, MFSXB모드로 사용할 경우.
	#if MCBSP_B_CS_USE_MFSX
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 3;	// MFSXB *Bug Fixed. 140203 2에서 3으로 수정. 2는 EQEP임.
	GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;	    // Enable pull-up
	//-GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 3;   // Asynch input GPIO27 (MFSXA)
	#else
	// GPIO 모드 CS로 사용할 경우
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;	// Chip select for GPIO
	GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;	    // Enable pull-up
	GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 3;   // Asynch input
	GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;  	// output
	GpioDataRegs.GPASET.bit.GPIO27 = 1;
	#endif // MCBSP_B_CS_USE_MFSX
#endif // MCU_PCB_REV_VER
#endif // McBSP_SPI_GPIO_MODE
	EDIS;

	/*===================================================================*/
	//External Interface(XINTF) ADC
	/*===================================================================*/
	EALLOW;

	GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;	/* XD15 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;	/* XD14 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3;	/* XD13 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 3;	/* XD12 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 3;	/* XD11 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 3;	/* XD10 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 3;	/* XD19 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 3;	/* XD8 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 3;	/* XD7 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 3;	/* XD6 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 3;	/* XD5 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 3;	/* XD4 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 3;	/* XD3 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 3;	/* XD2 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 3;	/* XD1 */
	GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 3;	/* XD0 */

	// 주소 접근 시, 자동으로 Chip Selection이 되므로, GPIO 모드로 구현하지 않아도 된다.
	GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 3;	/*XZC0 ADC - A*/
	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;	/*XZC6 ADC - B*/
	GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 3;	/*XZC7 ADC - C*/

	GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 0;	/* ADC - A BUSY Input. B와 동일한 line. 같은 AD 칩이므로 A상태만 확인한다. */
	GpioCtrlRegs.GPBDIR.bit.GPIO46 = 0;

	GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 0;	/* ADC - C BUSY Input.  */
	GpioCtrlRegs.GPCDIR.bit.GPIO80 = 0;

	GpioCtrlRegs.GPBPUD.bit.GPIO47 = 0; 	/* ADC - A,B,C Reset */
	GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO47 = 1;
	GpioDataRegs.GPBDAT.bit.GPIO47 = 0;

	EDIS;

	/*===================================================================*/
	// LED SET-UP
	/*===================================================================*/
	EALLOW;                              // MMR 보호 영역 해제
	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
	GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0; 	// enable pull-up
	GpioDataRegs.GPBSET.bit.GPIO34 = 1;   // Load output latch
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;  	// PIN Direction: OUT
	GpioDataRegs.GPBDAT.bit.GPIO34 = 0; 	// DATA

	GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;
	GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0; 	// enable pull-up
	GpioDataRegs.GPBSET.bit.GPIO35 = 1;   // Load output latch
	GpioCtrlRegs.GPBDIR.bit.GPIO35 = 1;  	// PIN Direction: OUT
	GpioDataRegs.GPBDAT.bit.GPIO35 = 0; 	// DATA
	EDIS;
}
