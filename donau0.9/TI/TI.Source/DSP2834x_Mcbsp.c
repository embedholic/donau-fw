  // TI File $Revision: /main/3 $
// Checkin $Date: August 1, 2008   16:35:37 $
//###########################################################################
//
// FILE:	DSP2834x_McBSP.c
//
// TITLE:	DSP2834x Device McBSP Initialization & Support Functions.
//
//###########################################################################
// $TI Release: 2834x Header Files V1.12 $
// $Release Date: March 2011 $
//###########################################################################

#include "DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "DSP2834x_Examples.h"   // DSP2834x Examples Include File
//---------------------------------------------------------------------------
// MCBSP_INIT_DELAY determines the amount of CPU cycles in the 2 sample rate
// generator (SRG) cycles required for the Mcbsp initialization routine.
// MCBSP_CLKG_DELAY determines the amount of CPU cycles in the 2 clock
// generator (CLKG) cycles required for the Mcbsp initialization routine.
// For the functions defined in Mcbsp.c, MCBSP_INIT_DELAY and MCBSP_CLKG_DELAY
// are based off of SYSCLKOUT frequency.
//
// CPU_FRQ_300MHZ, CPU_FRQ_250MHz, and CPU_FRQ_200MHZ are defined in DSP2834x_Examples.h
//---------------------------------------------------------------------------

#if CPU_FRQ_300MHZ                                          // For 300 MHz SYSCLKOUT
  #define CPU_SPD              300E6
  #define MCBSP_SRG_FREQ       CPU_SPD/4                    // SRG input is LSPCLK (SYSCLKOUT/4) for examples
#endif
#if CPU_FRQ_250MHZ                                          // For 250 MHz SYSCLKOUT(default)
  #define CPU_SPD              250E6
  #define MCBSP_SRG_FREQ       CPU_SPD/4                    // SRG input is LSPCLK (SYSCLKOUT/4) for examples
#endif
#if CPU_FRQ_200MHZ                                          // For 200 MHz SYSCLKOUT
  #define CPU_SPD              200E6
  #define MCBSP_SRG_FREQ       CPU_SPD/4                    // SRG input is LSPCLK (SYSCLKOUT/4) for examples
#endif

#define CLKGDV_VAL           1
#define MCBSP_INIT_DELAY     2*(CPU_SPD/MCBSP_SRG_FREQ)                  // # of CPU cycles in 2 SRG cycles-init delay
#define MCBSP_CLKG_DELAY     2*(CPU_SPD/(MCBSP_SRG_FREQ/(1+CLKGDV_VAL))) // # of CPU cycles in 2 CLKG cycles-init delay
//---------------------------------------------------------------------------
// InitMcbsp:
//---------------------------------------------------------------------------
// This function initializes the McBSP to a known state.
//

void delay_loop(void);		// Delay function used for SRG initialization
void clkg_delay_loop(void); // Delay function used for CLKG initialization

void InitMcbsp(void)
{
	InitMcbspa();
	#if DSP28_MCBSPB
	  InitMcbspb();
	#endif               // end DSP28_MCBSPB
}

void InitMcbspa(void)
{
// McBSP-A register settings

    McbspaRegs.SPCR2.all=0x0000;		// Reset FS generator, sample rate generator & transmitter
	McbspaRegs.SPCR1.all=0x0000;		// Reset Receiver, Right justify word
	McbspaRegs.SPCR1.bit.DLB = 1;       // Enable loopback mode for test. Comment out for normal McBSP transfer mode.


	McbspaRegs.MFFINT.all=0x0;			// Disable all interrupts

    McbspaRegs.RCR2.all=0x0;			// Single-phase frame, 1 word/frame, No companding	(Receive)
    McbspaRegs.RCR1.all=0x0;

    McbspaRegs.XCR2.all=0x0;			// Single-phase frame, 1 word/frame, No companding	(Transmit)
    McbspaRegs.XCR1.all=0x0;

    McbspaRegs.PCR.bit.FSXM = 1;		// FSX generated internally, FSR derived from an external source
	McbspaRegs.PCR.bit.CLKXM = 1;		// CLKX generated internally, CLKR derived from an external source

    McbspaRegs.SRGR2.bit.CLKSM = 1;		// CLKSM=1 (If SCLKME=0, i/p clock to SRG is LSPCLK)
	McbspaRegs.SRGR2.bit.FPER = 31;		// FPER = 32 CLKG periods

    McbspaRegs.SRGR1.bit.FWID = 0;              // Frame Width = 1 CLKG period
    McbspaRegs.SRGR1.bit.CLKGDV = CLKGDV_VAL;	// CLKG frequency = LSPCLK/(CLKGDV+1)

    delay_loop();                // Wait at least 2 SRG clock cycles

    McbspaRegs.SPCR2.bit.GRST=1; // Enable the sample rate generator
	clkg_delay_loop();           // Wait at least 2 CLKG cycles
	McbspaRegs.SPCR2.bit.XRST=1; // Release TX from Reset
	McbspaRegs.SPCR1.bit.RRST=1; // Release RX from Reset
    McbspaRegs.SPCR2.bit.FRST=1; // Frame Sync Generator reset

}


#if (DSP28_MCBSPB)
void InitMcbspb(void)
{

// McBSP-B register settings

    McbspbRegs.SPCR2.all=0x0000;		// Reset FS generator, sample rate generator & transmitter
	McbspbRegs.SPCR1.all=0x0000;		// Reset Receiver, Right justify word
	McbspbRegs.SPCR1.bit.DLB = 1;       // Enable loopback mode for test. Comment out for normal McBSP transfer mode.

	McbspbRegs.MFFINT.all=0x0;			// Disable all interrupts

    McbspbRegs.RCR2.all=0x0;			// Single-phase frame, 1 word/frame, No companding	(Receive)
    McbspbRegs.RCR1.all=0x0;

    McbspbRegs.XCR2.all=0x0;			// Single-phase frame, 1 word/frame, No companding	(Transmit)
    McbspbRegs.XCR1.all=0x0;

    McbspbRegs.SRGR2.bit.CLKSM = 1;		// CLKSM=1 (If SCLKME=0, i/p clock to SRG is LSPCLK)
	McbspbRegs.SRGR2.bit.FPER = 31;		// FPER = 32 CLKG periods

    McbspbRegs.SRGR1.bit.FWID = 0;              // Frame Width = 1 CLKG period
    McbspbRegs.SRGR1.bit.CLKGDV = CLKGDV_VAL;	// CLKG frequency = LSPCLK/(CLKGDV+1)

   	McbspbRegs.PCR.bit.FSXM = 1;		// FSX generated internally, FSR derived from an external source
	McbspbRegs.PCR.bit.CLKXM = 1;		// CLKX generated internally, CLKR derived from an external source
    delay_loop();                // Wait at least 2 SRG clock cycles
    McbspbRegs.SPCR2.bit.GRST=1; // Enable the sample rate generator
	clkg_delay_loop();           // Wait at least 2 CLKG cycles
	McbspbRegs.SPCR2.bit.XRST=1; // Release TX from Reset
	McbspbRegs.SPCR1.bit.RRST=1; // Release RX from Reset
    McbspbRegs.SPCR2.bit.FRST=1; // Frame Sync Generator reset

}


#endif // end DSP28_MCBSPB

// McBSP-A Data Lengths
void InitMcbspa8bit(void)
{
    McbspaRegs.RCR1.bit.RWDLEN1=0;     // 8-bit word
    McbspaRegs.XCR1.bit.XWDLEN1=0;     // 8-bit word
}

void InitMcbspa12bit(void)
{
    McbspaRegs.RCR1.bit.RWDLEN1=1;     // 12-bit word
    McbspaRegs.XCR1.bit.XWDLEN1=1;     // 12-bit word
}

void InitMcbspa16bit(void)
{
    McbspaRegs.RCR1.bit.RWDLEN1=2;      // 16-bit word
    McbspaRegs.XCR1.bit.XWDLEN1=2;      // 16-bit word
}

void InitMcbspa20bit(void)
{
    McbspaRegs.RCR1.bit.RWDLEN1=3;     // 20-bit word
    McbspaRegs.XCR1.bit.XWDLEN1=3;     // 20-bit word
}

void InitMcbspa24bit(void)
{
    McbspaRegs.RCR1.bit.RWDLEN1=4;     // 24-bit word
    McbspaRegs.XCR1.bit.XWDLEN1=4;     // 24-bit word
}

void InitMcbspa32bit(void)
{
    McbspaRegs.RCR1.bit.RWDLEN1=5;     // 32-bit word
    McbspaRegs.XCR1.bit.XWDLEN1=5;     // 32-bit word
}

// McBSP-B Data Lengths
#if (DSP28_MCBSPB)

void InitMcbspb8bit(void)
{
    McbspbRegs.RCR1.bit.RWDLEN1=0;     // 8-bit word
    McbspbRegs.XCR1.bit.XWDLEN1=0;     // 8-bit word
}

void InitMcbspb12bit(void)
{
    McbspbRegs.RCR1.bit.RWDLEN1=1;     // 12-bit word
    McbspbRegs.XCR1.bit.XWDLEN1=1;     // 12-bit word
}

void InitMcbspb16bit(void)
{
    McbspbRegs.RCR1.bit.RWDLEN1=2;      // 16-bit word
    McbspbRegs.XCR1.bit.XWDLEN1=2;      // 16-bit word
}

void InitMcbspb20bit(void)
{
    McbspbRegs.RCR1.bit.RWDLEN1=3;     // 20-bit word
    McbspbRegs.XCR1.bit.XWDLEN1=3;     // 20-bit word
}

void InitMcbspb24bit(void)
{
    McbspbRegs.RCR1.bit.RWDLEN1=4;     // 24-bit word
    McbspbRegs.XCR1.bit.XWDLEN1=4;     // 24-bit word
}

void InitMcbspb32bit(void)
{
    McbspbRegs.RCR1.bit.RWDLEN1=5;     // 32-bit word
    McbspbRegs.XCR1.bit.XWDLEN1=5;     // 32-bit word
}

#endif //end DSP28_MCBSPB



void InitMcbspGpio(void)
{
	InitMcbspaGpio();
	#if DSP28_MCBSPB
	  InitMcbspbGpio();
	#endif               // end DSP28_MCBSPB
}

void InitMcbspaGpio(void)
{
	EALLOW;

/* Configure McBSP-A pins using GPIO regs */
// This specifies which of the possible GPIO pins will be McBSP functional pins.
// Comment out other unwanted lines.

	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 2;	// GPIO20 is MDXA pin
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 2;	// GPIO21 is MDRA pin
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 2;	// GPIO22 is MCLKXA pin
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 2;		// GPIO7 is MCLKRA pin (Comment as needed)
	//GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 1;	// GPIO58 is MCLKRA pin (Comment as needed)
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 2;	// GPIO23 is MFSXA pin
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 2;		// GPIO5 is MFSRA pin (Comment as needed)
	//GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 1;	// GPIO59 is MFSRA pin (Comment as needed)

/* Enable internal pull-up for the selected I/O pins. Disable
   pull-ups on output-only pins to reduce power consumption */
// Pull-ups can be enabled or disabled by the user.
// Comment out other unwanted lines.

	GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;     // Disable pull-up on GPIO20 (MDXA)
	GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;     // Enable pull-up on GPIO21 (MDRA)
	GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;     // Enable pull-up on GPIO22 (MCLKXA)
	GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;      // Enable pull-up on GPIO7 (MCLKRA) (Comment as needed)
	//GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;   // Enable pull-up on GPIO58 (MCLKRA) (Comment as needed)
	GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;     // Enable pull-up on GPIO23 (MFSXA)
	GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;      // Enable pull-up on GPIO5 (MFSRA) (Comment as needed)
	//GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;   // Enable pull-up on GPIO59 (MFSRA) (Comment as needed)

/* Set qualification for selected input pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.


    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3;   // Asynch input GPIO21 (MDRA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 3;   // Asynch input GPIO22 (MCLKXA)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 3;    // Asynch input GPIO7 (MCLKRA) (Comment as needed)
    //GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3; // Asynch input GPIO58(MCLKRA) (Comment as needed)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;   // Asynch input GPIO23 (MFSXA)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 3;    // Asynch input GPIO5 (MFSRA) (Comment as needed)
    //GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3; // Asynch input GPIO59 (MFSRA) (Comment as needed)

	EDIS;
}

#if DSP28_MCBSPB
void InitMcbspbGpio(void)
{
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
		GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 3;	// GPIO25 is MDRB pin (Comment as needed)
		GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 3;	// GPIO26 is MCLKXB pin (Comment as needed)
		GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;	// Chip select for GPIO


		GpioCtrlRegs.GPAPUD.bit.GPIO24 = 1;	    // Disable pull-up on GPIO24 (MDXB) (Comment as needed)
		GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;	    // Enable pull-up on GPIO25 (MDRB) (Comment as needed)
		GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;	    // Enable pull-up on GPIO26 (MCLKXB) (Comment as needed)
		GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;	    // Enable pull-up


		GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 3;   // Asynch input GPIO25 (MDRB) (Comment as needed)
		GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 3;   // Asynch input GPIO26(MCLKXB) (Comment as needed)
		GpioCtrlRegs.GPBQSEL1.bit.GPIO40 = 3;   // Asynch input

		GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;  // GPIO19 is output
		GpioDataRegs.GPBSET.bit.GPIO40 = 1;
		GpioDataRegs.GPBDAT.bit.GPIO40 = 1;
		#endif
	EDIS;
}
#endif                // end DSP28_MCBSPB

void delay_loop(void)
{
    long      i;
    for (i = 0; i < MCBSP_INIT_DELAY; i++) {} //delay in McBsp init. must be at least 2 SRG cycles
}

void clkg_delay_loop(void)
{
    long      i;
    for (i = 0; i < MCBSP_CLKG_DELAY; i++) {} //delay in McBsp init. must be at least 2 SRG cycles
}
//===========================================================================
// No more.
//===========================================================================











