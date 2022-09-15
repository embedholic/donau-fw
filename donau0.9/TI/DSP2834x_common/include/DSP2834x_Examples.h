// TI File $Revision: /main/9 $
// Checkin $Date: July 9, 2009   09:41:30 $
//###########################################################################
//
// FILE:   DSP2834x_Examples.h
//
// TITLE:  DSP2834x Device Definitions.
//
//###########################################################################
// $TI Release: 2834x Header Files V1.12 $
// $Release Date: March 2011 $
//###########################################################################

#ifndef DSP2834x_EXAMPLES_H
#define DSP2834x_EXAMPLES_H


#ifdef __cplusplus
extern "C" {
#endif


/*-----------------------------------------------------------------------------
      Specify the PLL control register (PLLCR) and divide select (DIVSEL) value.
      PLLCR must be selected such that the input clock frequency (CLKIN) * PLLCR
      falls between 400 MHz and 600 MHz.

      For example, for a 20 MHz input clock, PLLCR may  be no less than
      19 (to achieve 20 MHz * 20 = 400 MHz) and no greater than 29 (to
      achieve 20 MHz * 30 = 600 MHz).

      NOTE: ONLY USE DIVSEL = 3 (/1 mode) SETTING IF PLL IS BYPASSED
      (I.E. PLLCR = 0)
-----------------------------------------------------------------------------*/
//#define DSP28_DIVSEL   0   // Enable /8 for SYSCLKOUT
//#define DSP28_DIVSEL   1 // Enable /4 for SYSCKOUT
//#define DSP28_DIVSEL     2 // Enable /2 for SYSCLKOUT
#define DSP28_DIVSEL     3 // Enable /1 for SYSCLKOUT

#define DSP28_PLLCR   29  // Fout=Fin x 30
//#define DSP28_PLLCR   28  // Fout=Fin x 29
//#define DSP28_PLLCR   27  // Fout=Fin x 28
//#define DSP28_PLLCR   26  // Fout=Fin x 27
//#define DSP28_PLLCR   25  // Fout=Fin x 26
//#define DSP28_PLLCR   24  // Fout=Fin x 25
//#define DSP28_PLLCR   23  // Fout=Fin x 24
//#define DSP28_PLLCR   22  // Fout=Fin x 23
//#define DSP28_PLLCR   21  // Fout=Fin x 22
//#define DSP28_PLLCR   20  // Fout=Fin x 21
//#define DSP28_PLLCR   19  // Fout=Fin x 20
//#define DSP28_PLLCR    9  // Fout=Fin x 10
//#define DSP28_PLLCR    8  // Fout=Fin x 9
//#define DSP28_PLLCR    7  // Fout=Fin x 8
//#define DSP28_PLLCR    6  // Fout=Fin x 7
//#define DSP28_PLLCR    5  // Fout=Fin x 6
//#define DSP28_PLLCR    4  // Fout=Fin x 5
//#define DSP28_PLLCR    3  // Fout=Fin x 4
//#define DSP28_PLLCR    2  // Fout=Fin x 3
//#define DSP28_PLLCR    1  // Fout=Fin x 2
//#define DSP28_PLLCR    0  // PLL is bypassed in this mode Fout = Fin
//----------------------------------------------------------------------------


/*-----------------------------------------------------------------------------
      Specify the clock rate of the CPU (SYSCLKOUT) in nS.

      Take into account the input clock frequency and the PLL multiplier
      selected in step 1:

      SYSCLKOUT = CLKIN * (PLLCR+1)/DIVSEL

      Use one of the values provided, or define your own.
      The trailing L is required tells the compiler to treat
      the number as a 64-bit value.

      Only one statement should be uncommented.

      Example 1:  300 MHz devices:
                  CLKIN is a 20MHz crystal.

                  In step 1 the user specified PLLCR = 29 and
                  DIVSEL=2 for a
                  300Mhz CPU clock (SYSCLKOUT = 300MHz).

                  In this case, the CPU_RATE will be 3.333L
                  Uncomment the line:  #define CPU_RATE  3.333L

      Example 2:  250 MHz devices:
                  CLKIN is a 20MHz crystal.

                  In step 1 the user specified PLLCR = 24 and
                  DIVSEL=2 for a
                  250Mhz CPU clock (SYSCLKOUT = 300MHz).

                  In this case, the CPU_RATE will be 4.000L
                  Uncomment the line:  #define CPU_RATE  4.000L


      Example 3:  200 MHz devices:
                  CLKIN is a 20MHz crystal.

	              In step 1 the user specified PLLCR = 19 and
	              DIVSEL=2 for a
	              200Mhz CPU clock (SYSCLKOUT = 200MHz).

	              In this case, the CPU_RATE will be 5.000L
                  Uncomment the line:  #define CPU_RATE  5.000L
-----------------------------------------------------------------------------*/
#define CPU_RATE    3.333L     // for 300MHz CPU clock speed (SYSCLKOUT)
//#define CPU_RATE    4.000L   // for 250MHz CPU clock speed (SYSCLKOUT)
//#define CPU_RATE    5.000L   // for 200 MHz CPU clock speed (SYSCLKOUT)
//#define CPU_RATE    6.667L   // for a 150MHz CPU clock speed (SYSCLKOUT)
//#define CPU_RATE   10.000L   // for a 100MHz CPU clock speed (SYSCLKOUT)
//#define CPU_RATE   13.330L   // for a 75MHz CPU clock speed (SYSCLKOUT)
//#define CPU_RATE   20.000L   // for a 50MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   33.333L   // for a 30MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   41.667L   // for a 24MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   50.000L   // for a 20MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   66.667L   // for a 15MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE  100.000L   // for a 10MHz CPU clock speed  (SYSCLKOUT)

//----------------------------------------------------------------------------

/*-----------------------------------------------------------------------------
      Target device (in DSP2834x_Device.h) determines CPU frequency
      (for examples) - either 300 MHz or 200 MHz. User does not have
      to change anything here.
-----------------------------------------------------------------------------*/
#if (DSP28_28346||DSP28_28344||DSP28_28342)  // DSP28_28346||DSP28_28344||DSP28_28342
  #define CPU_FRQ_200MHZ    0     // 300 Mhz CPU Freq (20 MHz input freq)
  #define CPU_FRQ_250MHZ    0
  #define CPU_FRQ_300MHZ    1
#elif (DSP28_28345||DSP28_28343||DSP28_28341)  // DSP28_28345||DSP28_28343||DSP28_28341
  #define CPU_FRQ_200MHZ    1
  #define CPU_FRQ_250MHZ    0     // 200 MHz CPU Freq (20 MHz input freq)
  #define CPU_FRQ_300MHZ    0
#else                             // DSP28_2834F device only
  #define CPU_FRQ_200MHZ    0
  #define CPU_FRQ_250MHZ    1
  #define CPU_FRQ_300MHZ    0     // 250 MHz CPU Freq (20 MHz input freq)
#endif


//---------------------------------------------------------------------------
// Include Example Header Files:
//

#include "DSP2834x_GlobalPrototypes.h"         // Prototypes for global functions within the
                                              // .c files.

#include "DSP2834x_ePwm_defines.h"             // Macros used for PWM examples.
#include "DSP2834x_Dma_defines.h"              // Macros used for DMA examples.
#include "DSP2834x_I2C_defines.h"              // Macros used for I2C examples.


#define PARTNO_28346  0xD0
#define PARTNO_28345  0xD1
#define PARTNO_28344  0xD2
#define PARTNO_28343  0xD3
#define PARTNO_28342  0xD4
#define PARTNO_28341  0xD5
#define PARTNO_2834F  0xD6


// Include files not used with DSP/BIOS
//#define DSP28_BIOS
#ifndef DSP28_BIOS
#include "DSP2834x_DefaultISR.h"
#endif


// DO NOT MODIFY THIS LINE.
#define DELAY_US(A)  DSP28x_usDelay(((((long double) A * 1000.0L) / (long double)CPU_RATE) - 9.0L) / 5.0L) // 1: 0.0000016sec
#define DELAY_NS(A)  DSP28x_usDelay(((((long double) A) / (long double)CPU_RATE) - 9.0L) / 5.0L)


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // end of DSP2834x_EXAMPLES_H definition


//===========================================================================
// End of file.
//===========================================================================
