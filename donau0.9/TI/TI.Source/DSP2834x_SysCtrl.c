// TI File $Revision: /main/4 $
// Checkin $Date: June 29, 2009   16:44:29 $
//###########################################################################
//
// FILE:   DSP2834x_SysCtrl.c
//
// TITLE:  DSP2834x Device System Control Initialization & Support Functions.
//
// DESCRIPTION:
//
//         Example initialization of system resources.
//
//###########################################################################
// $TI Release: 2834x Header Files V1.12 $
// $Release Date: March 2011 $
//###########################################################################


#include "DSP2834x_Device.h"     // Headerfile Include File
#include "DSP2834x_Examples.h"   // Examples Include File

//---------------------------------------------------------------------------
// InitSysCtrl:
//---------------------------------------------------------------------------
// This function initializes the System Control registers to a known state.
// - Disables the watchdog
// - Set the PLLCR for proper SYSCLKOUT frequency
// - Set the pre-scaler for the high and low frequency peripheral clocks
// - Enable the clocks to the peripherals

void InitSysCtrl(void)
{

   // Disable the watchdog
   DisableDog();

   // Initialize the PLL control: PLLCR and DIVSEL
   // DSP28_PLLCR and DSP28_DIVSEL are defined in DSP2834x_Examples.h
   InitPll(DSP28_PLLCR,DSP28_DIVSEL);

   // Initialize the peripheral clocks
   InitPeripheralClocks();
}

//---------------------------------------------------------------------------
// Example: ServiceDog:
//---------------------------------------------------------------------------
// This function resets the watchdog timer.
// Enable this function for using ServiceDog in the application

void ServiceDog(void)
{
    EALLOW;
    SysCtrlRegs.WDKEY = 0x0055;
    SysCtrlRegs.WDKEY = 0x00AA;
    EDIS;
}

//---------------------------------------------------------------------------
// Example: DisableDog:
//---------------------------------------------------------------------------
// This function disables the watchdog timer.

void DisableDog(void)
{
    EALLOW;
    SysCtrlRegs.WDCR= 0x0068;
    EDIS;
}

void EnableDog(void)
{
	// Enable the watchdog
	EALLOW;
	SysCtrlRegs.WDCR = 0x0028;
	EDIS;
}

// Added function
void RebootSystem(void)
{
	EALLOW;
    SysCtrlRegs.WDCR = 0x8;
    EDIS;
}
//---------------------------------------------------------------------------
// Example: InitPll:
//---------------------------------------------------------------------------
// This function initializes the PLLCR register.

void InitPll(Uint16 val, Uint16 divsel)
{
   // DIVSEL MUST be 0 before PLLCR can be changed from
   // 0x0000. It is set to 0 by an external reset XRSn
   // This puts us in 1/8
   if (SysCtrlRegs.PLLSTS.bit.DIVSEL != 0)
   {
       EALLOW;
       SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
       EDIS;
   }

   // Change the PLLCR
   if (SysCtrlRegs.PLLCR.bit.DIV != val)
   {

      EALLOW;
      SysCtrlRegs.PLLCR.bit.DIV = val;
      EDIS;

      // Optional: Wait for PLL to lock.
      // During this time the CPU will switch to OSCCLK/2 until
      // the PLL is stable.  Once the PLL is stable the CPU will
      // switch to the new PLL value.
      //
      // This time-to-lock is monitored by a PLL lock counter.
      //
      // Code is not required to sit and wait for the PLL to lock.
      // However, if the code does anything that is timing critical,
      // and requires the correct clock be locked, then it is best to
      // wait until this switching has completed.

      // Wait for the PLL lock bit to be set.

      // The watchdog should be disabled before this loop, or fed within
      // the loop via ServiceDog().

	  // Uncomment to disable the watchdog
      DisableDog();

      while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1)
      {
	      // Uncomment to service the watchdog
          // ServiceDog();
      }

     }

    // If switching to 1/4
    if (divsel == 1)
    {
		EALLOW;
		SysCtrlRegs.PLLSTS.bit.DIVSEL = divsel;
		EDIS;
	}


    // If switching to 1/2
    // * First go to 1/4 and let the power settle
    //   The time required will depend on the system, this is only an example
	// * Then switch to 1/2
	if(divsel == 2)
	{
		EALLOW;
	    SysCtrlRegs.PLLSTS.bit.DIVSEL = 1;
	    DELAY_US(50L);
	    SysCtrlRegs.PLLSTS.bit.DIVSEL = 2;
	    EDIS;
	}

	// If switching to 1/1
	// NOTE: ONLY USE THIS SETTING IF PLL IS BYPASSED (I.E. PLLCR = 0)
	// * First go to 1/4 and let the power settle then go to 1/2 and let the power settle
	//   The time required will depend on the system, this is only an example
	// * Then switch to 1/1
	if(divsel == 3)
	{
		EALLOW;
	    SysCtrlRegs.PLLSTS.bit.DIVSEL = 1;
	    DELAY_US(50L);
	    SysCtrlRegs.PLLSTS.bit.DIVSEL = 2;
	    DELAY_US(50L);
	    SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;
	    EDIS;
    }
}

//--------------------------------------------------------------------------
// Example: InitPeripheralClocks:
//---------------------------------------------------------------------------
// This function initializes the clocks to the peripheral modules.
// First the high and low clock prescalers are set
// Second the clocks are enabled to each peripheral.
// To reduce power, leave clocks to unused peripherals disabled
//
// Note: If a peripherals clock is not enabled then you cannot
// read or write to the registers for that peripheral

void InitPeripheralClocks(void)
{
   EALLOW;

// HISPCP/LOSPCP prescale register settings, normally it will be set to default values
   SysCtrlRegs.HISPCP.all = 0x001F; // 4.6875Mhz
   SysCtrlRegs.LOSPCP.all = 0x0000; // 300Mhz

// XCLKOUT to SYSCLKOUT ratio.  By default XCLKOUT = 1/8 SYSCLKOUT
   // XTIMCLK = SYSCLKOUT/2
   XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
   // XCLKOUT = XTIMCLK/2
   XintfRegs.XINTCNF2.bit.CLKMODE = 1;
  // XCLKOUT = XTIMCLK/4
   XintfRegs.XINTCNF2.bit.BY4CLKMODE = 1;
   // Enable XCLKOUT
   XintfRegs.XINTCNF2.bit.CLKOFF = 0;

// Peripheral clock enables set for the selected peripherals.
// If you are not using a peripheral leave the clock off
// to save on power.
//
// Note: not all peripherals are available on all 2834x derivates.
// Refer to the datasheet for your particular device.
//
// This function is not written to be an example of efficient code.


   SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 1;   // I2C
   SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;   // SCI-A
   SysCtrlRegs.PCLKCR0.bit.SCIBENCLK = 1;   // SCI-B
   SysCtrlRegs.PCLKCR0.bit.SCICENCLK = 1;   // SCI-C
   SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;   // SPI-A
   SysCtrlRegs.PCLKCR0.bit.SPIDENCLK = 1;   // SPI-D
   SysCtrlRegs.PCLKCR0.bit.MCBSPAENCLK = 1; // McBSP-A
   SysCtrlRegs.PCLKCR0.bit.MCBSPBENCLK = 1; // McBSP-B
   SysCtrlRegs.PCLKCR0.bit.ECANAENCLK=1;    // eCAN-A
   SysCtrlRegs.PCLKCR0.bit.ECANBENCLK=1;    // eCAN-B

   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // Disable TBCLK within the ePWM
   SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;  // ePWM1
   SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;  // ePWM2
   SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;  // ePWM3
   SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 1;  // ePWM4
   SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1;  // ePWM5
   SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1;  // ePWM6
   SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 1;  // ePWM7
   SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK = 1;  // ePWM8
//XXX PWM 모드로 설정하지 않으면 EPWM Clock 이 나가도 GPIO 모드에 영향을 주지 않을 것으로 보이나 확인.
//#if !STATIC_SWITCH_GPIO
   //-150406 SysCtrlRegs.PCLKCR2.bit.EPWM9ENCLK = 1;  // ePWM9
//#endif
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Enable TBCLK within the ePWM
/*
   SysCtrlRegs.PCLKCR1.bit.ECAP3ENCLK = 1;  // eCAP3
   SysCtrlRegs.PCLKCR1.bit.ECAP4ENCLK = 1;  // eCAP4
   SysCtrlRegs.PCLKCR1.bit.ECAP5ENCLK = 1;  // eCAP5
   SysCtrlRegs.PCLKCR1.bit.ECAP6ENCLK = 1;  // eCAP6
   SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK = 1;  // eCAP1
   SysCtrlRegs.PCLKCR1.bit.ECAP2ENCLK = 1;  // eCAP2
   SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 1;  // eQEP1
   SysCtrlRegs.PCLKCR1.bit.EQEP2ENCLK = 1;  // eQEP2
   SysCtrlRegs.PCLKCR2.bit.EQEP3ENCLK = 1;  // eQEP3
*/
   SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK = 1; // CPU Timer 0
   SysCtrlRegs.PCLKCR3.bit.CPUTIMER1ENCLK = 1; // CPU Timer 1
   SysCtrlRegs.PCLKCR3.bit.CPUTIMER2ENCLK = 1; // CPU Timer 2

   SysCtrlRegs.PCLKCR3.bit.DMAENCLK = 1;       // DMA Clock
   SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;     // XTIMCLK
   SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;    // GPIO input clock

   EDIS;
}

//---------------------------------------------------------------------------
// Example: CsmUnlock:
//---------------------------------------------------------------------------
// This function unlocks the CSM. User must replace 0xFFFF's with current
// password for the DSP. Returns 1 if unlock is successful.

#define STATUS_FAIL          0
#define STATUS_SUCCESS       1

Uint16 CsmUnlock()
{
    volatile Uint16 temp;

    /* Below lines commented out because writing to key values
       will have no effect on these devices.
       Maintained here for legacy compatibility to older
       C28x designs */

    /*
	// Load the key registers with the current password. The 0xFFFF's are dummy
	// passwords.  User should replace them with the correct password for the DSP.

    EALLOW;
    CsmRegs.KEY0 = 0xFFFF;
    CsmRegs.KEY1 = 0xFFFF;
    CsmRegs.KEY2 = 0xFFFF;
    CsmRegs.KEY3 = 0xFFFF;
    CsmRegs.KEY4 = 0xFFFF;
    CsmRegs.KEY5 = 0xFFFF;
    CsmRegs.KEY6 = 0xFFFF;
    CsmRegs.KEY7 = 0xFFFF;
    EDIS;
    */
    // Perform a dummy read of the password locations
    // They should all return 0xFFFF. After the reads
    // have completed, the device memories will be
    // accessible.

    temp = CsmPwl.PSWD0;
    temp = CsmPwl.PSWD1;
    temp = CsmPwl.PSWD2;
    temp = CsmPwl.PSWD3;
    temp = CsmPwl.PSWD4;
    temp = CsmPwl.PSWD5;
    temp = CsmPwl.PSWD6;
    temp = CsmPwl.PSWD7;

    /* If password locations are properly read, return SUCCESS,
	   otherwise, return FAIL
	*/

    if (CsmRegs.CSMSCR.bit.SECURE == 0) return STATUS_SUCCESS;
    else return STATUS_FAIL;


}


//===========================================================================
// End of file.
//===========================================================================
