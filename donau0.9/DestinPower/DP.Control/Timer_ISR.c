/*
 * Timer_ISR.c
 *
 *  Created on: 2016. 5. 10.
 *      Author: Welcome!
 */
#include "LGS_Common.h"
#include "DSP28x_Project.h"
//
#include "Timer_ISR.h"
#include "CC.h"
#include "GPIO.h"
#include "EPWM.h"
//#include <xdc/std.h>

float InterruptCount = 0;
float timer_cnt = 0.000001;
//int timer_us = 1;
void Timer_ISR_init(void)
{
	//ConfigCpuTimer(&CpuTimer0, 300, 1000000);
	//CpuTimer0Regs.TIM.all = 586;
//	CpuTimer0Regs.TPR.all = 586;

	CpuTimer0Regs.PRD.all = 15;
	CpuTimer0Regs.TPR.bit.TDDR = 9;
	CpuTimer0Regs.TCR.all = 0x4000;

}

uint32_t my_tick = 0;
volatile uint32_t my_flag = 0;
interrupt void cpu_timer0_isr(void)
{
   if(++my_tick >= 1875000)
   {
       my_tick = 0;
       my_flag = 1;
   }
   //CpuTimer0Regs.PRD.all = 15*timer_us;
   InterruptCount += timer_cnt;

#ifndef	STABLEEN
   if(PRM_PCS[CTRL_TEST_MODE].iValue != 0)
   {
		if(PRM_PCS[SYS_REV_5].iValue == 1 )
		{
			EPWM_SSW_ON();
		}
		else if( PRM_PCS[SYS_REV_5].iValue == 2)
		{
			EPWM_SSW_OFF();
		}
   }
   else
   {
		//GpioDataRegs.GPBDAT.bit.GPIO63 = (Bool)OnOff;

		if( GpioDataRegs.GPBDAT.bit.GPIO63 == GPIO_STATIC_SW_ON )
			EPWM_SSW_ON();
		else
			EPWM_SSW_OFF();
   }
#endif	//	#ifndef	STABLEEN
//	CpuTimer0Regs.TCR.bit.TIE = 0;
	EALLOW;
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
   EDIS;
}
