/*
 * EPWM.c
 *
 *  Created on: 2012. 11. 19.
 *      Author: destinPower
 */
#include "LGS_Common.h"
#include "EPWM.h"
#include "cc.h"
#include "pwm.h"
#include "XINTF_ADC.h"
#include "DSP28x_Project.h"
#if DEBUG_MODE == 1
#include <xdc/runtime/System.h>
#endif
#include "trace.h"

#include "CC.h" // fixme delete
#include "PRM_PCS.h" // fixme delete

// 300 = 2us, 450 = 3us, 600 = 4us, 750 = 5us
#define PWM_DEAD_TIME  (450)

//by JCNET static	Bool bEnb;
UInt16 uOnTimeMax, uOnTimeMaxStepDown;
UInt16 uOnTimeMin;
UInt16 uPWMPeriod;

#define SCR_DUTY 0.25*2
#define SCR_PWM_FREQ	10e3//(10e3->20kHz)

public void EPWM_SyncSignalON()
{
	GpioDataRegs.GPBDAT.bit.GPIO62 = 1;
}
public void EPWM_SyncSignalOFF()
{
	GpioDataRegs.GPBDAT.bit.GPIO62 = 0;
}

public void EPWM_SSW_ON()
{
#if	STATIC_SWITCH_GPIO_PWM6_9 == 1
	EPwm6Regs.CMPA.half.CMPA = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;
	EPwm7Regs.CMPA.half.CMPA = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;
	EPwm8Regs.CMPA.half.CMPA = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;

	EPwm6Regs.CMPB = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;
	EPwm7Regs.CMPB = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;
	EPwm8Regs.CMPB = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;
#elif STATIC_SWITCH_GPIO_PWM6_9 == 2
	//21.06.30 June Node: ON조건은 1B, 2B모두 ON이거나 OFF면 SCR ON 그 외는 SCR OFF
    EPwm6Regs.CMPA.half.CMPA = 0;
    EPwm7Regs.CMPA.half.CMPA = 0;
    EPwm8Regs.CMPA.half.CMPA = 0;

    EPwm6Regs.CMPB = 0;
    EPwm7Regs.CMPB = 0;
    EPwm8Regs.CMPB = 0;
#endif
}

public void EPWM_SSW_OFF()
{
#if	STATIC_SWITCH_GPIO_PWM6_9 == 1
	EPwm6Regs.CMPA.half.CMPA = 0;
	EPwm7Regs.CMPA.half.CMPA = 0;
	EPwm8Regs.CMPA.half.CMPA = 0;

	EPwm6Regs.CMPB = 0;
	EPwm7Regs.CMPB = 0;
	EPwm8Regs.CMPB = 0;
#elif STATIC_SWITCH_GPIO_PWM6_9 == 2
    EPwm6Regs.CMPA.half.CMPA = EPwm6Regs.TBPRD;
    EPwm6Regs.CMPB = EPwm6Regs.TBPRD;   //27
    EPwm7Regs.CMPA.half.CMPA = 0;
    EPwm7Regs.CMPB = 0;
#endif
}


public void EPWM_Init(void)
{
	uPWMPeriod = 0;

	EALLOW;
#if DOUBLE_CONTROL == 1
	uOnTimeMin = 750 / 2; // 13.12.28 Double 제어 할 경우 /2 를 해야 함.
#else
	uOnTimeMin = 750;
#endif
	EPwm1Regs.TBPRD = (TBCLK/PWMCARRIER)/2;			/* Set Timer Period */
													//150MHz Period = 15000 TBCLK counts, TBPRD = 7500
	EPwm1Regs.TBCTR = 0;							/* Clear Counter */
	EPwm2Regs.TBPRD = (TBCLK/PWMCARRIER)/2;			/* Set Timer Period */
	EPwm2Regs.TBCTR = 0;							/* Clear Counter */
	EPwm3Regs.TBPRD = (TBCLK/PWMCARRIER)/2;			/* Set Timer Period */
	EPwm3Regs.TBCTR = 0;
#if IGBT_LEVEL == 3
	EPwm4Regs.TBPRD = 7500;							/* Set Timer Period - 10kHz */
	EPwm4Regs.TBCTR = 0;							/* Clear Counter */
	EPwm5Regs.TBPRD = 7500;							/* Set Timer Period */
	EPwm5Regs.TBCTR = 0;							/* Clear Counter */
	EPwm6Regs.TBPRD = 7500;							/* Set Timer Period */
	EPwm6Regs.TBCTR = 0;
#endif

	#if STATIC_SWITCH_GPIO_PWM6_9 != 0
	EPwm6Regs.TBPRD = (TBCLK/SCR_PWM_FREQ) * 0.5 - 1;							/* Set Timer Period - 10kHz */
	EPwm6Regs.TBCTR = 0;							/* Clear Counter */
	EPwm7Regs.TBPRD = (TBCLK/SCR_PWM_FREQ) * 0.5 - 1;							/* Set Timer Period */
	EPwm7Regs.TBCTR = 0;							/* Clear Counter */
	EPwm8Regs.TBPRD = (TBCLK/SCR_PWM_FREQ) * 0.5 - 1;							/* Set Timer Period */
	EPwm8Regs.TBCTR = 0;
	#endif

	// EPWM Module 1 config
	EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD/2; 	// Compare A = 3750
	EPwm1Regs.TBPHS.half.TBPHS = 0; 				// Set Phase register to zero
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; 		// Master module
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;						/* Clock ratio to 150MHz */
	EPwm1Regs.TBCTL.bit.CLKDIV = 0;							/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
#if DOUBLE_CONTROL == 0
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero
#else
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; 	// load on CTR = Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD; 	// load on CTR = Zero
#endif
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; 	// enable Dead-band module
	EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       //DB_ACTV_LOC; 		// Active Hi complementary
	EPwm1Regs.DBFED = PWM_DEAD_TIME; 							// FED = 450 TBCLKs.3us, 300 = 2us, 600 = 4us
	EPwm1Regs.DBRED = PWM_DEAD_TIME; 							// RED = 450 TBCLKs.3us

//JCNET start 2022/05/23
    EPwm1Regs.CMPB = EPwm1Regs.TBPRD/2;   // Compare A = 3750
    EPwm2Regs.CMPB = EPwm2Regs.TBPRD/2;   // Compare B = 3750
    EPwm3Regs.CMPB = EPwm3Regs.TBPRD/2;   // Compare C = 3750
//JCNET end 2022/05/23

	// EPWM Module 2 config
	EPwm2Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD/2; 	// Compare A = 3750
	EPwm2Regs.TBPHS.half.TBPHS = 0; 				// Set Phase register to zero. 13.3.2 0으로 수정 함. (120도 위상차는 Interative 타입만)
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE; 			// Slave module
	EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN; 			//
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	//-EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1;						/* Clock ratio to 150MHz */
	//-EPwm2Regs.TBCTL.bit.CLKDIV = 0;							/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
#if DOUBLE_CONTROL == 0
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero
#else
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; 	// load on CTR = Zero
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD; 	// load on CTR = Zero
#endif
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; 	// enable Dead-band module
	EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 		// Active Hi complementary
	EPwm2Regs.DBFED = PWM_DEAD_TIME; 							// FED = 450 TBCLKs
	EPwm2Regs.DBRED = PWM_DEAD_TIME; 							// RED = 450 TBCLKs

	// EPWM Module 3 config
	EPwm3Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD/2; 	// Compare A = 3750
	EPwm3Regs.TBPHS.half.TBPHS = 0; 				// Set Phase register to zero. 13.3.2 0으로 수정 함. (120도 위상차는 Interative 타입만)
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE; 			// Slave module
	EPwm3Regs.TBCTL.bit.PHSDIR = TB_DOWN; 			// TODO CHECK 13.3.2 UP으로 해야 하는가?
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	//-EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1;						/* Clock ratio to 150MHz */
	//-EPwm3Regs.TBCTL.bit.CLKDIV = 0;							/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
#if DOUBLE_CONTROL == 0
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero
#else
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD; 	// load on CTR = Zero
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD; 	// load on CTR = Zero
#endif
	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; 	// enable Dead-band module
	EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 		// Active Low complementary
	EPwm3Regs.DBFED = PWM_DEAD_TIME; 							// FED = 450 TBCLKs
	EPwm3Regs.DBRED = PWM_DEAD_TIME;

#if STATIC_SWITCH_GPIO_PWM6_9 != 0
	// EPWM Module 6 config
	EPwm6Regs.CMPA.half.CMPA = (TBCLK/SCR_PWM_FREQ) * 0.5 - 1; 	// Compare A = 3750
	EPwm6Regs.TBPHS.half.TBPHS = 0; 				// Set Phase register to zero
	EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE; 		// Master module
	EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	EPwm6Regs.TBCTL.bit.HSPCLKDIV = 1;						/* Clock ratio to 150MHz */
	EPwm6Regs.TBCTL.bit.CLKDIV = 0;							/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */
	EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero

	EPwm6Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

#if STATIC_SWITCH_GPIO_PWM6_9 == 1
	EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;				/* Set EPWMxA on event A, up count */
	EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;			/* Clear EPWMxA on event A, down count */
	EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
#endif

	// EPWM Module 4 config
	EPwm7Regs.CMPA.half.CMPA = (TBCLK/SCR_PWM_FREQ) * 0.5 - 1; 	// Compare A = 3750
	EPwm7Regs.TBPHS.half.TBPHS = 0; 				// Set Phase register to zero
	EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE; 		// Master module
	EPwm7Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	EPwm7Regs.TBCTL.bit.HSPCLKDIV = 1;						/* Clock ratio to 150MHz */
	EPwm7Regs.TBCTL.bit.CLKDIV = 0;							/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */
	EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero

	EPwm7Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
#if STATIC_SWITCH_GPIO_PWM6_9 == 1
	EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;				/* Set EPWMxA on event A, up count */
	EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;			/* Clear EPWMxA on event A, down count */
	EPwm7Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm7Regs.AQCTLB.bit.CBU = AQ_CLEAR;
#endif

	// EPWM Module 5 config
	EPwm8Regs.CMPA.half.CMPA = (TBCLK/SCR_PWM_FREQ) * 0.5 - 1; 	// Compare A = 3750
	EPwm8Regs.TBPHS.half.TBPHS = 0; 				// Set Phase register to zero
	EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE; 		// Master module
	EPwm8Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	EPwm8Regs.TBCTL.bit.HSPCLKDIV = 1;						/* Clock ratio to 150MHz */
	EPwm8Regs.TBCTL.bit.CLKDIV = 0;							/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */
	EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero

	EPwm8Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

	EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;				/* Set EPWMxA on event A, up count */
	EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;			/* Clear EPWMxA on event A, down count */
	EPwm8Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR;
#endif

#if STATIC_SWITCH_GPIO_PWM6_9 == 2
	/* Setup TBCLK */
	EPwm6Regs.TBPRD = (TBCLK/SCR_PWM_FREQ) * 0.5 - 1;	/* Set Timer Period */
	EPwm6Regs.TBCTR = 0;								/* Clear Counter */

	EPwm7Regs.TBPRD = EPwm6Regs.TBPRD;					/* Set Timer Period */
	EPwm7Regs.TBCTR = 0;								/* Clear Counter */

	EPwm8Regs.TBPRD = EPwm6Regs.TBPRD;					/* Set Timer Period */
	EPwm8Regs.TBCTR = 0;								/* Clear Counter */

	/* Setup counter mode */
	EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		/* Count Up/Down (Symmetric) */
	EPwm6Regs.TBPHS.half.TBPHS = 0;						/* Phase is 0 */
	EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;				/* Disable phase loading */
	EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;				/* Period Register is loaded from its shadow when CNTR=Zero */
	EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;			/* Clock ratio to SYSCLKOUT */
	EPwm6Regs.TBCTL.bit.CLKDIV = 0;						/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */

	EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		/* Count Up/Down (Symmetric) */
	EPwm7Regs.TBPHS.half.TBPHS = 0;						/* Phase is 0 */
	EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;				/* Disable phase loading */
	EPwm7Regs.TBCTL.bit.PRDLD = TB_SHADOW;				/* Period Register is loaded from its shadow when CNTR=Zero */
	EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;			/* Clock ratio to SYSCLKOUT */
	EPwm7Regs.TBCTL.bit.CLKDIV = 0;						/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */

	EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;		/* Count Up/Down (Symmetric) */
	EPwm8Regs.TBPHS.half.TBPHS = 0;						/* Phase is 0 */
	EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;				/* Disable phase loading */
	EPwm8Regs.TBCTL.bit.PRDLD = TB_SHADOW;				/* Period Register is loaded from its shadow when CNTR=Zero */
	EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;			/* 	Clock ratio to SYSCLKOUT */
	EPwm8Regs.TBCTL.bit.CLKDIV = 0;			 			/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */

	/* Setup shadowing */
	EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;		/* Enable Shadowing */
	EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;				/* Load on CNTR=Zero */
	EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;		/* Enable Shadowing */
	EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;				/* Load on CNTR=Zero */

	EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;		/* Enable Shadowing */
	EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;				/* Load on CNTR=Zero */
	EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;		/* Enable Shadowing */
	EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;		/* Enable Shadowing */
	EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;				/* Load on CNTR=Zero */
	EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;		/* Enable Shadowing */
	EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;				/* Load on CNTR=Zero */

    EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;              /* Set EPWMxA on event A, up count */
    EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR;            /* Clear EPWMxA on event A, down count */
    EPwm6Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm6Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm7Regs.AQCTLA.bit.PRD = AQ_SET;              /* Set EPWMxA on event A, up count */
    EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR;            /* Clear EPWMxA on event A, down count */
    EPwm7Regs.AQCTLB.bit.PRD = AQ_SET;              /* Set EPWMxB on event B, up count */
    EPwm7Regs.AQCTLB.bit.CBD = AQ_CLEAR;            /* Clear EPWMxB on event B, down count */

	EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;				/* Set EPWMxA on event A, up count */
	EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;			/* Clear EPWMxA on event A, down count */
	EPwm8Regs.AQCTLB.bit.PRD = AQ_SET;				/* Set EPWMxB on event B, up count */
	EPwm8Regs.AQCTLB.bit.CBD = AQ_CLEAR;			/* Clear EPWMxB on event B, down count */

	/* Set Dead-time */
	EPwm6Regs.DBCTL.bit.IN_MODE = 0;					/* EPWMxA is the source for both falling-edge & rising-edge delay */
	EPwm6Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;			/* Dead-band is fully Disabled for both rising-edge delay on EPWMxA and falling-edge delay on EPWMxB */
	EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;			/* Active High Complementary (AHC). EPWMxB is inverted */
	EPwm6Regs.DBFED = 0;								/* xusec */
	EPwm6Regs.DBRED = 0;								/* xusec */

	EPwm7Regs.DBCTL.bit.IN_MODE = 0;					/* EPWMxA is the source for both falling-edge & rising-edge delay */
	EPwm7Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;			/* Dead-band is fully Disabled for both rising-edge delay on EPWMxA and falling-edge delay on EPWMxB */
	EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;			/* Active High Complementary (AHC). EPWMxB is inverted */
	EPwm7Regs.DBFED = 0;								/* xusec */
	EPwm7Regs.DBRED = 0;								/* xusec */

	EPwm8Regs.DBCTL.bit.IN_MODE = 0;					/* EPWMxA is the source for both falling-edge & rising-edge delay */
	EPwm8Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;			/* Dead-band is fully Disabled for both rising-edge delay on EPWMxA and falling-edge delay on EPWMxB */
	EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;			/* Active High Complementary (AHC). EPWMxB is inverted */
	EPwm8Regs.DBFED = 0;								/* xusec */
	EPwm8Regs.DBRED = 0;								/* xusec */
#endif

#if IGBT_LEVEL == 3
	// EPWM Module 4 config
	EPwm4Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD/2; 	// Compare A = 3750
	EPwm4Regs.TBPHS.half.TBPHS = 5000; 				// Set Phase register to zero
	EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE; 			// Slave module
	EPwm4Regs.TBCTL.bit.PHSDIR = TB_DOWN; 			// Count DOWN on sync (=120 deg)
	EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;						/* Clock ratio to 150MHz */
	EPwm1Regs.TBCTL.bit.CLKDIV = 0;							/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */
	EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; 	// enable Dead-band module
	EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 		// Active Hi complementary
	EPwm4Regs.DBFED = PWM_DEAD_TIME; 							// FED = 450 TBCLKs
	EPwm4Regs.DBRED = PWM_DEAD_TIME;

	// EPWM Module 5 config
	EPwm5Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD/2; 	// Compare A = 3750
	EPwm5Regs.TBPHS.half.TBPHS = 5000; 				// Set Phase register to zero
	EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE; 			// Slave module
	EPwm5Regs.TBCTL.bit.PHSDIR = TB_UP; 			// Count UP on sync (=240 deg)
	EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;						/* Clock ratio to 150MHz */
	EPwm1Regs.TBCTL.bit.CLKDIV = 0;							/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */
	EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm5Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; 	// enable Dead-band module
	EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 		// Active Hi complementary
	EPwm5Regs.DBFED = PWM_DEAD_TIME; 							// FED = 450 TBCLKs
	EPwm5Regs.DBRED = PWM_DEAD_TIME;

	// EPWM Module 6 config
	EPwm6Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD/2; 	// Compare A = 3750
	EPwm6Regs.TBPHS.half.TBPHS = 5000; 				// Set Phase register to zero
	EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE; 			// Slave module
	EPwm6Regs.TBCTL.bit.PHSDIR = TB_UP; 			// Count UP on sync (=240 deg)
	EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;						/* Clock ratio to 150MHz */
	EPwm1Regs.TBCTL.bit.CLKDIV = 0;							/* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */
	EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; 	// enable Dead-band module
	EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 		// Active Hi complementary
	EPwm6Regs.DBFED = PWM_DEAD_TIME; 							// FED = 450 TBCLKs
	EPwm6Regs.DBRED = PWM_DEAD_TIME;
#endif

#if DOUBLE_CONTROL == 0
	EPwm1Regs.ETSEL.bit.SOCAEN = 0x1;   			// Enable extsoc1a event
	EPwm1Regs.ETSEL.bit.SOCASEL = 0x1;  			// Enable event time-base counter equal to zero. (TBCTR = 0x0000)
	EPwm1Regs.ETPS.bit.SOCAPRD = 0x1;   			// Generate SoC on first event
#else
	EPwm1Regs.ETSEL.bit.SOCAEN = 1;   				// Enable extsoc1a event
	EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;  	// Enable event time-base counter equal to zero. (TBCTR = 0x0000)
	EPwm1Regs.ETPS.bit.SOCAPRD = 0x1;   			// Generate SoC on first event

	EPwm2Regs.ETSEL.bit.SOCAEN = 1;   				// Enable extsoc1a event
    EPwm2Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;  		// Enable event time-base counter equal to zero. (TBCTR = 0x0000)
	EPwm2Regs.ETPS.bit.SOCAPRD = 0x1;   			// Generate SoC on first event
#endif

	// Enable extsoc1A set polarity
	SysCtrlRegs.EXTSOCCFG.bit.EXTSOC1AEN = 0x1;     // Enable extsoc1 A
	SysCtrlRegs.EXTSOCCFG.bit.EXTSOC1APOLSEL= 0x1;  // Set inverted polarity (CONVST is active low)

	/* Set Interrupts */
#if DOUBLE_CONTROL == 0
	EPwm1Regs.ETSEL.bit.INTSEL = 1;					/* Select INT on CNTR=Zero */
	EPwm1Regs.ETPS.bit.INTPRD = 1;					/* Generate INT on 1st event */
	EPwm1Regs.ETSEL.bit.INTEN = 1;					/* Enable INT */
#else
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;		/* Select INT on CNTR=Zero */
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;					/* Generate INT on 1st event */
	EPwm1Regs.ETSEL.bit.INTEN = 1;					/* Enable INT */

	EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;		/* Select INT on CNTR=Zero */
	EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;					/* Generate INT on 1st event */
	EPwm2Regs.ETSEL.bit.INTEN = 1;					/* Enable INT */
#endif

	EDIS;
}
#if 0 //by JCNET
Bool bPwmSetOnTime = 0;
#else
Bool bConverterPwmSetOnTime = 0;
Bool bInverterPwmSetOnTime = 0;
#endif
//JCNET start 2022/05/23
#define EPWM_CONVERTER_ASIDE
#ifdef EPWM_CONVERTER_ASIDE
#define EPWM_CONVERTER_MASK ((1<<0) | (1 << 4) | (1 << 8))
#define EPWM_INVERTER_MASK  ((1<<2) | (1 << 6) | (1 << 10))
#else
#define EPWM_INVERTER_MASK ((1<<0) | (1 << 4) | (1 << 8))
#define EPWM_CONVERTER_MASK  ((1<<2) | (1 << 6) | (1 << 10))
#endif
public void EPWM_Converter_Enable()
{
	//Enable PWM1,2,3 A/B:
	EALLOW;
	GpioCtrlRegs.GPAMUX1.all = GpioCtrlRegs.GPAMUX1.all | EPWM_CONVERTER_MASK;
	EDIS;
}
public void EPWM_Converter_Disable()
{
    bConverterPwmSetOnTime = FALSE;

    //Disable PWM1,2,3 A/B converter tuple
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all &= ~EPWM_CONVERTER_MASK;
    EDIS;
#ifdef EPWM_CONVERTER_ASIDE
    EPwm1Regs.CMPA.half.CMPA = 0;
    EPwm2Regs.CMPA.half.CMPA = 0;
    EPwm3Regs.CMPA.half.CMPA = 0;
#else
    EPwm1Regs.CMPB = 0;
    EPwm2Regs.CMPB = 0;
    EPwm3Regs.CMPB = 0;
#endif
}

public void EPWM_Inverter_Enable()
{
    //Enable PWM1,2,3 A/B:
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all = GpioCtrlRegs.GPAMUX1.all | EPWM_INVERTER_MASK;
    EDIS;
}
public void EPWM_Inverter_Disable()
{
    bInverterPwmSetOnTime = FALSE;

    //Disable PWM1,2,3 A/B converter tuple
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all &= ~EPWM_INVERTER_MASK;
    EDIS;
#ifndef EPWM_CONVERTER_ASIDE
    EPwm1Regs.CMPA.half.CMPA = 0;
    EPwm2Regs.CMPA.half.CMPA = 0;
    EPwm3Regs.CMPA.half.CMPA = 0;
#else
    EPwm1Regs.CMPB = 0;
    EPwm2Regs.CMPB = 0;
    EPwm3Regs.CMPB = 0;
#endif
}
//JCNET end 2022/05/23

//JCNET
#if 0 // JCNET
public void EPWM_Disable()
{
    return; // by isjeon for test
    bPwmSetOnTime = FALSE;

    //Disable PWM1,2,3 A/B:
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all &= 0xFFFFF000;
    EDIS;

    EPwm1Regs.CMPA.half.CMPA = 0;
    EPwm2Regs.CMPA.half.CMPA = 0;
    EPwm3Regs.CMPA.half.CMPA = 0;


#if IGBT_LEVEL == 3
    EPwm4Regs.CMPA.half.CMPA = 0;
    EPwm5Regs.CMPA.half.CMPA = 0;
    EPwm6Regs.CMPA.half.CMPA = 0;
#endif
}

public void EPWM_Enable()
{
    //Enable PWM1,2,3 A/B:
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all |= 0x00000555;
    EDIS;
}
#endif

volatile uint16_t A, B, C;
#if 0 //by JCNET
public void EPWM_CALC()
#endif
public void EPWM_Converter_CALC()
{
    Bool bEnb;//by JCNET
#if DEBUG_PWM_SIGNAL
    bEnb = 1;
#else
    bEnb = Con.bEnb;
#endif
    if (!bEnb)
    {
        EPWM_Converter_Disable();
    }

#if DEBUG_PWM_SIGNAL
    A = 7500 * 0.5;
    B = A;
    C = A;
#else
//by JCNET

    A = Con.uPhA ; // ON Duty
    B = Con.uPhB ; // ON Duty
    C = Con.uPhC ; // ON Duty
#endif

    if ( A > uOnTimeMax ) A = uOnTimeMax;
    if ( B > uOnTimeMax ) B = uOnTimeMax;
    if ( C > uOnTimeMax ) C = uOnTimeMax;

    if ( A < uOnTimeMin ) A = 0;
    if ( B < uOnTimeMin ) B = 0;
    if ( C < uOnTimeMin ) C = 0;

    A = uPWMPeriod - A;
    B = uPWMPeriod - B;
    C = uPWMPeriod - C;
#ifdef EPWM_CONVERTER_ASIDE
    EPwm1Regs.CMPA.half.CMPA = A;
    EPwm2Regs.CMPA.half.CMPA = B;
    EPwm3Regs.CMPA.half.CMPA = C;
#else
    EPwm1Regs.CMPB = A;
    EPwm2Regs.CMPB = B;
    EPwm3Regs.CMPB = C;
#endif
    if(bEnb) bConverterPwmSetOnTime = TRUE;


#if IGBT_LEVEL == 3
    EPwm4Regs.CMPA.half.CMPA = B;
    EPwm5Regs.CMPA.half.CMPA = C;
    EPwm6Regs.CMPA.half.CMPA = C;
#endif

}
public void EPWM_Inverter_CALC()
{
    Bool bEnb;//by JCNET
#if DEBUG_PWM_SIGNAL
	bEnb = 1;
#else
    bEnb = Inv_Pwm_Info.bEnb;
#endif

	if (!bEnb)
	{
        EPWM_Inverter_Disable();
	}

#if DEBUG_PWM_SIGNAL
	A = 7500 * 0.5;
	B = A;
	C = A;
#else
    A = Inv_Pwm_Info.uPhA ; // ON Duty
    B = Inv_Pwm_Info.uPhB ; // ON Duty
    C = Inv_Pwm_Info.uPhC ; // ON Duty
#endif

	if ( A > uOnTimeMax ) A = uOnTimeMax;
	if ( B > uOnTimeMax ) B = uOnTimeMax;
	if ( C > uOnTimeMax ) C = uOnTimeMax;

	if ( A < uOnTimeMin ) A = 0;
	if ( B < uOnTimeMin ) B = 0;
	if ( C < uOnTimeMin ) C = 0;

	A = uPWMPeriod - A;
	B = uPWMPeriod - B;
	C = uPWMPeriod - C;

#ifndef EPWM_CONVERTER_ASIDE
    EPwm1Regs.CMPA.half.CMPA = A;
    EPwm2Regs.CMPA.half.CMPA = B;
    EPwm3Regs.CMPA.half.CMPA = C;
#else
    EPwm1Regs.CMPB = A;
    EPwm2Regs.CMPB = B;
    EPwm3Regs.CMPB = C;
#endif

    if(bEnb) bInverterPwmSetOnTime = TRUE;


#if IGBT_LEVEL == 3
	EPwm4Regs.CMPA.half.CMPA = B;
	EPwm5Regs.CMPA.half.CMPA = C;
	EPwm6Regs.CMPA.half.CMPA = C;
#endif

}
//JCNET end 2022/05/23

unsigned int iCallCount =0;
interrupt void EPWM1_INT_ISR(void)
{
	EPWM_SyncSignalON();

	//PWM 인터럽트가 발생 후 CMPA값이 적용되어 PWM이 나갈 수 있도록 인터럽트가 발생하자 마자 GPIO를 Enable시킴. 연산 후 Enable 시키면 Default상태 값인 Active High가 초기에 들어가게 됨
#if 0 //by JCNET
	if( bPwmSetOnTime )
	{
		EPWM_Enable();
	}
#else
    if( bConverterPwmSetOnTime )
    {
        EPWM_Converter_Enable();
    }
    if( bInverterPwmSetOnTime )
    {
        EPWM_Inverter_Enable();
    }
#endif
	// PWM 주기는 제어기가 동작하지 않아도 적용되도록 이곳으로 옮김.
	A = Con.uPeriod;
	if ( A < PWM_CC_PERIOD_MIN ) A = PWM_CC_PERIOD_MIN;
	if ( A > PWM_CC_PERIOD_MAX ) A = PWM_CC_PERIOD_MAX;

	if ( uPWMPeriod != A )
	{
		uPWMPeriod = A;
		uOnTimeMax = A;

		EPwm1Regs.TBPRD = uPWMPeriod;			/* Set Timer Period */
		EPwm2Regs.TBPRD = uPWMPeriod;			/* Set Timer Period */
		EPwm3Regs.TBPRD = uPWMPeriod;			/* Set Timer Period */
	}

	XINTF_ADC_READ_ALL();

	c_int11(0); // ADC-> CC

	EPWM_SyncSignalOFF();
	/* Clear INT flag for this timer */
	EPwm1Regs.ETCLR.bit.INT = 1;
	/* Acknowledge this interrupt to receive more interrupts from group 3 */
	PieCtrlRegs.PIEACK.bit.ACK3 = 1;
}

// INT3.2
interrupt void EPWM2_INT_ISR(void)     // EPWM-2
{
	XINTF_ADC_READ_ALL();

	c_int11(1); // ADC-> CC

	EPwm2Regs.ETCLR.bit.INT = 1;
	// To receive more interrupts from this PIE group, acknowledge this interrupt
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
//===========================================================================
// End of file.
//===========================================================================

