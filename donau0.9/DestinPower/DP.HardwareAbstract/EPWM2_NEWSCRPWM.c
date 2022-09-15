#if 0
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

static	Bool bEnb;
UInt16 uOnTimeMax, uOnTimeMaxStepDown;
UInt16 uOnTimeMin;
UInt16 uPWMPeriod;

#if 0
// fixme delete
volatile int HoldCount = 0;
volatile int BeforeValue = 0;
volatile int BeforeValues[40] = {0};
#endif

/* 16.04.26 */
//#define	NEW_SCR_PWM /* 일본 특허용 테스트 버전인 것으로 생각됨. 황동옥 수석이 개발한 보드용이 아님. */
#define PWM_DELAY_COMP

#ifndef NEW_SCR_PWM
#define SCR_DUTY 0.25*2
#else
#define SCR_DUTY 0.3*2//0.25
#endif

#define SCR_PWM_FREQ	10e3//(10e3->20kHz)

//#define MAG		20
float MAG = 25.0;

unsigned int rFlag = 0, sFlag = 0, tFlag = 0;

typedef enum
{
	R,
	S,
	T
}Phase;

typedef struct _Flag
{
	unsigned int rFlag;
	unsigned int sFlag;
	unsigned int tFlag;
}Flag;

Flag bFlag;

//void NEW_SCR_PWM(float PhsVolt, Flag *This, Phase RST);
//void NEW_SCR_PWM(Flag *This, Phase RST);

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
#if 0
	if(GRID_BYP_THETA.ThetaPolarity_r == TRUE)
	{
		EPwm6Regs.CMPA.half.CMPA = (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
		EPwm6Regs.CMPB = EPwm6Regs.TBPRD;
	}
	else
	{
		EPwm6Regs.CMPA.half.CMPA = 0;
		EPwm6Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
	}

	if(GRID_BYP_THETA.ThetaPolarity_s == TRUE)
	{
		EPwm7Regs.CMPA.half.CMPA = (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
		EPwm7Regs.CMPB = EPwm7Regs.TBPRD;
	}
	else
	{
		EPwm7Regs.CMPA.half.CMPA = 0;
		EPwm7Regs.CMPB = (EPwm7Regs.TBPRD + 1) - (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
	}

	if(GRID_BYP_THETA.ThetaPolarity_t == TRUE)
	{
		EPwm8Regs.CMPA.half.CMPA = (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
		EPwm8Regs.CMPB = EPwm8Regs.TBPRD;
	}
	else
	{
		EPwm8Regs.CMPA.half.CMPA = 0;
		EPwm8Regs.CMPB = (EPwm8Regs.TBPRD + 1) - (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
	}
#endif
#if 0
	EPwm7Regs.CMPA.half.CMPA = 0;
	EPwm7Regs.CMPB = EPwm6Regs.TBPRD;
	//EPwm6Regs.CMPA.half.CMPA = 0;
	//EPwm6Regs.CMPB = EPwm6Regs.TBPRD;
#endif

#ifndef NEW_SCR_PWM
#if 1
	EPwm6Regs.CMPA.half.CMPA = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;
	EPwm7Regs.CMPA.half.CMPA = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;
	EPwm8Regs.CMPA.half.CMPA = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;

	EPwm6Regs.CMPB = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;
	EPwm7Regs.CMPB = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;
	EPwm8Regs.CMPB = ((TBCLK/SCR_PWM_FREQ) * 0.5 - 1) * SCR_DUTY;
#endif	//#if 0
#if 0
	if(ACP.BYP.va > 0.)
	{
		EPwm6Regs.CMPA.half.CMPA = 750;
		EPwm6Regs.CMPB = 0;
	}
	else if(ACP.BYP.va < 0.)
	{
		EPwm6Regs.CMPA.half.CMPA = 0;
		EPwm6Regs.CMPB = 750;
	}

	if(ACP.BYP.vb > 0.)
	{
		EPwm7Regs.CMPA.half.CMPA = 750;
		EPwm7Regs.CMPB = 0;
	}
	else if(ACP.BYP.vb < 0.)
	{
		EPwm7Regs.CMPA.half.CMPA = 0;
		EPwm7Regs.CMPB = 750;
	}

	if(ACP.BYP.vc > 0.)
	{
		EPwm8Regs.CMPA.half.CMPA = 750;
		EPwm8Regs.CMPB = 0;
	}
	else if(ACP.BYP.vc < 0.)
	{
		EPwm8Regs.CMPA.half.CMPA = 0;
		EPwm8Regs.CMPB = 750;
	}
#endif	//#if 0

#else // #ifndef NEW_SCR_PWM
#if 0
	/* Set Compare values */
	if(ACP.BYP.va > 0.)
	{
		EPwm6Regs.CMPA.half.CMPA = (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
		EPwm6Regs.CMPB = EPwm6Regs.TBPRD;
	}
	else
	{
		EPwm6Regs.CMPA.half.CMPA = 0;
		EPwm6Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
	}

	if(ACP.BYP.vb > 0.)
	{
		EPwm7Regs.CMPA.half.CMPA = (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
		EPwm7Regs.CMPB = EPwm7Regs.TBPRD;
	}
	else
	{
		EPwm7Regs.CMPA.half.CMPA = 0;
		EPwm7Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
	}

	if(ACP.BYP.vc > 0.)
	{
		EPwm8Regs.CMPA.half.CMPA = (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
		EPwm8Regs.CMPB = EPwm8Regs.TBPRD;
	}
	else
	{
		EPwm8Regs.CMPA.half.CMPA = 0;
		EPwm8Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
	}
#endif //#if 0
#ifdef PWM_DELAY_COMP
#if 0
	// R상
	if (ACP.BYP.va >= MAG)
	{
		EPwm6Regs.CMPA.half.CMPA = (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
		EPwm6Regs.CMPB = EPwm6Regs.TBPRD;
		rFlag = TRUE;
	}
	else if (ACP.BYP.va > -MAG && ACP.BYP.va < MAG && rFlag == TRUE)
	{
		EPwm6Regs.CMPA.half.CMPA = 0;
		EPwm6Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
		rFlag = TRUE;
	}
	else if (ACP.BYP.va <= -MAG)
	{
		EPwm6Regs.CMPA.half.CMPA = 0;
		EPwm6Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
		rFlag = FALSE;
	}
	else if (ACP.BYP.va > -MAG && ACP.BYP.va < MAG && rFlag == FALSE)
	{
		EPwm6Regs.CMPA.half.CMPA = (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
		EPwm6Regs.CMPB = EPwm6Regs.TBPRD;
		rFlag = FALSE;
	}

	// S상
	if (ACP.BYP.vb >= MAG)
	{
		EPwm7Regs.CMPA.half.CMPA = (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
		EPwm7Regs.CMPB = EPwm7Regs.TBPRD;
		sFlag = TRUE;
	}
	else if (ACP.BYP.vb > -MAG && ACP.BYP.vb < MAG && sFlag == TRUE)
	{
		EPwm7Regs.CMPA.half.CMPA = 0;
		EPwm7Regs.CMPB = (EPwm7Regs.TBPRD + 1) - (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
		sFlag = TRUE;
	}
	else if (ACP.BYP.vb <= -MAG)
	{
		EPwm7Regs.CMPA.half.CMPA = 0;
		EPwm7Regs.CMPB = (EPwm7Regs.TBPRD + 1) - (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
		sFlag = FALSE;
	}
	else if (ACP.BYP.vb > -MAG && ACP.BYP.vb < MAG && sFlag == FALSE)
	{
		EPwm7Regs.CMPA.half.CMPA = (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
		EPwm7Regs.CMPB = EPwm7Regs.TBPRD;
		sFlag = FALSE;
	}

	// T상
	if (ACP.BYP.vc >= MAG)
	{
		EPwm8Regs.CMPA.half.CMPA = (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
		EPwm8Regs.CMPB = EPwm8Regs.TBPRD;
		tFlag = TRUE;
	}
	else if (ACP.BYP.vc > -MAG && ACP.BYP.vc < MAG && tFlag == TRUE)
	{
		EPwm8Regs.CMPA.half.CMPA = 0;
		EPwm8Regs.CMPB = (EPwm8Regs.TBPRD + 1) - (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
		tFlag = TRUE;
	}
	else if (ACP.BYP.vc <= -MAG)
	{
		EPwm8Regs.CMPA.half.CMPA = 0;
		EPwm8Regs.CMPB = (EPwm8Regs.TBPRD + 1) - (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
		tFlag = FALSE;
	}
	else if (ACP.BYP.vc > -MAG && ACP.BYP.vc < MAG && tFlag == FALSE)
	{
		EPwm8Regs.CMPA.half.CMPA = (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
		EPwm8Regs.CMPB = EPwm8Regs.TBPRD;
		tFlag = FALSE;
	}
#endif	//#if 0
#endif	// #ifdef PWM_DELAY_COMP
#endif  // #ifndef NEW_SCR_PWM
#endif  // #if	STATIC_SWITCH_GPIO_PWM6_9 == 1
}

public void EPWM_SSW_OFF()
{
#ifndef NEW_SCR_PWM
#if	STATIC_SWITCH_GPIO_PWM6_9 == 1
	EPwm6Regs.CMPA.half.CMPA = 0;
	EPwm7Regs.CMPA.half.CMPA = 0;
	EPwm8Regs.CMPA.half.CMPA = 0;

	EPwm6Regs.CMPB = 0;
	EPwm7Regs.CMPB = 0;
	EPwm8Regs.CMPB = 0;
#endif
#else
#if	STATIC_SWITCH_GPIO_PWM6_9 == 1
	EPwm6Regs.CMPA.half.CMPA = 0;
	EPwm6Regs.CMPB = EPwm6Regs.TBPRD;
	EPwm7Regs.CMPA.half.CMPA = 0;
	EPwm7Regs.CMPB = EPwm6Regs.TBPRD;
	EPwm8Regs.CMPA.half.CMPA = 0;
	EPwm8Regs.CMPB = EPwm6Regs.TBPRD;
#endif
#endif
}

/* 2016.05.02 */
/* PWM Delay 임의 보상  완벽하지 않음 */

#if 0

//NEW_SCR_PWM(&ACP.BYP.va, &Flag, R);
//NEW_SCR_PWM(&ACP.BYP.vb, &Flag, S);
//NEW_SCR_PWM(&ACP.BYP.vc, &Flag, T);
//NEW_SCR_PWM(ACP.BYP.va, &Flag, R);
//NEW_SCR_PWM(ACP.BYP.vb, &Flag, S);
//NEW_SCR_PWM(ACP.BYP.vc, &Flag, T);
//NEW_SCR_PWM(ACP.BYP.va, &Flag, 0);
//NEW_SCR_PWM(ACP.BYP.vb, &Flag, 1);
//NEW_SCR_PWM(ACP.BYP.vc, &Flag, 2);

void NEW_SCR_PWM(float PhsVolt, Flag *This, Phase RST)
{
	unsigned int bFlag = 0;

	if (RST == R)
		bFlag = This->rFlag;
	else if (RST == S)
		bFlag = This->sFlag;
	else if (RST == T)
		bFlag = This->tFlag;

	if (PhsVolt >= MAG)
	{
		if (RST == R)
		{
			EPwm6Regs.CMPA.half.CMPA = (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
			EPwm6Regs.CMPB = EPwm6Regs.TBPRD;
			This->rFlag = TRUE;
		}
		else if (RST == S)
		{
			EPwm7Regs.CMPA.half.CMPA = (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
			EPwm7Regs.CMPB = EPwm7Regs.TBPRD;
			This->sFlag = TRUE;
		}
		else if (RST == T)
		{
			EPwm8Regs.CMPA.half.CMPA = (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
			EPwm8Regs.CMPB = EPwm8Regs.TBPRD;
			This->tFlag = TRUE;
		}
	}
	else if (PhsVolt > -MAG && PhsVolt < MAG && bFlag == TRUE)
	{
		if (RST == R)
		{
			EPwm6Regs.CMPA.half.CMPA = 0;
			EPwm6Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
			This->rFlag = TRUE;
		}
		else if (RST == S)
		{
			EPwm7Regs.CMPA.half.CMPA = 0;
			EPwm7Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
			This->sFlag = TRUE;
		}
		else if (RST == T)
		{
			EPwm8Regs.CMPA.half.CMPA = 0;
			EPwm8Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
			This->tFlag = TRUE;
		}
	}
	else if (PhsVolt <= -MAG)
	{
		if (RST == R)
		{
			EPwm6Regs.CMPA.half.CMPA = 0;
			EPwm6Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
			This->rFlag = FALSE;
		}
		else if (RST == S)
		{
			EPwm7Regs.CMPA.half.CMPA = 0;
			EPwm7Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
			This->sFlag = FALSE;
		}
		else if (RST == T)
		{
			EPwm8Regs.CMPA.half.CMPA = 0;
			EPwm8Regs.CMPB = (EPwm6Regs.TBPRD + 1) - (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
			This->tFlag = FALSE;
		}
	}
	else if (PhsVolt > -MAG && PhsVolt < MAG && bFlag == FALSE)
	{
		if (RST == R)
		{
			EPwm6Regs.CMPA.half.CMPA = (EPwm6Regs.TBPRD + 1) * SCR_DUTY;
			EPwm6Regs.CMPB = EPwm6Regs.TBPRD;
			This->rFlag = FALSE;
		}
		else if (RST == S)
		{
			EPwm7Regs.CMPA.half.CMPA = (EPwm7Regs.TBPRD + 1) * SCR_DUTY;
			EPwm7Regs.CMPB = EPwm7Regs.TBPRD;
			This->sFlag = FALSE;
		}
		else if (RST == T)
		{
			EPwm8Regs.CMPA.half.CMPA = (EPwm8Regs.TBPRD + 1) * SCR_DUTY;
			EPwm8Regs.CMPB = EPwm8Regs.TBPRD;
			This->tFlag = FALSE;
		}
	}
}
#endif

public void EPWM_Init(void)
{
#if 0
	memset(BeforeValues, 0, sizeof(BeforeValues)); // fixme delete
#endif

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
#ifndef NEW_SCR_PWM
#if STATIC_SWITCH_GPIO_PWM6_9 == 1
	EPwm6Regs.TBPRD = (TBCLK/SCR_PWM_FREQ) * 0.5 - 1;							/* Set Timer Period - 10kHz */
	EPwm6Regs.TBCTR = 0;							/* Clear Counter */
	EPwm7Regs.TBPRD = (TBCLK/SCR_PWM_FREQ) * 0.5 - 1;							/* Set Timer Period */
	EPwm7Regs.TBCTR = 0;							/* Clear Counter */
	EPwm8Regs.TBPRD = (TBCLK/SCR_PWM_FREQ) * 0.5 - 1;							/* Set Timer Period */
	EPwm8Regs.TBCTR = 0;
#endif
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

// 16.04.26
#ifndef NEW_SCR_PWM
#if STATIC_SWITCH_GPIO_PWM6_9 == 1
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

	EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;				/* Set EPWMxA on event A, up count */
	EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;			/* Clear EPWMxA on event A, down count */
	EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;

#if 0
	EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; 	// enable Dead-band module
	EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_LO;		// MCU 출력 단과 PMI 출력단과는 신호가 반전이다.
	EPwm6Regs.DBFED = 0; 							// FED = 450 TBCLKs.3us, 300 = 2us, 600 = 4us
	EPwm6Regs.DBRED = 0;
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

	EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;				/* Set EPWMxA on event A, up count */
	EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;			/* Clear EPWMxA on event A, down count */
	EPwm7Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm7Regs.AQCTLB.bit.CBU = AQ_CLEAR;

#if 0
	EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm7Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm7Regs.AQCTLB.bit.CBD = AQ_CLEAR;
	EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; 	// enable Dead-band module
	EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_LO;
	EPwm7Regs.DBFED = 0; 							// FED = 450 TBCLKs.3us, 300 = 2us, 600 = 4us
	EPwm7Regs.DBRED = 0;
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

#if 0
	EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm8Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm8Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; 	// enable Dead-band module
	EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_LO;
	EPwm8Regs.DBFED = 0; 							// FED = 450 TBCLKs.3us, 300 = 2us, 600 = 4us
	EPwm8Regs.DBRED = 0;
#endif
#endif
#else
#if STATIC_SWITCH_GPIO_PWM6_9 == 1
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

	/* Set actions */
	EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;				/* Set EPWMxA on event A, up count */
	EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;			/* Clear EPWMxA on event A, down count */
	EPwm6Regs.AQCTLB.bit.PRD = AQ_SET;
	EPwm6Regs.AQCTLB.bit.CBD = AQ_CLEAR;

	EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;				/* Set EPWMxA on event A, up count */
	EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;			/* Clear EPWMxA on event A, down count */
	EPwm7Regs.AQCTLB.bit.PRD = AQ_SET;				/* Set EPWMxB on event B, up count */
	EPwm7Regs.AQCTLB.bit.CBD = AQ_CLEAR;			/* Clear EPWMxB on event B, down count */

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



	/* set Trip for disalbe pwm signal 13.3.5 */
	// TRIP 신호는 eGBI로 입력 됨.
//	if(PRM_PCS[DGT_ARGUS_BOX1].iValue == 1)
//	{
//		EPwm1Regs.TZSEL.bit.OSHT1 = 1; // TZ1 onshot trip Select
//		EPwm2Regs.TZSEL.bit.OSHT1 = 1; // TZ1 Select
//		EPwm3Regs.TZSEL.bit.OSHT1 = 1; // TZ1 Select
//
//		EPwm1Regs.TZCTL.bit.TZA = 0x2; // EPWMxA to a low state
//		EPwm1Regs.TZCTL.bit.TZB = 0x2; // EPWMxB to a low state
//		EPwm2Regs.TZCTL.bit.TZA = 0x2;
//		EPwm2Regs.TZCTL.bit.TZB = 0x2;
//		EPwm3Regs.TZCTL.bit.TZA = 0x2;
//		EPwm3Regs.TZCTL.bit.TZB = 0x2;
//	}

#if 0 // 시작하자마자 Active High 되는 것을 방지 하기 위해.
	/* Init */
	EPwm1Regs.CMPA.half.CMPA = 0;
	EPwm2Regs.CMPA.half.CMPA = 0;
	EPwm3Regs.CMPA.half.CMPA = 0;
#endif
	EDIS;
}

Bool bPwmSetOnTime = 0;
public void EPWM_Disable()
{
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


volatile uint16_t A, B, C;
public void EPWM_CALC()
{
#if DEBUG_PWM_SIGNAL
	bEnb = 1;
#else
	bEnb = Con.bEnb;
#endif

	if (!bEnb)
	{
		EPWM_Disable();
	}

#if 0 //!Moved to EPWM INT
	A = Con.uPeriod;
	if ( A < PWM_CC_PERIOD_MIN ) A = PWM_CC_PERIOD_MIN;
	if ( A > PWM_CC_PERIOD_MAX ) A = PWM_CC_PERIOD_MAX;

	if ( uPWMPeriod != A )
	{
		uPWMPeriod = A;
		uOnTimeMax = A;
	}

	EPwm1Regs.TBPRD = uPWMPeriod;			/* Set Timer Period */
	EPwm2Regs.TBPRD = uPWMPeriod;			/* Set Timer Period */
	EPwm3Regs.TBPRD = uPWMPeriod;			/* Set Timer Period */
#endif

#if DEBUG_PWM_SIGNAL
	A = 7500 * 0.5;
	B = A;
	C = A;
#else
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

#if 0 // before
	EvaRegs.CMPR1 = A;
	EvaRegs.CMPR2 = B;
	EvaRegs.CMPR3 = C;
#endif

	EPwm1Regs.CMPA.half.CMPA = A;
	EPwm2Regs.CMPA.half.CMPA = B;
	EPwm3Regs.CMPA.half.CMPA = C;

#if 0
	if( CVC_GetStatus() == CVC_ENABLE ){
		if( A > uOnTimeMin && A < uOnTimeMax)
		{
			if( A  == BeforeValue )
			{
					BeforeValues[HoldCount++] = A;
			}
			else
			{
				HoldCount = 0;
			}

			if( HoldCount >= 10)
			{
				PRM_PCS[PV_MPP_RANGE_UPPER].iValue = 333;
				TRC_StopTrace();
				HoldCount = 0;
			}

			BeforeValue	= 	A;
		}
		else
		{
			HoldCount = 0;
		}


	}
#endif

	if(bEnb)
		bPwmSetOnTime = TRUE;


#if IGBT_LEVEL == 3
	EPwm4Regs.CMPA.half.CMPA = B;
	EPwm5Regs.CMPA.half.CMPA = C;
	EPwm6Regs.CMPA.half.CMPA = C;
#endif

}
unsigned int iCallCount =0;
interrupt void EPWM1_INT_ISR(void)
{
	EPWM_SyncSignalON();

	//PWM 인터럽트가 발생 후 CMPA값이 적용되어 PWM이 나갈 수 있도록 인터럽트가 발생하자 마자 GPIO를 Enable시킴. 연산 후 Enable 시키면 Default상태 값인 Active High가 초기에 들어가게 됨
	if( bPwmSetOnTime )
	{
		EPWM_Enable();
	}

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
#endif
