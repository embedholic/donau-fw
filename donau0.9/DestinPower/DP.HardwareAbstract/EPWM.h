/*
 * EPWM.h
 *
 *  Created on: 2012. 11. 19.
 *      Author: destinPower
 */

#ifndef EPWM_H_
#define EPWM_H_

#include "LGS_Common.h"

#define	TBCLK		150E6	/* 150MHz */ // PWM 설정에서 sysclock / 2 를 하였음.
#define	PWMCARRIER	5E3		/* 5kHz */


public void EPWM_Init(void);

#if 0 //by JCNET
public void EPWM_CALC();
public void EPWM_Enable();
public void EPWM_Disable();
#else

public void EPWM_Converter_CALC();
public void EPWM_Converter_Enable();
public void EPWM_Converter_Disable();
public void EPWM_Inverter_CALC();
public void EPWM_Inverter_Enable();
public void EPWM_Inverter_Disable();
#endif
public void EPWM_SyncSignalON();
public void EPWM_SyncSignalOFF();

public void EPWM_SSW_ON();
public void EPWM_SSW_OFF();

#endif /* EPWM_H_ */
