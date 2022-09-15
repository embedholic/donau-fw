/*
 * PWM.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */


#include "PWM.h"
#include "FAULT.h"
#include "trace.h"
#include "EPWM.h"


//Pwm PWM_ADDR;//+
PwmOnTime		Con;
PwmOnTime       Inv_Pwm_Info;
Uns CC_uPeriod = 285;


void PWM_Create( void )
{
	PWM_Disable(PWM_CON);
	PWM_Disable(PWM_SCC); //by JCNET
}

//-#pragma CODE_SECTION(PWM_SetOnTime, ".iram")
void PWM_SetOnTime( PwmID ID, PwmOnTime *pOnTime )
{
	switch( ID )
	{
		case PWM_CON:
			Con.uPhA = pOnTime->uPhA;
			Con.uPhB = pOnTime->uPhB;
			Con.uPhC = pOnTime->uPhC;
			Con.bEnb = TRUE;
			EPWM_Converter_CALC();
			break;
//by JCNET
		case PWM_SCC:
		    Inv_Pwm_Info.uPhA = pOnTime->uPhA;
		    Inv_Pwm_Info.uPhB = pOnTime->uPhB;
		    Inv_Pwm_Info.uPhC = pOnTime->uPhC;
		    Inv_Pwm_Info.bEnb = TRUE;
            EPWM_Inverter_CALC();
		    break;
	}
#if 0
	EPWM_CALC();
#endif
}

void PWM_Disable( PwmID ID )
{
#if PWM_TEST == 1
	if(ID == 3)
	{
		PWM_ADDR.Con.bEnb = FALSE;
		PWM_ADDR.Con.uPhA = 0;
		PWM_ADDR.Con.uPhB = 0;
		PWM_ADDR.Con.uPhC = 0;
	}
	return;
#endif
	switch( ID )
	{
		case PWM_CON:
			Con.bEnb = FALSE;
			Con.uPhA = 0;
			Con.uPhB = 0;
			Con.uPhC = 0;
//by JCNET
//			EPWM_Disable();
			EPWM_Converter_Disable();
			break;
        case PWM_SCC:
            Inv_Pwm_Info.bEnb = FALSE;
            Inv_Pwm_Info.uPhA = 0;
            Inv_Pwm_Info.uPhB = 0;
            Inv_Pwm_Info.uPhC = 0;
//by JCNET
//          EPWM_Disable();
            EPWM_Inverter_Disable();
            break;
	}
}

void PWM_SetPeriod( PwmID ID, PwmOnTime *pOnTime )
{
	switch( ID )
	{
		case PWM_CON:
			Con.uPeriod = pOnTime->uPeriod;
			break;
//by JCNET
        case PWM_SCC:
            Inv_Pwm_Info.uPeriod = pOnTime->uPeriod;
            break;
	}
}


