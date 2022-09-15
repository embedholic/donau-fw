/*
 * EVE_DC_CHARGER.c
 *
 *  Created on: 2018. 1. 22.
 *      Author: Seokhyun
 */

#include "EVE_DC_CHARGER.h"
#include "CC.h"
#include "SYSTEM.h"
#include "Fault.h"
#include "Event.h"

Eve EVE;

void EVE_DC_Charger_Create(void)
{
	EVE.bCMD_On = OFF;
	EVE.bCMD_FaultReset = FALSE;
	EVE.bCMDCharge = OFF;
	EVE.bStatus = OFF;
	EVE.bFaultStatus = FALSE; // EVE가 Shutdown 시 SET
	EVE.bInitCharge = FALSE; // EVE 초충 상태

	EVE.bMC8_OnOff = OFF;
	EVE.bMC8_Status = OFF;
	EVE.bMC9_OnOff = OFF;
	EVE.bMC9_Status = OFF;

	EVE.bCAN_Fail = FALSE;
	EVE.bCAN_Fail_TxChekbit = 0;
	EVE.bCAN_Fail_RxChekbit = 0;

	EVE.MAX_Charge_Volt = PARAM_VAL(EVE_MAXIMUM_CHARGE_VOLTAGE);
	EVE.Start_Charge_Volt = PARAM_VAL(EVE_START_CHARGE_VOLTAGE);

	EVE.Charge_Power = 0;
	EVE.nCAN_Fail_Timer = 0;
}

#pragma CODE_SECTION (EVE_Charging_Operation, "dp_ctrl")
void EVE_Charging_Operation(void)
{
	if(!FLT_GetEachStatus(FLTL_EVE_WARNING))
	{
		if(BATCTRL.DCBattV < PARAM_VAL(EVE_START_CHARGE_VOLTAGE))
			EVE.bCMDCharge = ON;
		else if(BATCTRL.DCBattV >= PARAM_VAL(EVE_MAXIMUM_CHARGE_VOLTAGE))
			EVE.bCMDCharge = OFF;
	}
	else
	{
		EVE.bCMDCharge = OFF;
	}
}

unsigned int iEveStartDelay = 0;

#pragma CODE_SECTION (EVE_Proceed, "dp_ctrl")
void EVE_Proceed(void)
{
#if 0 //by JCNET
	if(PARAM_VAL(EVE_DC_CHARGER_ENABLE) == FALSE)
		return;

	EVE_Check_EVE_Stautus();
	EVE_MCB_Chekcing();

	switch(INVERTER.uStatus)
	{
		case SYS_INV_FAULT:
		{
			EVE.bCMD_On = OFF;	// EVE OFF
			EVE.bCMDCharge = OFF;	//	EVE Charge Off
		}
			break;
		case SYS_INV_STOP:
		{
			EVE.bCMD_On = OFF;	// EVE OFF
			EVE.bCMDCharge = OFF;	//	EVE Charge Off
		}
			break;
		case SYS_INV_SCR_ON:
		{
			EVE.bCMD_On = OFF;	// EVE OFF
			EVE.bCMDCharge = OFF;	//	EVE Charge Off
			EVE.bCMD_FaultReset = TRUE;
		}
			break;
		case SYS_INV_DC_CHARGE:
		{
			EVE.bCMD_On = OFF;	// EVE OFF
			EVE.bCMDCharge = OFF;	//	EVE Charge Off
			iEveStartDelay = 0;
		}
			break;
		case SYS_INV_EVE_DC_CHARGE:
		{
			if( iEveStartDelay++ < 500 )
			{
				//wait for EVE fault reset
				return;
			}

			iEveStartDelay = 501;
		    if(EVE.bFaultStatus == FALSE && EVE.bCAN_Fail == FALSE)
		    {/*
		    	if( !CTRL_BYP_NormalOK() )
		    	{
		    		if(MC_GetStatus(STATUS_MC8))
		    			MC_UpdateStatus(CTRL_MC8_ONOFF, OPEN);
		    		if(MC_GetStatus(STATUS_MC9))
		    			 MC_UpdateStatus(CTRL_MC9_ONOFF, OPEN);
		    		EVE.bCMD_On = OFF;
					EVE.bCMDCharge = OFF;
					ODT_Initialize(INVERTER.odtBypassVok);
					break;
		    	}
		    	else
		    	{
		    		if( ODT_Update(INVERTER.odtBypassVok, TRUE) != ODT_FINISH )
		    			return;*/
					/* EVE는 동작 X, 초기 충전도 되지 않은 상태 */
					if( (EVE.bStatus == OFF) && (EVE.bInitCharge == FALSE) )
					{
						/* 초기 충전 하기 위해 MC9 ON */
						if(!MC_GetStatus(STATUS_MC9))
							MC_UpdateStatus(CTRL_MC9_ONOFF, CLOSED);
						break;
					}
					/* 초기 충전 완료 되어 bInitCharge가 SET이 되면 */
					else if((EVE.bInitCharge == TRUE))
					{
						if(!MC_GetStatus(STATUS_MC8))
						{
							/* 초기 충전 완료 되었기 때문에 MC8 붙이고 EVE ON 명령 전송 */
							MC_UpdateStatus(CTRL_MC8_ONOFF, CLOSED);
							EVE.bCMD_On = ON;
							EVE.bCMDCharge = ON;
							break;
						}

						/* MC8 이 On 된 것 확인 하고 MC9을 Off 한다 */
						if(MC_GetStatus(STATUS_MC8))
						{
							MC_UpdateStatus(CTRL_MC9_ONOFF, OPEN);
							break;
						}
					}
					else
					{
						/* EVE 초기 충전 되지 않았기 때문에 OFF 명령 */
						//EVE.bCMD_On = OFF;	// EVE OFF
					}
				//}
		    }
			else
			{
				/*
	    		if(MC_GetStatus(STATUS_MC8))
	    			MC_UpdateStatus(CTRL_MC8_ONOFF, OPEN);
	    		if(MC_GetStatus(STATUS_MC9))
	    			 MC_UpdateStatus(CTRL_MC9_ONOFF, OPEN);

	    		EVE.bCMD_On = OFF;
				EVE.bCMDCharge = OFF;
				ODT_Initialize(INVERTER.odtBypassVok);
				*/

				if(EVE.bCAN_Fail)
					FLT_Raise(FLTH_CAN);
				if(EVE.bFaultStatus)
					FLT_Raise(FLTH_EVE_FAULT);
			}
		}
			break;
		case SYS_INV_AC_GENERATE:
		{
			if(MC_GetStatus(STATUS_MC8))
				 MC_UpdateStatus(CTRL_MC8_ONOFF, OPEN);
            //if(EVE.bFaultStatus == FALSE || EVE.bCAN_Fail == FALSE)
            {
                EVE.bCMD_On = OFF;	// EVE ON
                EVE.bCMDCharge = OFF;	//	EVE Charge ON
            }
		}
			break;
		case SYS_INV_START_SYNC:
		{
            if(EVE.bFaultStatus == FALSE || EVE.bCAN_Fail == FALSE)
            {
                EVE.bCMD_On = OFF;	// EVE ON
                EVE.bCMDCharge = OFF;	//	EVE Charge ON
            }
		}
			break;
		case SYS_INV_RUN:
		{
            if(EVE.bFaultStatus == FALSE || EVE.bCAN_Fail == FALSE)
            {
                EVE.bCMD_On = OFF;	// EVE OFF
                //EVE_Charging_Operation();
                EVE.bCMDCharge = OFF;	//	EVE Charge OFF
            }
		}
			break;
		case SYS_INV_ISLANDING:
		{
			iEveStartDelay = 0;
            if(EVE.bFaultStatus == FALSE || EVE.bCAN_Fail == FALSE)
            {
                EVE.bCMD_On = OFF;	// EVE OFF
                EVE.bCMDCharge = OFF;	//	EVE Charge OFF
            }
		}
			break;
		case SYS_INV_RE_SYNC:
		{
			iEveStartDelay = 0;
            if(EVE.bFaultStatus == FALSE || EVE.bCAN_Fail == FALSE)
            {
                EVE.bCMD_On = OFF;	// EVE OFF
                EVE.bCMDCharge = OFF;	//	EVE Charge OFF
            }
		}
			break;
		case SYS_INV_BYP_EVT_OPERATION:
			break;
		case SYS_INV_TEST_MODE:
			break;
	}

	if(EVE.bFaultStatus == TRUE)
	{
	    EVE.bCMD_On = FALSE;
	    EVE.bCMDCharge = FALSE;

	    MC_UpdateStatus(CTRL_MC8_ONOFF, OPEN);
	    MC_UpdateStatus(CTRL_MC9_ONOFF, OPEN);
	}
#endif
}

#pragma CODE_SECTION (EVE_Check_EVE_Stautus, "dp_ctrl")
void EVE_Check_EVE_Stautus(void)
{
	if( EVE.bCMD_On == ON )
	{
		if(EVE.bStatus == OFF)
		{
			if(ODT_Update(INVERTER.odtEveCheck, TRUE))
				EVT_Store_NoDup(FLTL_EVE_WARNING);
		}
		else
		{
			ODT_Initialize(INVERTER.odtEveCheck);
			FLT_Clear(FLTL_EVE_WARNING);
		}
	}
	else
	{
		if(EVE.bStatus == ON)
		{
			if(ODT_Update(INVERTER.odtEveCheck, TRUE))
				EVT_Store_NoDup(FLTL_EVE_WARNING);
		}
		else
		{
			ODT_Initialize(INVERTER.odtEveCheck);
			FLT_Clear(FLTL_EVE_WARNING);
		}
	}
}

#pragma CODE_SECTION (EVE_UpdateParameter, "dp_ctrl")
void EVE_UpdateParameter(void)
{
	EVE.MAX_Charge_Volt = PARAM_VAL(EVE_MAXIMUM_CHARGE_VOLTAGE);
	EVE.Start_Charge_Volt = PARAM_VAL(EVE_START_CHARGE_VOLTAGE);
}

#pragma CODE_SECTION (EVE_MCB_Chekcing, "dp_ctrl")
void EVE_MCB_Chekcing(void)
{
#if 0 //by JCNET
    if(MCCB.Command.BitVal.bMC8_ONOFF == CLOSED)
        EVE.bMC8_OnOff = CLOSED;
    else
        EVE.bMC8_OnOff = OPEN;

    if(MCCB.Status.BitVal.bMC8 == CLOSED)
        EVE.bMC8_Status = CLOSED;
    else
        EVE.bMC8_Status = OPEN;

    if(MCCB.Command.BitVal.bMC9_ONOFF == CLOSED)
        EVE.bMC9_OnOff = CLOSED;
    else
        EVE.bMC9_OnOff = OPEN;

    if(MCCB.Status.BitVal.bMC9 == CLOSED)
        EVE.bMC9_Status = CLOSED;
    else
        EVE.bMC9_Status = OPEN;
#endif //
}

#pragma CODE_SECTION (EVE_CAN_Communication_TxChecking, "dp_ctrl")
void EVE_CAN_Communication_TxChecking(void)
{
    /* EVE CAN 통신 확인 */
    if(EVE.bCAN_Fail_TxChekbit == 1)
        EVE.bCAN_Fail_TxChekbit = 0;
    else
        EVE.bCAN_Fail_TxChekbit = 1;
}

#pragma CODE_SECTION (EVE_CAN_Communication_RxChecking, "dp_ctrl")
void EVE_CAN_Communication_RxChecking(void)
{
    /* EVE CAN 통신 확인 */
    if(EVE.bCAN_Fail_RxChekbit == EVE.bCAN_Prev_RxChekbit)
    {
        EVE.nCAN_Fail_Timer++;
    }
    else
    {
        EVE.nCAN_Fail_Timer = 0;

        if(FLT_GetEachStatus(FLTL_WAR_REV1))
            FLT_Clear( FLTL_WAR_REV1 );
    }

    if(EVE.nCAN_Fail_Timer > 60)
    {
        EVE.nCAN_Fail_Timer = 60;
        FLT_Raise( FLTL_WAR_REV1 );
    }

    EVE.bCAN_Prev_RxChekbit = EVE.bCAN_Fail_RxChekbit;
}
