/*
 * JCNET_SYSTEM.c
 *
 *  Created on: 2022. 6. 14.
 *      Author: isjeon
 */


#define DCLINK_SIMUL
#define _CB_SIMUL
//#define _PC_SIMUL_
#ifdef _PC_SIMUL_

#include <stdio.h>
#include <string.h>

typedef unsigned int Bool;
typedef unsigned int Uint32;
typedef unsigned int Uns;

typedef enum
{
    CB_RESET,
    CB_OFF,
    CB_ON,
    CB_TRIP
}MCB_STATUS;
typedef enum
{
    CB_CMD_RESET,
    CB_CMD_OFF,
    CB_CMD_ON,
    CB_CMD_TRIP
}M_CB_CMD;
typedef enum _SSW_Status { GPIO_STATIC_SW_OFF = 0, GPIO_STATIC_SW_ON = 1 } SSW_Status;

#define M_CB1_BATT 1
#define M_CB2_SSW  2

#define CMD_OFF 0
#define CMD_ON 1
#define OPEN  0
#define CLOSE 1

#define CTRL_MC1A_ONOFF 0
#define STATUS_MC1A 0
#define STATUS_MC1  1
#define TRUE 1
#define FALSE 0
#define ZERO 0
#define ON 1
#define OFF 0

#include "odt.c"
#else

#include <math.h>
#include <string.h>

#include "SYSTEM.h"
#include "ODT.h"
#include "FAULT.h"
#include "Event.h"
#include "MVP.h"
#include "CC.h"
#include "EADC.h"
#include "trace.h"
#include "PWM.h"
#include "MCCB.h"
#include "MathConst.h"
#include "RMS.h"
#include "LEVELCHECK.h"
#include "PHASECHECK.h"
#include "MCB.h"
#include "parameter.h"
#include "Prm_pcs.h"
//-#include "COSPHIcontrol.h"
#include "NvSRAM.h"
#include "CAN.h"
#include "CAN_GBI.h"
#include "MO.h"
#include "Task.h"
#include "GPIO.h"
#include "CTRL_INV_VI.h"
#include "CTRL_MODE.h"
#include "SYS_ESLV.h"
//#include "CTRL_GEN.h"
#include "CTRL_BYP_EVT_OPERATION.h"
#include "SagEventHistory.h"

#define SYS_VOLT_HYSTERESIS 2.0  /* 2.0V */
#define SYS_FREQ_HYSTERESIS 0.05  /* 0.05Hz */
#define SYS_PHASE_ERR_LEVEL 100

#define SYS_20MSEC_PERIOD 4 /* 5msec * 4 = 20msec */
#define SYS_10MSEC_PERIOD 2 /* 5msec * 2 = 10msec */
#define SYS_UF_OF_CHECK_NUM 100 /* 주파수가 이 횟수 만큼 동안 연속해서 문제가 있다면 고장 발생 */

#define SYS_OC_LEVEL 1.8
#define SYS_INPUT_I_LIMIT_LEVEL 1.25
#define SYS_OV_LEVEL 1.1
#define SYS_UV_LEVEL 0.9
#define SYS_BYP_INV_EQUAL_LEVEL 2 /* V */
#define SYS_CLEAR_CNT 50
#define MPP_VDC_REF_STEP    5.0



extern  AC_Panel    ACP;
extern  Controller  CTRL;

#pragma CODE_SECTION (SYS_UpdateChargerStatus, "dp_ctrl")
#pragma CODE_SECTION (SYS_UpdateRectifierStatus, "dp_ctrl")
#pragma CODE_SECTION (SYS_UpdateSagCompensatorStatus, "dp_ctrl")
#endif
typedef enum sys_charger_state_tag {
    CHG_STATE_DISCHARGED,
    CHG_STATE_MC1AON_ODT,
    CHG_STATE_MC1AON,
    CHG_STATE_MC1REQ_ODT,
    CHG_STATE_MC1REQ,
    CHG_STATE_CHARGED ,
    CHG_STATE_FAULT,
} charger_state_t;

typedef enum sys_rectifier_state_tag {
    REC_STATE_STOP,
    REC_STATE_READY,
    REC_STATE_RUN_ODT,
    REC_STATE_RUN
} rectifier_state_t;

typedef enum sys_inverter_state_tag {
    SCC_STATE_STOP,
    SCC_STATE_READY_ODT,
    SCC_STATE_READY,
    SCC_STATE_START_ODT,
    SCC_STATE_START,
    SCC_STATE_RUN
} inverter_state_t;

typedef struct charger_state_machine_tag {
    Bool bActive; // active but can be not run
    Bool bRun;    // actually in run state;
    Bool bOdtRun; //
    charger_state_t   state;
    Odt odtMC1A_On;
    Odt odtMC1_On;
    Odt odtDCLinkRisingCheck;
    void (*init)(void);
} charger_state_machine_t;

typedef struct rectifier_state_machine_tag {
    Bool bActive; // active but can be not run
    Bool bRun;    // actually in run state;
    Bool bOdtRun;
    rectifier_state_t   state;
    Odt odtDoOp;
    void (*init)(void);
} rectifier_state_machine_t;

typedef struct inverter_state_machine_tag {
    Bool bActive; // active but can be not run
    Bool bRun;    // actually in run state;
    Bool bOdtRun;
    inverter_state_t   state;
    Odt odtDoOp;
    void (*init)(void);
} inverter_state_machine_t;


typedef struct _DONAU_machine_tag {
    Uns  uTestMode;// PARAM[TEST_MODE] copy
    Bool bActive; //
    Bool bStart; //
    Bool bHeavyFault;

    Bool RDC_ON;  // Rectifier DC Link voltage controller on or off
    Bool IAC_ON;

    Uns   uPeriod;
    float DCLinkV;
    float DCInitMax;
    charger_state_machine_t charger;
    rectifier_state_machine_t rectifier;
    inverter_state_machine_t inverter;
} DONAU_machine_t;

DONAU_machine_t DONAU_State;
extern Inverter INVERTER;
#ifdef _CB_SIMUL
#define MC_UpdateStatus   simu_MC_UpdateStatus
#define MC_GetStatus      simu_MC_MC_GetStatus
#define MCB_UpdateCmd     simu_MCB_UpdateCmd
#define GPIO_StaticSwitch simu_GPIO_StaticSwitch

Uns status_mc1a, status_mc1, status_mc2, status_ssw;
void MC_UpdateStatus(Uns sw, Uns onoff)
{
        if(sw == CTRL_MC1A_ONOFF) status_mc1a = onoff;
#if 0
        if(sw == CTRL_MC1_ONOFF)  status_mc1 = onoff;
        if(sw == CTRL_MC2_ONOFF)  status_mc2 = onoff;
#endif
}

Uns MC_GetStatus(Uns sw)
{
        if(sw == STATUS_MC1A) return status_mc1a;
        if(sw == STATUS_MC1) return status_mc1;
        if(sw == STATUS_MC2) return status_mc2;
        return 0;
}

MCB_UpdateCmd(Uns id, Uns sw)
{
    if(id == M_CB1_BATT)
    {
        if(sw == CB_CMD_ON) status_mc1 = 1;
        else status_mc1 = 0;
    }
    if(id == M_CB2_SSW)
    {
        if(sw == CB_CMD_ON) status_mc2 = 1;
        else status_mc2 = 0;
    }
}

void GPIO_StaticSwitch(SSW_Status OnOff)
{
}
#endif
extern void EPWM_Converter_Enable();
extern void EPWM_Converter_Disable();
extern void EPWM_Inverter_Enable();
extern void EPWM_Inverter_Disable();

#define IS_TEST_MODE()    (DONAU_State.uTestMode == 1)
#define IS_RUN(module)    (DONAU_State.module.bRun == TRUE)
#define IS_ALL_ACTIVE()   (DONAU_State.bActive == TRUE)
#define IS_IND_ACTIVE(module)  (DONAU_State.module.bActive == TRUE)

#define IND_DO_OP(module)      ( IS_TEST_MODE() &&  IS_IND_ACTIVE(module))
#define ALL_DO_OP()       (!IS_TEST_MODE() &&  IS_ALL_ACTIVE())
#define IND_STOP_OP(module)    ( IS_TEST_MODE() && !IS_IND_ACTIVE(module))
#define ALL_STOP_OP()     (!IS_TEST_MODE() && !IS_ALL_ACTIVE())


static void init_charger()
{
//    volatile Odt *odt_handle;
    memset(&DONAU_State.charger,0,sizeof(DONAU_State.charger) - sizeof(DONAU_State.charger.init));
    Odt_(&DONAU_State.charger.odtMC1A_On, 500, DONAU_State.uPeriod);
    Odt_(&DONAU_State.charger.odtMC1_On , 500, DONAU_State.uPeriod);
    Odt_(&DONAU_State.charger.odtDCLinkRisingCheck, 10000, DONAU_State.uPeriod);
    DONAU_State.charger.state = CHG_STATE_DISCHARGED;
    MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN);
}

static void init_rectifier()
{

    memset(&DONAU_State.rectifier,0,sizeof(DONAU_State.rectifier) - sizeof(DONAU_State.rectifier.init));
    Odt_(&DONAU_State.rectifier.odtDoOp, 1000, DONAU_State.uPeriod);
    DONAU_State.rectifier.state = REC_STATE_STOP;
    MCB_UpdateCmd(M_CB1_BATT, CB_CMD_OFF);
    EPWM_Converter_Disable();
//  MC_UpdateStatus(CTRL_MC1_ONOFF, OPEN);
}

static void init_inverter()
{

    memset(&DONAU_State.inverter,0,sizeof(DONAU_State.inverter) - sizeof(DONAU_State.inverter.init));
    Odt_(&DONAU_State.inverter.odtDoOp, 1000, DONAU_State.uPeriod);
    DONAU_State.inverter.state = SCC_STATE_STOP;
    MCB_UpdateCmd(M_CB2_SSW, CB_CMD_ON);
    GPIO_StaticSwitch(GPIO_STATIC_SW_ON);
    EPWM_Inverter_Disable();
//  MC_UpdateStatus(CTRL_MC2_ONOFF, CLOSED);
//  SSW_UpdateStatus(CLOSE);
}

Bool DONAU_is_active_all()
{
    return DONAU_State.bActive;
}

void DONAU_SetAllCommand(Bool onOff)
{
    DONAU_State.bActive = onOff;
}

void DONAU_Machine_Create()
{
    memset(&DONAU_State,0,sizeof(DONAU_State));
    DONAU_State.bActive = FALSE;
    DONAU_State.bHeavyFault = FALSE;
    DONAU_State.RDC_ON = FALSE;
    DONAU_State.IAC_ON = FALSE;
    DONAU_State.uPeriod = 5;
//TODO paramter test_mode assignment at start time
//  DONAU_State.uTestMode = ;
//
    DONAU_State.charger.init = init_charger;
    DONAU_State.rectifier.init = init_rectifier;
    DONAU_State.inverter.init = init_inverter;

    DONAU_State.charger.init();
    DONAU_State.rectifier.init();
    DONAU_State.inverter.init();

    DONAU_State.DCLinkV = 0;
    DONAU_State.DCInitMax = 700.0;
}
//
//
// MC2 and SSW default state(init state) are all CLOSED(ON) to inhibit Trans.
//
void SYS_UpdateSagCompensatorStatus(void)
{
    if(
        DONAU_State.bHeavyFault ||
        IND_STOP_OP(inverter) || (!IS_TEST_MODE() && (!IS_ALL_ACTIVE() || !IS_RUN(rectifier)))
    )
    {
        if(DONAU_State.inverter.state != SCC_STATE_STOP)
        {
            DONAU_State.inverter.state = SCC_STATE_STOP;
            DONAU_State.inverter.bOdtRun = FALSE;
            DONAU_State.IAC_ON = FALSE;
            EPWM_Inverter_Disable();
            MCB_UpdateCmd(M_CB2_SSW, CB_CMD_ON);
            GPIO_StaticSwitch(GPIO_STATIC_SW_ON);
        }
        return; // ON any state, Inverster stop condition will make inverter to STOP state.
    }

    switch(DONAU_State.inverter.state)
    {
        case SCC_STATE_STOP:
            if( IND_DO_OP(inverter) || ALL_DO_OP())
            {
                ODT_Initialize(&DONAU_State.inverter.odtDoOp);
                DONAU_State.inverter.bOdtRun = TRUE;
                DONAU_State.inverter.state = SCC_STATE_READY_ODT;
            }
            break;
// If during 1 second operation cmd is valid then Do oeperation.
        case SCC_STATE_READY_ODT:
            if(DONAU_State.inverter.bOdtRun)
            {
                    if( ODT_Update(&DONAU_State.inverter.odtDoOp, TRUE) == ODT_FINISH )
                    {
                        DONAU_State.inverter.state = SCC_STATE_READY;
                        DONAU_State.inverter.bOdtRun = FALSE;
                    }
            }
            break;
        case SCC_STATE_READY:
            DONAU_State.IAC_ON = TRUE;
            ODT_Initialize(&DONAU_State.inverter.odtDoOp);
            DONAU_State.inverter.bOdtRun = TRUE;
            DONAU_State.inverter.state = SCC_STATE_START_ODT;
            break;
        case SCC_STATE_START_ODT:

            if(DONAU_State.inverter.bOdtRun)
            {
                    if( ODT_Update(&DONAU_State.inverter.odtDoOp, TRUE) == ODT_FINISH )
                    {
                        DONAU_State.inverter.state = SCC_STATE_RUN;
                        DONAU_State.inverter.bOdtRun = FALSE;
                        DONAU_State.inverter.bRun = TRUE;
                        MCB_UpdateCmd(M_CB2_SSW, CB_CMD_OFF);
                        GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
                        EPWM_Inverter_Enable();
                    }
            }
            break;
        case SCC_STATE_RUN:
            break;
    }
}

void SYS_UpdateRectifierStatus(void)
{

    if(( DONAU_State.bHeavyFault ||
         IND_STOP_OP(rectifier) ||
         ALL_STOP_OP()) || (ALL_DO_OP() && !IS_RUN(charger)))
    {
        DONAU_State.rectifier.state = REC_STATE_STOP;
        DONAU_State.rectifier.bOdtRun = FALSE;
        DONAU_State.rectifier.bRun = FALSE;
        DONAU_State.RDC_ON = FALSE;
        EPWM_Converter_Disable();
        return; // 어떤 상태에 있어도 converter 멈춤 조건이면 멈춤으로
    }

    switch(DONAU_State.rectifier.state)
    {
        case REC_STATE_STOP:
            if( IND_DO_OP(rectifier) ||
               (ALL_DO_OP() && IS_RUN(charger)))
            {
                DONAU_State.rectifier.state = REC_STATE_READY;
                if (CTRL.INV.ctrl_mode == PARAM_CSI_PR && PRM_PCS[CONTROLLER_SEL].iValue == PARAM_CSI_DQ)
                {
                    CTRL.INV.ctrl_mode = PRM_PCS[CONTROLLER_SEL].iValue;
                    CTRL.INV.ctrl_mode_change_enb = ON;
                    CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;
                    CTRL.INV.filter_mode = FILTER_MODE_GC_SwF_1;
                    CTRL_INV_PRC_ApplyGC_GI_PI_Gain();
                    CTRL.INV.ctrl_mode_change_enb = ON;
                }
                else
                if (CTRL.INV.ctrl_mode == PARAM_CSI_DQ && PRM_PCS[CONTROLLER_SEL].iValue == PARAM_CSI_PR)
                {
                    CTRL.INV.ctrl_mode = PRM_PCS[CONTROLLER_SEL].iValue;
                    CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;
                    CTRL.INV.filter_mode = FILTER_MODE_GC_SwF_1;
                    CTRL_INV_PRC_ApplyGC_GI_PI_Gain();
                    CTRL.INV.ctrl_mode_change_enb = ON;
                }
            }
            break;
        case REC_STATE_READY:
                ODT_Initialize(&DONAU_State.rectifier.odtDoOp);
                DONAU_State.rectifier.bOdtRun = TRUE;
                DONAU_State.rectifier.state = REC_STATE_RUN_ODT;
                break;
        case REC_STATE_RUN_ODT:
                if(DONAU_State.rectifier.bOdtRun)
                {
                    if( ODT_Update(&DONAU_State.rectifier.odtDoOp, TRUE) == ODT_FINISH )
                    {
//Rectifier Run condition continues 1 second.

                        if( IND_DO_OP(rectifier) || (ALL_DO_OP() && IS_RUN(charger)))
                        {
                            DONAU_State.rectifier.bOdtRun = FALSE;
                            DONAU_State.rectifier.state = REC_STATE_RUN;
                            DONAU_State.rectifier.bRun = TRUE;
                            DONAU_State.RDC_ON = TRUE;
                            EPWM_Converter_Enable();
                            break;
                        }
                    }
                }
                else {} ; // ??
                break;
        case REC_STATE_RUN:
                break;
    }
}

//
// SYS_UpdateChargerStatus
//
//
//
void SYS_UpdateChargerStatus(void)
{
    if( DONAU_State.bHeavyFault ||
        IND_STOP_OP(charger) ||
        ALL_STOP_OP())
    {
        if(DONAU_State.charger.state != CHG_STATE_DISCHARGED)
        {
        DONAU_State.charger.bRun = FALSE;
        MCB_UpdateCmd(M_CB1_BATT, CB_CMD_OFF);
        MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN);
        DONAU_State.charger.state = CHG_STATE_DISCHARGED;
        INVERTER.bDcChargeOK = FALSE;
#ifdef DCLINK_SIMUL
        DONAU_State.DCLinkV = 0;
#endif
        }
        return;
    }
#ifdef DCLINK_SIMUL
    if(DONAU_State.charger.state == CHG_STATE_MC1AON) {
        DONAU_State.DCLinkV += 100;
        if(DONAU_State.DCLinkV >= DONAU_State.DCInitMax) DONAU_State.DCLinkV = DONAU_State.DCInitMax;
    }
#endif

    switch(DONAU_State.charger.state)
    {
        case CHG_STATE_DISCHARGED :
            if( IND_DO_OP(charger) || ALL_DO_OP())
            {
                if(DONAU_State.DCLinkV >= DONAU_State.DCInitMax) //  bypass initial charge because of enough voltage at CAP.
                {
                    MCB_UpdateCmd(M_CB1_BATT, CB_CMD_ON);
                    ODT_Initialize(&DONAU_State.charger.odtMC1_On);
                    DONAU_State.charger.bOdtRun = TRUE;
                    DONAU_State.charger.state = CHG_STATE_MC1REQ_ODT;
                    break;
                }
                MC_UpdateStatus(CTRL_MC1A_ONOFF, CLOSED); // recifier switch MC1A off
                ODT_Initialize(&DONAU_State.charger.odtMC1A_On);
                DONAU_State.charger.bOdtRun = TRUE;
                DONAU_State.charger.state = CHG_STATE_MC1AON_ODT;
            }
            break;
        case CHG_STATE_MC1AON_ODT:
            if( !DONAU_State.charger.bOdtRun || MC_GetStatus(STATUS_MC1A) ) // If MC1A switch NO verify  mode ?
            {
                ODT_Initialize(&DONAU_State.charger.odtDCLinkRisingCheck);
                DONAU_State.charger.bOdtRun = TRUE;
                DONAU_State.charger.state = CHG_STATE_MC1AON;
#ifdef DCLINK_SIMUL
                DONAU_State.DCLinkV = 0;
#endif
            } //
            else if( ODT_Update(&DONAU_State.charger.odtMC1A_On, TRUE) == ODT_FINISH )
            {
                MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN); // recifier switch MC1A off
                DONAU_State.charger.bOdtRun = FALSE;
                DONAU_State.charger.state = CHG_STATE_DISCHARGED;
// MC1A is not ON : error
// Something to do
            }
            break;
        case CHG_STATE_MC1AON :
            if( DONAU_State.charger.bOdtRun )// check DCLink for normal initial charging
            {
                if( ODT_Update(&DONAU_State.charger.odtDCLinkRisingCheck, TRUE) == ODT_FINISH )
                {
                    DONAU_State.charger.bOdtRun = FALSE;
                    MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN); // recifier switch MC1A off
                    DONAU_State.charger.state = CHG_STATE_DISCHARGED;
                }
                else if(DONAU_State.DCLinkV >= DONAU_State.DCInitMax) //
                {
                    MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN);
                    MCB_UpdateCmd(M_CB1_BATT, CB_CMD_ON);

                    ODT_Initialize(&DONAU_State.charger.odtMC1_On);
                    DONAU_State.charger.state = CHG_STATE_MC1REQ_ODT;
                }
            }
            else {}// Something wrong state..
            break;
        case CHG_STATE_MC1REQ_ODT:
            if( ODT_Update(&DONAU_State.charger.odtMC1_On, TRUE) == ODT_FINISH )
            { //  MC1 switch remains off after switch status check time.
                MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN);
                MCB_UpdateCmd(M_CB1_BATT, CB_CMD_OFF);
                DONAU_State.charger.bOdtRun = FALSE;
                DONAU_State.charger.state = CHG_STATE_DISCHARGED;
            }
            else if(MC_GetStatus(STATUS_MC1))
            {
                DONAU_State.charger.bOdtRun = FALSE;
                DONAU_State.charger.state = CHG_STATE_MC1REQ;
                break;
            }
            break;
        case CHG_STATE_MC1REQ :
            DONAU_State.charger.state = CHG_STATE_CHARGED;
            DONAU_State.charger.bRun = TRUE;
            INVERTER.bDcChargeOK = TRUE;
            break;
        case CHG_STATE_CHARGED  :
            if(MC_GetStatus(STATUS_MC1) == 0)
            {
                ODT_Initialize(&DONAU_State.charger.odtMC1_On);
                DONAU_State.charger.bOdtRun = TRUE;
                DONAU_State.charger.state = CHG_STATE_MC1REQ_ODT;
                DONAU_State.charger.bRun = FALSE;
            }
            break;
    }
}

extern Flt FLT;
#pragma CODE_SECTION (SYS_UpdateStateMachine, "dp_ctrl")

void SYS_UpdateStateMachine(void)
{
    extern int     InverterStateShadow;
    DONAU_State.bHeavyFault = FLT.uHeavyFlag;
#ifdef DCLINK_SIMUL
#else
#define ENTER_CRITICAL_SECTION()    DINT
#define EXIT_CRITICAL_SECTION()     EINT
    ENTER_CRITICAL_SECTION();
    DONAU_State.DCLinkV = BATCTRL.DCLinkV; //
    EXIT_CRITICAL_SECTION();
#endif
    SYS_UpdateChargerStatus();
    SYS_UpdateRectifierStatus();
    SYS_UpdateSagCompensatorStatus();
    InverterStateShadow = INVERTER.uStatus;

    switch(DONAU_State.charger.state)
    {
        case CHG_STATE_MC1AON:
            INVERTER.uStatus = SYS_INV_DC_CHARGE;
            INVERTER.bRun = FALSE;
            break;
        case  CHG_STATE_CHARGED:
            if(DONAU_State.rectifier.bRun)
            {
                INVERTER.uStatus = SYS_INV_RUN;
                INVERTER.bRun = TRUE;
            }
            break;
        case CHG_STATE_DISCHARGED:
            INVERTER.bRun = FALSE;
            if(DONAU_State.bHeavyFault)
            {
                INVERTER.uStatus = SYS_INV_FAULT;
            }
            else if(!IND_DO_OP(rectifier) && !ALL_DO_OP())
            {
                INVERTER.uStatus = SYS_INV_STOP;
            }
            break;
        default:
            INVERTER.bRun = FALSE;
            break;
    }
    switch(DONAU_State.rectifier.state)
    {
    case REC_STATE_STOP :
        if(DONAU_State.bHeavyFault)
        {
            INVERTER.uStatus = SYS_INV_FAULT;
        }
        else if(!IND_DO_OP(rectifier) && !ALL_DO_OP())
        {
            INVERTER.uStatus = SYS_INV_STOP;
        }
        break;
    case REC_STATE_READY :
        break;
    case REC_STATE_RUN_ODT :
        break;
    case REC_STATE_RUN :
        INVERTER.uStatus = SYS_INV_RUN;
        break;
    }

}
void DONAU_SetEachState(Uns wReg)
{
    if(wReg & 0x1) DONAU_State.charger.bActive = 1;
    else           DONAU_State.charger.bActive = 0;
    if(wReg & 0x2) DONAU_State.rectifier.bActive = 1;
    else           DONAU_State.rectifier.bActive = 0;
    if(wReg & 0x4) DONAU_State.inverter.bActive = 1;
    else           DONAU_State.inverter.bActive = 0;
}
