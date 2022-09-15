#if 1
UInt16 uiSystemStateTemp = 0;
#pragma CODE_SECTION (SYS_InverterStateTransition, "dp_ctrl")

static void SYS_InverterStateTransition(Uns tran)
{
    ODT_Initialize(INVERTER.odtCb1AFault);
    ODT_Initialize(INVERTER.odtDcCharge);
    ODT_Initialize(INVERTER.odtBattCbOn);
    ODT_Initialize(INVERTER.odtCb2Off);
    ODT_Initialize(INVERTER.odtDisconnect);
    ODT_Initialize(INVERTER.odtIinvSync);
    ODT_Initialize(INVERTER.odtIinvBuildup);
    ODT_Initialize(INVERTER.odtIinvSyncTimeOut);
    ODT_Initialize(INVERTER.odtGI_SlaveDelay);
    ODT_Initialize(INVERTER.odtMc1A);
    ODT_Initialize(INVERTER.odtBypNormal);
    ODT_Initialize(INVERTER.odtBypassDelay);
    ODT_Initialize(INVERTER.odtResyncCb2Delay);
    ODT_Initialize(GenBlock.odtGenNormal);
    ODT_Initialize(CTRL.BYP.SYNC.odtBypNormal);
    ODT_Initialize(GenBlock.odtGenSyncOk);
    ODT_Initialize(GenBlock.odtMC6OpenCheck);
    ODT_Initialize(INVERTER.odtSeamlessDisable);
    ODT_Initialize(INVERTER.odtHoldLoadP);
    ODT_Initialize(INVERTER.odtMC8Fault);
    ODT_Initialize(INVERTER.odtMC9Fault);

    RECLOSER_CLR_CMD();
    GenBlock.bMCOffCmd = FALSE;
    GenBlock.bGeneratorMC_Off_command = FALSE;
    GenBlock.bApply_w_ramp = FALSE;
    GenBlock.bSetInit_w_ramp = FALSE;
    EVT_Store_NoDupReset();
    uiSystemStateTemp = 0;

    switch (tran)
    {
    case SYS_INV_STATE_TRAN_FAULT: /****************************SYS_INV_STATE_TRAN_FAULT******************************/
        MO_SetState(PEBB_ALL_OFF);

        MCB_UpdateCmd(M_CB1_BATT, CB_CMD_TRIP);
        MCB_UpdateCmd(M_CB3, CB_CMD_TRIP);
        //MCB_UpdateCmd(M_CB4_SSW, CB_CMD_TRIP);    //  WESCO 요구사항 Fault 시에 CB4는 On
        MC_UpdateStatus(CTRL_MC8_ONOFF, OPEN);
        MC_UpdateStatus(CTRL_MC9_ONOFF, OPEN);
        MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN);

        EVT_Store(EVT_STAT_INV_FAULT);
        INVERTER.uStatus = SYS_INV_FAULT;

        /*
        if (FLT_GetEachStatus(FLTH_GRID_OF_LEVEL1) || FLT_GetEachStatus(FLTH_GRID_OF_LEVEL2) ||
                FLT_GetEachStatus(FLTH_GRID_UF_LEVEL1) || FLT_GetEachStatus(FLTH_GRID_OV_LEVEL1) || FLT_GetEachStatus(FLTH_GRID_UV_LEVEL1) ||
                FLT_GetEachStatus(FLTH_GRID_UF_LEVEL2) || FLT_GetEachStatus(FLTH_GRID_OV_LEVEL2) || FLT_GetEachStatus(FLTH_GRID_UV_LEVEL2) ||
                FLT_GetEachStatus(FLTL_GRID_UV) || FLT_GetEachStatus(FLTH_BATT_UV)
                //||
                //TODO Check (INVERTER.uStatus == SYS_INV_DISCONNECT && !SYS_ReConnectionCondition()))
                //-FIXMED (INVERTER.uStatus == SYS_INV_STOP && !SYS_ReConnectionCondition()))
                //-DELETED(INVERTER.uStatus == SYS_INV_STOP))
                )
        {
        }
        else
            SYS_SetInverterCommand(OFF); //. SWI에서 SYS_SetInverterCommand 함수 사용 X
        */

        break;
    case SYS_INV_STATE_TRAN_FAULT_RESET:/*****************************SYS_INV_STATE_TRAN_FAULT_RESET*****************************/
        MO_PebbFaultReset();

        MCB_UpdateCmd(M_CB1_BATT, CB_CMD_RESET);
        MCB_UpdateCmd(M_CB3, CB_CMD_RESET);
        // XXX TODO 에러 발생
        //if(_cbSSW.State == CB_TRIP)
            MCB_UpdateCmd(M_CB4_SSW, CB_CMD_RESET);

        INVERTER.uStatus = SYS_INV_STOP;
        EVT_Store(EVT_STAT_INV_STOP);

        break;
    case SYS_INV_STATE_TRAN_STOP://************************************SYS_INV_STATE_TRAN_STOP********************************/

        MO_SetState(PEBB_ALL_OFF);
        CTRL_INV_SEAMLESS_Reset();

        if( PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1 )
        {
            //MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);
            MCB_UpdateCmd(M_CB3, CB_CMD_OFF);
            MCB_UpdateCmd(M_CB1_BATT, CB_CMD_OFF);
        }
        else
        {
            //MC_UpdateStatus(CTRL_CB4_ON, OPEN);
            MC_UpdateStatus(CTRL_CB3_ON, OPEN);
            MC_UpdateStatus(CTRL_DS1_ON, OPEN);
        }
        MC_UpdateStatus(CTRL_MC8_ONOFF, OPEN);
        MC_UpdateStatus(CTRL_MC9_ONOFF, OPEN);
        MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN);

        GenBlock.bPccSyncOk = FALSE;
        CTRL.GRT.bEnable = FALSE; // 160912

        EVT_Store(EVT_STAT_INV_STOP);
        INVERTER.uStatus = SYS_INV_STOP;

        break;
    case SYS_INV_STATE_TRAN_SCR_ON:

        FLT_SetGridFailureEvent(OFF);
        INVERTER.gridReconnect.bEnb = FALSE;

        if( PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1 )
        {
            MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);
        }
        else
        {
            MC_UpdateStatus(CTRL_CB4_ON, OPEN);
        }

        EVT_Store(EVT_STAT_INV_SCR_ON);
        INVERTER.uStatus = SYS_INV_SCR_ON;

        CC_BCTransIdle();

        break;
    case SYS_INV_STATE_TRAN_DC_CHARGE://b:ON/**************************SYS_INV_STATE_TRAN_DC_CHARGE****************************/

        FLT_SetGridFailureEvent(OFF); // Stop 상태를 벗어날 때 Clear해야 함. GRID FAULT 가 SET 된 것을 CLEAR한다.
        INVERTER.gridReconnect.bEnb = FALSE;

        if(PRM_PCS[SYS_MO_ENABLE].iValue == 2)
            MO_SetState(PEBB_ALL_ON);

        MC_UpdateStatus(CTRL_MC1A_ONOFF, CLOSED); // 초충 CLOSED

        EVT_Store(EVT_STAT_INV_DC_CHARGE);
        INVERTER.uStatus = SYS_INV_DC_CHARGE;

        CC_BCTransIdle();

        break;
    case SYS_INV_STATE_TRAN_EVE_DC_CHARGE:

        FLT_SetGridFailureEvent(OFF);
        INVERTER.gridReconnect.bEnb = FALSE;

#if EVE_DC_CHARGER == 1
        MC_UpdateStatus(CTRL_MC9_ONOFF, CLOSED);
#endif

        EVT_Store(EVT_STAT_INV_EVE_DC_CHARGE);
        INVERTER.uStatus = SYS_INV_EVE_DC_CHARGE;
        //EVE.bCMD_FaultReset = TRUE;
        CC_BCTransIdle();

        break;
    case SYS_INV_STATE_TRAN_AC_GENERATE: //b:GRIDMC_ON:/***************SYS_INV_STATE_TRAN_AC_GENERATE**************************/
        CC_UpdatePLLGains(CC_PLL_VOLTAGE_BUILD);
        EVT_Store(EVT_STAT_INV_AC_GENERATE);
        pButton->bPwmOnOff = TRUE;
        OPTION.uPWM = 0; // 임시.
#if IIVC_ENB
        INVERTER.bInitInvOn = FALSE;
        INVERTER.bRequestInvThetaInit = TRUE;
#endif
        INVERTER.uStatus = SYS_INV_AC_GENERATE;
        CC_PCC_Gating();
        break;
    case SYS_INV_STATE_TRAN_START_SYNC: //b:MPP_INIT:/*****************SYS_INV_STATE_TRAN_START_SYNC**************************/
        CC_UpdatePLLGains(CC_PLL_VOLTAGE_BUILD);
        EVT_Store(EVT_STAT_INV_START_SYNC);
        INVERTER.uStatus = SYS_INV_START_SYNC;
        GenBlock.bPowerDecreaseEnable = FALSE;
        break;
    case SYS_INV_STATE_TRAN_RUN: //b:START:/***************************SYS_INV_STATE_TRAN_RUN*********************************/
        CC_BypOmegaReset();
        CC_UpdatePLLGains(CC_PLL_NORMAL);
        CTRL_GEN_ClearTimer();

        GenBlock.powerDecreaseState = GPC_HOLDING;
        GenBlock.autoChargingState = GAC_STANDBY;
        CTRL_GEN_RampInit();

        /*+ 160125 */
        CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;
        PRM_PCS[CTRL_SEAMLESS_UV_CNT_STATUS].iValue = 0;
        PRM_PCS[CTRL_SEAMLESS_OV_CNT_STATUS].iValue = 0;

        INVERTER.RUN_LOAD_OC_DELAY_CNT = 0;
        //-----------------------------------------------------

        EVT_Store(EVT_STAT_INV_RUN);
#if IIVC_ENB
        INVERTER.bInitInvOn = FALSE;
#endif
        INVERTER.bRun = TRUE;

        INVERTER.uStatus = SYS_INV_RUN;
        CC_PCC_Gating();
        break;
    case SYS_INV_STATE_TRAN_ISLANDING:/***************************************************************/
        EVT_Store(EVT_STAT_INV_ISLANDING);
        CTRL.GRT.bEnable = FALSE; // 160912

        /* 160411 */
        if( PRM_PCS[BYP_MODE].iValue == 20 )
            INVERTER.bRun = TRUE;

        PCC.fRefP = 0.;
        INVCTRL.IqeRef = 0.;

#if DBUG_MODE == 2
        //TODO VCB OPEN 가정
        GenBlock.bGenVCB_Status = FALSE;
#endif

        INVERTER.uStatus = SYS_INV_ISLANDING;

        // 150128 June
        CTRL.IRT.bPwmOff = TRUE;
        GenBlock.bPccSyncOk = FALSE;
        SAG_OP.bFilterDelayLatch = FALSE;
        break;
    case SYS_INV_STATE_TRAN_RESYNC:/******************************************************************/
        EVT_Store(EVT_STAT_INV_RESYNC);

        GenBlock.bSetInit_w_ramp = TRUE;
        GenBlock.bPccSyncOk = FALSE;
        PICON_Initialize(&CTRL.INV.SYNC.PI_MAG.CORE);
        PICON_Initialize(&CTRL.INV.SYNC.PI_MAG_BYP.CORE);

        INVERTER.uStatus = SYS_INV_RE_SYNC;
        break;
#ifdef BYP_EVT_OPERATION
    case SYS_INV_STATE_TRAN_BYP_EVT_OPERATION:
        EVT_Store(EVT_STAT_INV_BYP_EVT_OPERATION);

        INVERTER.uStatus = SYS_INV_BYP_EVT_OPERATION;
        break;
#endif

    case SYS_INV_STATE_TRAN_TEST_ON:/*****************************************************************/
        EVT_Store(EVT_STAT_INV_TEST);
        INVERTER.uStatus = SYS_INV_TEST_MODE;
        break;
    case SYS_INV_STATE_TRAN_TEST_OFF:/****************************************************************/
        EVT_Store(EVT_STAT_INV_STOP);
        INVERTER.uStatus = SYS_INV_STOP;

        if( PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1 )
        {
            //MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);
            MCB_UpdateCmd(M_CB3, CB_CMD_OFF);
            MCB_UpdateCmd(M_CB1_BATT, CB_CMD_OFF);
        }
        else
        {
            //MC_UpdateStatus(CTRL_CB4_ON, OPEN);
            MC_UpdateStatus(CTRL_CB3_ON, OPEN);
            MC_UpdateStatus(CTRL_DS1_ON, OPEN);
        }

        MC_UpdateStatus(CTRL_MC8_ONOFF, OPEN);
        MC_UpdateStatus(CTRL_MC9_ONOFF, OPEN);
        MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN);

        break;
    }
}


/*
 * @relation
 *  - obj
 *    BC_fsm
 *  - called by
 *    c_int25()
 *
 */

#pragma CODE_SECTION (SYS_UpdateInverterStatus, "dp_ctrl")
static void SYS_UpdateInverterStatus(void)
{
    volatile Uns bInvCmdOn;
    volatile float fDCLinkVolt;
    volatile Uns flgOdt;
    volatile Uns flgOdt_sub;
    volatile Bool bResult;
    volatile float fCalcVar;

    /* ON-OFF COMMAND CHECK */
    bInvCmdOn = SYS_CheckInvCmdOn();

    //  16.07.28 Yang
    //CTRL_BYP_EVT_OPERATION_Proceed();
    Sag_Evt_History_Proceed();
    MCB_Abnormal_Operation_Check();
#if EVE_DC_CHARGER == 1
    if(PRM_PCS[EVE_DC_CHARGER_ENABLE].iValue == TRUE)
        EVE_Proceed();
#endif
    SYS_FAN_Control();

    /* DC POLARITY CHECK */
    fDCLinkVolt = BATCTRL.fltDCBattV.fOut; //  주의! 2단 필터 사용
    if (fDCLinkVolt < -20.)
        FLT_Raise(FLTH_BATT_POLARITY);

    if( uiSysCallCount++ > 40 ) // 100ms
    {
        uiSysCallCount = 0;
    }

    if((INVERTER.uStatus == SYS_INV_ISLANDING) || (INVERTER.uStatus == SYS_INV_RE_SYNC))
        MC_UpdateStatus(CTRL_DDO_REV4, CLOSED);
    else
        MC_UpdateStatus(CTRL_DDO_REV4, OPEN);

    if( PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1 )
    {
        if(INVERTER.uStatus == SYS_INV_FAULT)
            MC_UpdateStatus(CTRL_SSEFAN_CTRL, CLOSED);
        else
            MC_UpdateStatus(CTRL_SSEFAN_CTRL, OPEN);
    }

    if( PRM_PCS[BYP_MODE].iValue == 20 )
    {
        //GPIO_UpdateStatus();

        /*  2016.06.15  Yang    */
#if DBUG_MODE == 2
        if( PRM_PCS[INV_CAB_T_MAX].iValue == 70 )
        {
            GenBlock.bypSw = 0;
        }
        if( PRM_PCS[INV_CAB_T_MAX].iValue == 80 )
        {
            GenBlock.bypSw = 1;
        }
        if( PRM_PCS[INV_CAB_T_MAX].iValue == 90 )
        {
            GenBlock.bBypV = 0;
        }
        if( PRM_PCS[INV_CAB_T_MAX].iValue == 100 )
        {
            GenBlock.bBypV = 1;
        }
#endif  //  #if DBUG_MODE == 2
    }

    InverterStateShadow = INVERTER.uStatus;

    switch (INVERTER.uStatus)
    {
    case SYS_INV_FAULT:
        PWM_Disable(PWM_CON);
        INVERTER.bRun = FALSE;
        pButton->bStart = FALSE;
#if IIVC_ENB
        INVERTER.bInitInvOn = FALSE;
#endif
        pButton->bTestRun = FALSE;
        pButton->bPwmOnOff = ON;
        CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;

        //-EVE.bCMD_On = OFF;   // EVE OFF
        //-EVE.bCMDCharge = OFF;    //  EVE Charge Off

        // CSI 모드로 전환
        if (CTRL.INV.ctrl_mode != PRM_PCS[CONTROLLER_SEL].iValue || CTRL.INV.filter_mode == FILTER_MODE_IS_SwF_2 )
        {
            CTRL.INV.ctrl_mode = PRM_PCS[CONTROLLER_SEL].iValue;
            CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;
            CTRL.INV.filter_mode = FILTER_MODE_GC_SwF_1;
            CTRL_INV_PRC_ApplyGC_GI_PI_Gain();
            CTRL.INV.ctrl_mode_change_enb = ON;
        }

#if 0
        // 그리드 문제 발생시 ES-LV 모델 또는 5분간 재투입 방지 후 자동 시스템 ON 옵션이 활성화 되 있으면 인버터 측 리셋 수행
        if( ((FLT_GetGridFailureEvent() == ON || FLT_GetEachStatus(FLTH_BATT_UV))&& PRM_PCS[SYS_OPTION].iValue & 0x10 && !FLT_GetEachStatus(FLTL_DOOR_AC_OPEN)) || eslvCtrl.g_bEnb )
        {
            // Grid Fault로 인해 발생한 다른 고장은 Clear
            if (FLT_GetHeavyStatus())
            {
                //FLT_Initialize();
                FLT_Clear(FLTH_BATT_UV);
                /*
                FLT_Clear(FLTH_BATT_OV);
                FLT_Clear(FLTH_BATT_OC);

                FLT_Clear(FLTH_INV_OV);
                FLT_Clear(FLTH_INV_UV);
                FLT_Clear(FLTH_INV_OF);
                FLT_Clear(FLTH_INV_UF);
                FLT_Clear(FLTH_INV_OC);
                FLT_Clear(FLTH_INV_PHASE);
                */
            }
        }
#endif

        if( PRM_PCS[BYP_MODE].iValue == 20 )
            SYS_Byp20_StopFault();
        else
        {
            GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);

            if( MC_GetStatus(STATUS_CB4) )
            {
                if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
                    MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);
                else
                    MC_UpdateStatus(CTRL_CB4_ON, OPEN);
            }
        }

        if (!FLT_GetHeavyStatus() && ( (/*MC_GetStatus(STATUS_CB4) &&*/ (PRM_PCS[BYP_MODE].iValue == 20))
                || (PRM_PCS[BYP_MODE].iValue == 0) ) )
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT_RESET);
        }

        break;

    case SYS_INV_STOP:
        PWM_Disable(PWM_CON);
        INVERTER.bRun = FALSE;
        pButton->bTestRun = FALSE;
        INVERTER.bDcChargeOK = FALSE;
        pButton->bStart = FALSE;
        pButton->bPwmOnOff = ON;

        FLT_Clear(FLTL_DPWM_ON);

        // CSI 모드로 전환
        if (CTRL.INV.ctrl_mode != PRM_PCS[CONTROLLER_SEL].iValue || CTRL.INV.filter_mode == FILTER_MODE_IS_SwF_2 )
        {
            CTRL.INV.ctrl_mode = PRM_PCS[CONTROLLER_SEL].iValue;
            CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;
            CTRL.INV.filter_mode = FILTER_MODE_GC_SwF_1;
            CTRL_INV_PRC_ApplyGC_GI_PI_Gain();
            CTRL.INV.ctrl_mode_change_enb = ON;
        }

        GenBlock.bGenVCB_On_Req = FALSE;
        GenBlock.bPowerDecreaseEnable = FALSE;
        CTRL.INV.SEAMLESS.pcc_blackout_enb = FALSE;

        if (FLT_GetHeavyStatus())
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
            break;
        }

        if (PRM_PCS[CTRL_TEST_MODE].iValue == 1) // Normal Mode or Test Mode 2
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_TEST_ON);
            break;
        }

        Prevent_Immediate_Grid_Reconnect();

        if( PRM_PCS[BYP_MODE].iValue == 20 )
            SYS_Byp20_StopFault();
        else
        {
            GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);

            if( MC_GetStatus(STATUS_CB4) )
            {
                if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)
                    MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);
                else
                    MC_UpdateStatus(CTRL_CB4_ON, OPEN);
            }
        }

        if (!INVERTER.gridReconnect.bEnb && bInvCmdOn
                /*&& (BATCTRL.fltDCBattVForCC.fOut >= PARAM_VAL(BATT_UV_LEVEL))*/)
        {
            if (MCB_GetStatus(M_CB1_BATT) == CB_OFF)
            {
                bResult = TRUE;

                if( PRM_PCS[BYP_MODE].iValue == 20 )
                {
                    if( !CTRL_BYP_NormalOK() )
                    {
                        bResult = FALSE;
                    }
                }
            }
        }
        else
        {
            bResult = FALSE;
        }

#if USE_MBP_PANNEL == 1
    #if 0
        if( PRM_PCS[BYP_MODE].iValue == 20 && MC_GetStatus(STATUS_EX_MCCB1) )
            flgOdt = ODT_Update(INVERTER.odtDisconnect, bResult);
        if (flgOdt == ODT_BROKEN)
        {
            ODT_Initialize(INVERTER.odtDisconnect);
        }
        else if (flgOdt == ODT_FINISH && MC_GetStatus(STATUS_EX_MCCB1)/* CLOSED */ && MC_GetStatus(STATUS_EX_MCCB2)/* CLOSED */
                && !MC_GetStatus(STATUS_EX_MCCB3)/* OPEN */ && CTRL_BYP_NormalOK() )
    #else
        if(PRM_PCS[BYP_MODE].iValue == 20 && CTRL_BYP_NormalOK() && !FLT_GetEachStatus(FLTL_GRID_UV)
            && MC_GetStatus(STATUS_EX_MCCB1) && MC_GetStatus(STATUS_EX_MCCB2) && !MC_GetStatus(STATUS_EX_MCCB3) )
    #endif
#else
        if ( PRM_PCS[BYP_MODE].iValue == 20 && CTRL_BYP_NormalOK() && !FLT_GetEachStatus(FLTL_GRID_UV))
#endif // #if USE_MBP_PANNEL == 1
        {
            flgOdt = ODT_Update(INVERTER.odtDisconnect, bResult);

            if( flgOdt == ODT_FINISH )
                SYS_InverterStateTransition(SYS_INV_STATE_TRAN_SCR_ON);
        }
        else if(PRM_PCS[BYP_MODE].iValue == 0)
        {
            flgOdt = ODT_Update(INVERTER.odtDisconnect, bResult);

            if (flgOdt == ODT_FINISH)
                SYS_InverterStateTransition(SYS_INV_STATE_TRAN_DC_CHARGE);
        }
        else
            ODT_Initialize(INVERTER.odtDisconnect);

        break;

    case SYS_INV_SCR_ON:
        if (FLT_GetHeavyStatus())
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
            break;
        }
        if (!bInvCmdOn)
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_STOP);
            break;
        }

        CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;

        if( PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1 )
        {
            MCB_UpdateCmd(M_CB4_SSW, CB_CMD_OFF);
        }
        else
        {
            MC_UpdateStatus(CTRL_CB4_ON, OPEN);
        }

        if(CTRL_BYP_NormalOK())
            GPIO_StaticSwitch(GPIO_STATIC_SW_ON);

#if USE_MBP_PANNEL == 1
        if ( MC_GetStatus(STATUS_EX_MCCB1)/* CLOSED */ && MC_GetStatus(STATUS_EX_MCCB2)/* CLOSED */
                 && !MC_GetStatus(STATUS_EX_MCCB3)/* OPEN */ && !MC_GetStatus(STATUS_CB4)/* OPEN */ && CTRL_Vpcc_NormalOK() )
#else
        if ( !MC_GetStatus(STATUS_CB4)/* OPEN */ && CTRL_Vpcc_NormalOK() )
#endif // #if USE_MBP_PANNEL == 1
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_DC_CHARGE);
        }
        break;

    case SYS_INV_DC_CHARGE:
        if (FLT_GetHeavyStatus())
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
            break;
        }
        if (!bInvCmdOn)
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_STOP);
            break;
        }

        CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;

//      // CB1A Fault Check
        // 3초내에 ON1A가 ON 되지 않았으면, FAULT 발생.
        if( ODT_Update(INVERTER.odtCb1AFault, TRUE) == ODT_FINISH )
        {
            if( MC_GetStatus(STATUS_MC1A2A) == OPEN )
            {
                FLT_Raise(FLTH_MC1A);
                SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
                break;
            }
        }

        /* 10 초 후에 DC Link V 체크. */
        flgOdt = ODT_Update(INVERTER.odtDcCharge, TRUE);
        if (flgOdt == ODT_FINISH)
        {
            /* 배터리 전압의 70% 이상이면 OK.  요청:김민국 수석 */
            if ( (BATCTRL.DCLinkV >= (BATCTRL.fltDCBattV.fOut * 0.70)) || (BATCTRL.fltDCBattV.fOut <= 30. )  )
            {
                if(PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1)/* HIL 차단기 모드 */
                    MCB_UpdateCmd(M_CB1_BATT, CB_CMD_ON); //  배터리 CB close
                else
                    MC_UpdateStatus(CTRL_DS1_ON, CLOSED);

                if(ODT_Update(INVERTER.odtBattCbOn, TRUE ) == ODT_FINISH)
                {
                    if(MC_GetStatus(STATUS_DS1))
                    {
                        INVERTER.bDcChargeOK = TRUE;

                        if( flgOdt == ODT_FINISH || flgOdt == ODT_BROKEN  )
                        {
                            ODT_Initialize(INVERTER.odtCb1AFault); /* 150807 명령 전송 후에는 체크하지 않도록 */
                            MC_UpdateStatus(CTRL_MC1A_ONOFF, OPEN);

                            if( MC_GetStatus(STATUS_MC1A2A) == OPEN )
                            {
                                SYS_UpdateLI();

                                if( PRM_PCS[BYP_MODE].iValue == 20 )
                                    SYS_InverterStateTransition(SYS_INV_STATE_TRAN_EVE_DC_CHARGE);
                                else if( PRM_PCS[BYP_MODE].iValue == 0 )
                                    SYS_InverterStateTransition(SYS_INV_STATE_TRAN_AC_GENERATE);
                            }
                        }
                    }
                    else
                    {
                        FLT_Raise(FLTH_BATT_CB1);
                        SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
                    }
                }
                else
                {
                     // odtBatteryCbOn working...
                    break;
                }
            }
            else
            {
                // Fault 상태로 이동할 때 MCB는 모두 자동으로 TRIP 되므로, 여기에서 OPEN 하지 않아도 된다.
                // Start Failure + 141211 DC 충전 실패
                FLT_Raise(FLTH_START_FAILURE);
                SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);

                break;
            }
        }
        break;

    case SYS_INV_EVE_DC_CHARGE:
        if (FLT_GetHeavyStatus())
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
            break;
        }

        if (!bInvCmdOn)
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_STOP);
            break;
        }

        if(EVE.bFaultStatus)
            FLT_Raise(FLTH_START_FAILURE);

#if EVE_DC_CHARGER != 1
        if( BATCTRL.DCLinkV > PARAM_VAL(AC_GEN_DC_VOLTAGE_LEVEL))
#else
        if( MC_GetStatus(STATUS_MC8) && !MC_GetStatus(STATUS_MC9)
                && (BATCTRL.DCLinkV > PARAM_VAL(AC_GEN_DC_VOLTAGE_LEVEL)) && (EVE.bStatus == ON) && (EVE.bCMDCharge == ON) )
#endif
        {
            if ( ODT_Update(INVERTER.odtDcCharge, TRUE) == ODT_FINISH)
                SYS_InverterStateTransition(SYS_INV_STATE_TRAN_AC_GENERATE);
        }
        else
        {
            ODT_Initialize(INVERTER.odtDcCharge);
        }

        break;

    case SYS_INV_AC_GENERATE:
        if (FLT_GetHeavyStatus())
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
            break;
        }

        if (!bInvCmdOn)
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_STOP);
            break;
        }

        if ( ODT_Update(INVERTER.odtCb1AFault, TRUE) == ODT_FINISH)
        {
            // 3s after
            if(MC_GetStatus(STATUS_MC1A2A))
            {
                FLT_Raise(FLTH_MC1A);
                SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
                break;
            }
        }
        else
        {
            // odt working....
            break;
        }//  odt finished

#if IIVC_ENB
        //Start Voltage Build
        INVERTER.bInitInvOn = TRUE; // => CC의 InitINV State ON(PWM ON) - 전압 발생 ON
#endif
        flgOdt = ODT_Update(INVERTER.odtIinvBuildup, TRUE); // CTRL_IINV_VOLTAGE_BUILDUP_TIME 1.5s
        if (flgOdt == ODT_FINISH)
        {
            if( CC_VoltageGenerateOK() && CTRL_Vpcc_NormalOK() )
            {
                // Remove Start Failure
                uiSystemStateTemp = 1;

                if(  (PRM_PCS[TEST_MODE2].iValue == 3) )
                    break;

                if ( (/*BYP MODE*/ PRM_PCS[BYP_MODE].iValue == 20 ) && CTRL.INV.operation_mode == PARAM_OPERATION_MODE_AUTO_IS
                        && (  ACP.BYP.v_pk < ACP.PCC.RATE.Vph_pk * 0.9 ))
                {
                    INVERTER.mc4On_timer_start = ON;
                }
                else
                {
                    SYS_InverterStateTransition(SYS_INV_STATE_TRAN_START_SYNC);
                }
            }
            else
            {
                if( !uiSystemStateTemp )
                {
                    // Start Failure + 141211 전압 Under Voltage
                    FLT_Raise(FLTH_START_FAILURE);
                    FLT_Raise(FLTH_INV_UV);
                    break;
                }
            }
        }
        else
        {
            break; // ODT ING...
        }
        break;

    case SYS_INV_START_SYNC:
        if (FLT_GetHeavyStatus())
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
            break;
        }
        if (!bInvCmdOn)
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_STOP);
            break;
        }

        if ( !(PRM_PCS[TEST_MODE2].iValue == 1))
        {
        //-------------------------------------------------------------------------------------
        flgOdt = ODT_Update(INVERTER.odtIinvSync, TRUE); // CTRL_IINV_PLL_STABILIZE_TIME 200ms
        if( flgOdt == ODT_ING)
        {
#if !DBUG_MODE || DBUG_MODE == 3
            // 200ms 동안 Sync OK가 지속되어야 SUCC
            if ( !CC_SyncOK() ) // 싱크가 맞지 않으면 Odt 초기화 하여 다시 200ms 동안 지속되는지 확인.
            {
                ODT_Initialize(INVERTER.odtIinvSync);
            }
#endif
            }
            else if (flgOdt == ODT_FINISH)
            {
                // 200 MS동안 Sync OK 가 지속 되었다면
                if( PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1 )
                    MCB_UpdateCmd(M_CB3, CB_CMD_ON);
                else
                    MC_UpdateStatus(CTRL_CB3_ON, CLOSED);

                INVERTER.mc4On_timer_start = ON;

                if(MC_GetStatus(STATUS_CB3)) //  일정 시간 내에 CLOSE 되지 않으면, MCCB 개체 내부에서 Fault 를 발생 시킴. 따라서 따로 odt를 두지 않음.
                {
                    bDcChargerOnOff = FALSE;    // 2016.06.10 Yang DC Charger Off

                    SYS_InverterStateTransition(SYS_INV_STATE_TRAN_RUN);

                    if (PRM_PCS[TRC_TRACE_MODE].iValue == TRC_VI_ENB_STOP)
                        TRC_StopTrace();
                    break;
                }
                // 싱크 완료 후, MC4를 Close 명령 후에는 Sync Time out 체크 하지 않도록 break 함.
                break;
            }

            //3초 간 싱크가 되지 않으면 중고장 Sync Timeout
            flgOdt = ODT_Update(INVERTER.odtIinvSyncTimeOut, TRUE);
            if (flgOdt == ODT_FINISH)
            {
                FLT_Raise(FLTH_ASYNC);
                break;
            }
        }
        break;

    case SYS_INV_RUN:
        if (FLT_GetHeavyStatus())
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
            break;
        }
        if (!bInvCmdOn)
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_STOP);
            break;
        }

        if( PRM_PCS[SYS_OPTION].iValue & 0x80 )
        {
            if( PARAM_RAW_VAL(BATT_LOCAL_POWER_FLOW) != PFLOW_DISCHARGE)
            {
                PARAM_RAW_VAL(BATT_LOCAL_POWER_FLOW) = PFLOW_DISCHARGE;
                BCState.powerFlow = (POWER_FLOW) PARAM_VAL(BATT_LOCAL_POWER_FLOW);
            }
        }

        if( PRM_PCS[BYP_MODE].iValue == 20 )
        {
            // 160912
            if( CTRL.GRT.bEnable )
            {
                // 1:5ms
                if( (CTRL.GRT.iSkipFreqFaultCnt++) * 5 >= PARAM_VAL(BESS71) )
                {
                    CTRL.GRT.bEnable = FALSE;
                }
                else
                {
                    // bEnable keep TRUE
                }
            }
            else
            {
                CTRL.GRT.iSkipFreqFaultCnt = 0;
            }
#if 0
            CTRL_GEN_SetCharge();
#else
            if((BATCTRL.fltDCBattV_2nd.fOut >= PARAM_VAL(BATT_V_RANGE_MAX) - 6.))
            {
                //if( (BATCTRL.fltDCBattV_2nd.fOut < PARAM_VAL(BATT_V_RANGE_MAX)) )
                    //PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = -15; // 1.5kW 충전
                //else
                    PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0;
            }
            if (BATCTRL.fltDCBattV_2nd.fOut <= PARAM_VAL(BATT_V_RANGE_MAX) - 50.)
            {
                if(INVERTER.RUN_LOAD_OC_DELAY_CNT < 200)
                {
                    INVERTER.RUN_LOAD_OC_DELAY_CNT++;
                    PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0;
                }
                else if(PARAM_VAL(INV_CAPACITY) < (EXCTRL.LoadPowerP * 0.001 + 35. - PARAM_VAL(AUTO_CHARGE_P_REF) ))
                {
#if 0
                    PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 10. * (EXCTRL.LoadPowerP * 0.001 + 25. - PARAM_VAL(INV_CAPACITY));
                    if(PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) < PRM_PCS[AUTO_CHARGE_P_REF].iValue)
                        PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = PRM_PCS[AUTO_CHARGE_P_REF].iValue;
                    else if(PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) > 0)
                        PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0;
#else
                    PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = 0.5 * PRM_PCS[AUTO_CHARGE_P_REF].iValue;
#endif
                }
                else
                    PARAM_RAW_VAL(BATT_REMOTE_PCC_P_REF) = PRM_PCS[AUTO_CHARGE_P_REF].iValue;
            }
            else
                INVERTER.RUN_LOAD_OC_DELAY_CNT = 0;
#endif
        }
        else
            CTRL.GRT.bEnable = FALSE;

        /*
         * ACP.BYP.blackout_enb<CTRL_BYP_SYNC.c> : (PARAM_OPERATION_MODE_AUTO_IS && SYS_INV_RUN && CTRL.INV.SEAMLESS.pcc_blackout_enb) ON일 경우 SET됨.
         * CTRL.INV.SEAMLESS.pcc_blackout_enb<CTRL_INV_SEAMLESS.c> :
         *  PARAM_OPERATION_MODE_AUTO_IS &&  SYS_INV_RUN &&
            ( ctrl_seamless_enb_instant_fault(PCC ov, uv counting) || GRID_OV || GRID_UV || GRID_OF || GRID_UF)
         *
         */
        if (CTRL.INV.SEAMLESS.pcc_blackout_enb)
        {
            gd(14);

            if(PRM_PCS[SEAMLESS_FAST_SCR_OFF].iValue == 0)
            {
                if( !GPIO_GetStaticSwitchOn() )
                    SYS_InverterStateTransition(SYS_INV_STATE_TRAN_ISLANDING);
            }
            else
                SYS_InverterStateTransition(SYS_INV_STATE_TRAN_ISLANDING);
            break;
        }

        #define PWM_OFF_GCGI()  (CTRL.INV.operation_mode == PARAM_OPERATION_MODE_GC || PARAM_OPERATION_MODE_AUTO_IS)

        // LI update
        SYS_UpdateLI();
        SYS_CheckBatteryCurrent();

        break;

    case SYS_INV_ISLANDING:

        if (FLT_GetHeavyStatus())
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
            if ( CTRL.INV.operation_mode == PARAM_OPERATION_MODE_AUTO_IS )
            {
                CTRL.INV.ctrl_mode = PRM_PCS[CONTROLLER_SEL].iValue;
                CTRL.INV.ctrl_mode_change_enb = ON;
                CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;
            }
            break;
        }
        if (!bInvCmdOn)
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_STOP);
            if ( CTRL.INV.operation_mode == PARAM_OPERATION_MODE_AUTO_IS )
            {
                CTRL.INV.ctrl_mode = PRM_PCS[CONTROLLER_SEL].iValue;
                CTRL.INV.ctrl_mode_change_enb = ON;
                CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;
            }
            break;
        }

        if( PRM_PCS[BYP_MODE].iValue == 20 )
        {
#if 0
             if( GPIO_GetStaticSwitch() == GPIO_STATIC_SW_ON )
             {
                 GPIO_StaticSwitchOff();
             }
#else
             GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
#endif

             /*
              * +190513 Sag Duration
              * Filter 성분이 강해서 전압 0%에서 100% 올라가는데 약 200ms이상 소요,
              * Input전압 peak로는 불평형일 경우 Sin파가 되므로 FastRms로 측정하도록 추가
              */
             if( EXCTRL.fltFsBypV[0].fPrevIn < EXCTRL.fltFsBypV[1].fPrevIn)
                 fCalcVar = EXCTRL.fltFsBypV[0].fPrevIn;
             else
                 fCalcVar = EXCTRL.fltFsBypV[1].fPrevIn;
             if( EXCTRL.fltFsBypV[2].fPrevIn < fCalcVar )
                 fCalcVar = EXCTRL.fltFsBypV[2].fPrevIn;

             if( fCalcVar > CTRL.INV.SEAMLESS.pcc_uv_level1_instant_restore )
             {
                 if( SAG_OP.bFilterDelayLatch )
                     SAG_OP.Sag_time_odt_flag = FALSE;
                 else
                     SAG_OP.Sag_time_odt_flag = TRUE;
             }
             else
             {
                 SAG_OP.bFilterDelayLatch = TRUE;
                 SAG_OP.Sag_time_odt_flag = TRUE;
             }


             if( (ACP.BYP.v_pk > CTRL.INV.SEAMLESS.pcc_uv_level2_instant_restore)
                     && fBypassE[0] > CTRL.INV.SEAMLESS.pcc_uv_level1_instant_restore && fBypassE[1] > CTRL.INV.SEAMLESS.pcc_uv_level1_instant_restore && fBypassE[2] > CTRL.INV.SEAMLESS.pcc_uv_level1_instant_restore)
            {
                // Sag_time_odt_flag = FALSE;
                ODT_Initialize(GenBlock.odtGenNormal);

                // BYPASS SYNC
                flgOdt = ODT_Update(INVERTER.odtBypNormal, TRUE);
                if( flgOdt == ODT_ING )
                {
                    //if ( !CTRL_BYP_NormalOK() )
                     if( (ACP.BYP.v_pk < CTRL.INV.SEAMLESS.pcc_uv_level2_instant_restore)
                             || fBypassE[0] < CTRL.INV.SEAMLESS.pcc_uv_level1_instant_restore || fBypassE[1] < CTRL.INV.SEAMLESS.pcc_uv_level1_instant_restore || fBypassE[2] < CTRL.INV.SEAMLESS.pcc_uv_level1_instant_restore)
                    {
                        ODT_Initialize(INVERTER.odtBypNormal);
                        gd(24);
                    }
                    else
                    {
                        gd(25);
                    }
                }
                else if (flgOdt == ODT_FINISH)
                {
                    CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;
                    gd(26);

                    if(!SAG_OP.bFilterDelayLatch)
                        INVERTER.odtSagCompTime->uCount -= SYS_INV_ODT_RESYNC_BYP_OK / SAG_STATE_CALL_PERIOD - 1;

                    SYS_InverterStateTransition(SYS_INV_STATE_TRAN_RESYNC);
                    break;
                }
            }
             else
             {
                 // Sag_time_odt_flag = TRUE;
             }
        }// BYP_MODE 20

        break;

    case SYS_INV_RE_SYNC:
        if (FLT_GetHeavyStatus())
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
            break;
        }
        if (!bInvCmdOn)
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_STOP);
            break;
        }

        // for DEBUG
        if (PRM_PCS[TEST_MODE2].iValue == 2)
            break;
/*
        if( PRM_PCS[BYP_MODE].iValue == 20 )
        {
            if((BATCTRL.fltDCBattV.fOut < PARAM_VAL(BATT_UV_LEVEL)) && (MC_GetStatus(STATUS_CB3)))
                MC_UpdateStatus(CTRL_CB3_ON, OPEN);
        }

        if(!MC_GetStatus(STATUS_CB3))
             SYS_InverterStateTransition(SYS_INV_STATE_TRAN_EVE_DC_CHARGE);
*/
        //Sync Fail Check ==============================================
        bResult = TRUE;

        //3초 간 싱크가 되지 않으면 중고장 Sync Timeout
        flgOdt = ODT_Update(INVERTER.odtIinvSyncTimeOut, bResult);
        if (flgOdt == ODT_FINISH)
        {
            FLT_Raise(FLTH_ASYNC);
            break;
        }
        //==============================================================

#if 1
        if( (ACP.BYP.v_pk < CTRL.INV.SEAMLESS.pcc_uv_level2_instant_restore)
                 || fBypassE[0] < CTRL.INV.SEAMLESS.pcc_uv_level1_instant_restore || fBypassE[1] < CTRL.INV.SEAMLESS.pcc_uv_level1_instant_restore || fBypassE[2] < CTRL.INV.SEAMLESS.pcc_uv_level1_instant_restore)
#else
        if( !CTRL_BYP_NormalOK() )
#endif
        {
            if( GPIO_GetStaticSwitch() && PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1 )
            {
                GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
            }

            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_ISLANDING);
            break;
        }

        flgOdt = ODT_Update(INVERTER.odtIinvSync, TRUE); // CTRL_IINV_PLL_STABILIZE_TIME 200ms
        if( flgOdt == ODT_ING)
        {
            if( !CTRL_BYP_PCC_SyncOK() ) // 싱크가 맞지 않으면 Odt 초기화 하여 다시 200ms 동안 지속되는지 확인.
                ODT_Initialize(INVERTER.odtIinvSync);

            gd(55);
            GenConn.bPredictGenConnect = FALSE;
        }
        else if (flgOdt == ODT_FINISH)
        {
            gd(52);
            // 싱크 Fail fault Clear
            ODT_Initialize(INVERTER.odtIinvSyncTimeOut);

            switch( PRM_PCS[BYP_MODE].iValue )
            {
            case 20:
                gd(54);
                if(PRM_PCS[TEST_MODE2].iValue == 4)
                    break;

                if( CTRL_BYP_PCC_SyncOK() )
                {
                    //Bypass line sync ok
                    GPIO_StaticSwitch(GPIO_STATIC_SW_ON);

                    gd(28);
                }

                if (CTRL.INV.ctrl_mode == PARAM_VSI_PR && PRM_PCS[CONTROLLER_SEL].iValue == PARAM_CSI_PR)
                {
                    CTRL.INV.ctrl_mode = PARAM_CSI_PR;
                }
                //CC_BCTransIdle();
                CTRL.INV.ctrl_mode_change_enb = ON;
                CTRL.INV.SEAMLESS.pcc_blackout_enb = OFF;

                CTRL.GRT.bEnable = TRUE; /* Resync -> 계통 연계 시 특정 시간 동간 OF, UF를 검출하지 않도록 */
                CTRL.GRT.iSkipFreqFaultCnt = 0;

                gd(27);

#if 1
                if( GPIO_GetStaticSwitch() && PRM_PCS[SYS_DPWM_OFFTEMP].iValue != 1 )
                {
                    //EVT_Store_NoDup(EVT_SSW_ON);
                    SYS_InverterStateTransition(SYS_INV_STATE_TRAN_RUN);
                }
                else if( /*MC_GetStatus(STATUS_REV14) &&*/ PRM_PCS[SYS_DPWM_OFFTEMP].iValue == 1)
                {
                    SYS_InverterStateTransition(SYS_INV_STATE_TRAN_RUN);
                }
#else
                if( GPIO_GetStaticSwitch() )
                {
                    //EVT_Store_NoDup(EVT_SSW_ON);
                    SYS_InverterStateTransition(SYS_INV_STATE_TRAN_RUN);
                }
#endif

                /* 16.06.29 Re-sync -> Run 확인용  */
                if (PRM_PCS[TRC_TRACE_MODE].iValue == TRC_RE_SYNC_TO_GC)
                    TRC_StopTrace();

                break;
            }// switch bypass mode

        }// else if(ODT FINISH)

        break;

#ifdef BYP_EVT_OPERATION
    case SYS_INV_BYP_EVT_OPERATION:

        if (FLT_GetHeavyStatus())
                {
                    SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
                    break;
                }

                else if (PRM_PCS[CTRL_TEST_MODE].iValue != 1) // Normal Mode or Test Mode 2
                {
                    SYS_InverterStateTransition(SYS_INV_STATE_TRAN_TEST_OFF);
                    break;
                }

        PWM_Disable(PWM_CON);

        break;
#endif

    case SYS_INV_TEST_MODE:
        if (FLT_GetHeavyStatus())
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_FAULT);
            break;
        }

        else if (PRM_PCS[CTRL_TEST_MODE].iValue != 1) // Normal Mode or Test Mode 2
        {
            SYS_InverterStateTransition(SYS_INV_STATE_TRAN_TEST_OFF);
            break;
        }

        // MC Less type에서 그리드 차단기 붙이면 쇼트 - TEST 안됨. - TRIP 발생 시킬 경우
        if( PRM_PCS[GRID_GATE_WAY_ENB].iValue == 1 )
        {
            // 그리드 전압이 살아 있으면 TRIP 발생
            if( ACP.PCC.v_pk >= 50 )
            {
                MCB_UpdateCmd(M_CB4_SSW, CB_CMD_TRIP);
            }
        }

        break;
    default:
        INVERTER.uStatus = SYS_INV_STOP;
        PWM_Disable(PWM_CON);
        break;
    }
    INVERTER.uPrevStatus = INVERTER.uStatus;

}


#endif // if 1
