/*
 * JCNET_CC.c
 *
 *  Created on: 2022. 6. 2.
 *      Author: isjeon
 */


#include <math.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/System.h>

#include "CC.h"
#include "FAULT.h"
#include "MVP.h"
#include "trace.h"
#include "PEAK.h"
#include "RMS.h"
//-#include "FREQ.h"
#include "RAMP.h"
#include "PHASECHECK.h"
#include "FastRms.h"
#include "pwm.h"
#include "epwm.h"
#include "parameter.h"
#include "prm_pcs.h"
#include "VariablePeriod.h"
#include "EADC.h"
#include "XINTF_ADC.h"
#include "SYSTEM.h"
extern void ThetaDetectRun(ThetaDetect *this, float EdeFlt);
extern SagCompenCtrl SCCTRL; //by JCNET

#pragma CODE_SECTION (InverterInitialize, "dp_ctrl")
void InverterInitialize(void)
{
    PICON_Initialize(&SCCTRL.PICon[0]);
    PICON_Initialize(&SCCTRL.PICon[1]);
    PICON_Initialize(&SCCTRL.PICon[2]);
}
void SagCompen_Param_Init()
{
    volatile float a;
    a = PRM_PCS[CTRL_SCC_D_LIMIT].iValue * (float) PRM_PCS[CTRL_SCC_D_LIMIT].fIncDec;
    SCCTRL.PIArg[0].fMax = a;
    SCCTRL.PIArg[0].fMin = -a;
    a = PRM_PCS[CTRL_SCC_Q_LIMIT].iValue * (float) PRM_PCS[CTRL_SCC_Q_LIMIT].fIncDec;
    SCCTRL.PIArg[1].fMax = a;
    SCCTRL.PIArg[1].fMin = -a;
    a = PRM_PCS[CTRL_SCC_N_LIMIT].iValue * (float) PRM_PCS[CTRL_SCC_N_LIMIT].fIncDec;
    SCCTRL.PIArg[2].fMax = a;
    SCCTRL.PIArg[2].fMin = -a;

    SCCTRL.Prate = 360.0E3;
    SCCTRL.I_lmt = 600.0E3 / (480 * SQRT2 / SQRT3);
#define TRANS_RATIO (277/110)
    SCCTRL.Vdc_max = 480 * SQRT2 / SQRT3 * (TRANS_RATIO);
}
void SagCompen_Run()
{
    volatile float a;
    volatile iir1st *pIIR1;
    PICon *pPICon;
    PIArg *pPIArg;

    EADC_GetAnalogValue(EADC.pAdcBypEa, &SCCTRL.Ea);
    EADC_GetAnalogValue(EADC.pAdcBypEb, &SCCTRL.Eb);
    EADC_GetAnalogValue(EADC.pAdcBypEc, &SCCTRL.Ec);

    EADC_GetAnalogValue(EADC.pAdcCapIa, &SCCTRL.Ia);
    EADC_GetAnalogValue(EADC.pAdcCapIb, &SCCTRL.Ib);
    EADC_GetAnalogValue(EADC.pAdcCapIc, &SCCTRL.Ic);

    TRANSFORM_abc_dqn(SCCTRL.Ea, SCCTRL.Eb, SCCTRL.Ec, SCCTRL.Eds, SCCTRL.Eqs, SCCTRL.Ens)

    ThetaDetectRun(&SCCTHETA, SCCTRL.fltEde.fOut); // R : 8us!

    SCCTRL.SinTheta = sin(SCCTHETA.fRadian);
    SCCTRL.CosTheta = cos(SCCTHETA.fRadian);

     // 14.4.7 -------------------------------------------------------------------
    SCCTRL.RefTheta = SCCTHETA.fRadian; /* - PI_6; */
    if (SCCTRL.RefTheta > PI)
        SCCTRL.RefTheta -= TWO_PI;
    else if (SCCTRL.RefTheta < -PI)
        SCCTRL.RefTheta += TWO_PI;

    SCCTRL.RefSinTheta = sin(SCCTRL.RefTheta);
    SCCTRL.RefCosTheta = cos(SCCTRL.RefTheta);
     // ---------------------------------------------------------------------------

    TRANSFORM_ROTATE_THETA_n(SCCTRL.Eds, SCCTRL.Eqs, SCCTRL.Ens, SCCTRL.CosTheta,
             SCCTRL.SinTheta, SCCTRL.Ede, SCCTRL.Eqe, SCCTRL.Ene);

    IIR1_Filter2( &SCCTRL.fltEde, SCCTRL.Ede);
    IIR1_Filter2( &SCCTRL.fltEqe, SCCTRL.Eqe);
    IIR1_Filter2( &SCCTRL.fltEqe2nd, SCCTRL.fltEqe.fOut); // 1Hz
    IIR1_Filter2( &SCCTRL.fltEne, SCCTRL.Ene);
    IIR1_Filter2( &SCCTRL.fltFreq, SCCTHETA.fFreq);


//by JCNET    a = ??
//D-axis

//    SCCTRL.fRef = RAMP_Change(SCCTRL.pRAMPInput, IVC.fRef_command);
    SCCTRL.PIArg[0].fErr = 0  - SCCTRL.Ede;
    SCCTRL.PIArg[0].fFbk = SCCTRL.Ede;
    SCCTRL.PIArg[0].fCompen = 0;
    PICON_RunM(&SCCTRL.PICon[0], &SCCTRL.PIArg[0], SCCTRL.Vd_comp0); //SCCTRL.fPIOut[0]);

//Q-axis

//    SCCTRL.fRef = RAMP_Change(SCCTRL.pRAMPInput, IVC.fRef_command);
    SCCTRL.PIArg[1].fErr = SCCTRL.Vqe_ref  - SCCTRL.Eqe;
    SCCTRL.PIArg[1].fFbk = SCCTRL.Eqe;
    SCCTRL.PIArg[1].fCompen = 0;
    PICON_RunM(&SCCTRL.PICon[1], &SCCTRL.PIArg[1], SCCTRL.Vq_comp0); // SCCTRL.fPIOut[1]);
//중성선

//    SCCTRL.fRef = RAMP_Change(SCCTRL.pRAMPInput, IVC.fRef_command);
    SCCTRL.PIArg[2].fErr = 0  - SCCTRL.Ene;
    SCCTRL.PIArg[2].fFbk = SCCTRL.Ene;
    SCCTRL.PIArg[2].fCompen = 0;
    PICON_RunM(&SCCTRL.PICon[2], &SCCTRL.PIArg[2], SCCTRL.Vn_comp0); //SCCTRL.fPIOut[2]);

// 입력 전력 제한 보상
    {
        float sum;
        sum = (SCCTRL.Ede * SCCTRL.I_lmt + SCCTRL.Eqe * SCCTRL.I_lmt + SCCTRL.Ene * SCCTRL.I_lmt);
        sum *= 1.0/SCCTRL.Prate;
        if(sum >= 1.0) SCCTRL.factor_prate  = 1.0;
        else if(sum < 0.0) sum = 0;
        SCCTRL.factor_prate = sum;
    }
    SCCTRL.Vd_comp1 = SCCTRL.Vd_comp0 * SCCTRL.factor_prate;
    SCCTRL.Vq_comp1 = SCCTRL.Vq_comp0 * SCCTRL.factor_prate;
    SCCTRL.Vn_comp1 = SCCTRL.Vn_comp0 * SCCTRL.factor_prate;
// 출력 제한 보상
    {
        float sum;
        sum = SCCTRL.Vd_comp1 * SCCTRL.Vd_comp1 + SCCTRL.Vq_comp1 * SCCTRL.Vq_comp1 + SCCTRL.Vn_comp1 * SCCTRL.Vn_comp1;
        sum = sqrt(sum) / SCCTRL.Vdc_max;
        if(sum > 1.0) sum = 1.0;
        else if(sum < 0.0) sum = 0.0;
        SCCTRL.factor_vdc = sum;
    }
    SCCTRL.Vd_comp2 = SCCTRL.Vd_comp1 * SCCTRL.factor_vdc;
    SCCTRL.Vq_comp2 = SCCTRL.Vq_comp1 * SCCTRL.factor_vdc;
    SCCTRL.Vn_comp2 = SCCTRL.Vn_comp1 * SCCTRL.factor_vdc;
#if 0
    SCCTRL.Vd_comp0 = SCCTRL.fPIOut[0];
    SCCTRL.Vq_comp0 = SCCTRL.fPIOut[1];
    SCCTRL.Vn_comp0 = SCCTRL.fPIOut[2];
#endif
    TRANSFORM_ROTATE_THETA_INVERSE_n(SCCTRL.Vd_comp2,SCCTRL.Vq_comp2, SCCTRL.Vn_comp2,  SCCTRL.CosTheta,  SCCTRL.SinTheta,
                                     SCCTRL.Vds_comp2, SCCTRL.Vqs_comp2, SCCTRL.Vns_comp2);
    TRANSFORM_dqn_abc(SCCTRL.Vds_comp2,SCCTRL.Vqs_comp2, SCCTRL.Vns_comp2, SCCTRL.VaRef, SCCTRL.VbRef, SCCTRL.VcRef);
}

void CTRL_SCC_PWM_Generate(void)
{

    extern float ScaleGating; // CC.c에 있는 변수,  PWM_CLK * 1.0e6 * 0.5 (상승,하강이 한주기이므로 0.5)
    volatile float a, b;                //- register float a,b;
    volatile float Ta, Tb, Tc, Tzero;   //Illegal operand combination point : if Ta~Tc == volatile is OK else is NOT OK!
    volatile float DCLinkV2;
    volatile Uns sel;

// by JCNET
// 아래 ACP.INV.tsw_div_vdc를 그대로 사용한다면.. DCLinkV2는 Converter측 PWM 계산시 update되어 이씅ㅁ
//
// Converter에서 반영하고 있음..    DCLinkV2 = BATCTRL.fltDCLinkV.fOut;

    /*------------------*/
    /* Space Vector PWM */
    /*------------------*/
// ACP.INV.tsw_div_vdc 를 곱하는 곳을 어떻게 처리할지??
//
//    ACP.INV.tsw_div_vdc = CC_tsCC / DCLinkV2;
    {
        Ta = SCCTRL.VaRef * ACP.INV.tsw_div_vdc;
        Tb = SCCTRL.VbRef * ACP.INV.tsw_div_vdc;
        Tc = SCCTRL.VcRef * ACP.INV.tsw_div_vdc;

        /* 3-element Sort Algorithm a=Tmax, b=Tmin */
        if (Ta > Tb)
        {
            a = Ta;
            b = Tb;
        }
        else
        {
            a = Tb;
            b = Ta;
        }
        if (Tc > a)
            a = Tc;
        if (Tc < b)
            b = Tc;

        /* Effective Time Calculation */
        Tzero = CC_tsCC - (a - b);
        if (Tzero < 2. * CC_DEAD_TIME)
        { /* Over Modulation */
#if USE_FASTRTS_LIB_DIV
            a = div((CC_tsCC - 2.* CC_DEAD_TIME) , (a-b));
#else
            a = (CC_tsCC - 2. * CC_DEAD_TIME) / (a - b);
#endif
            Ta *= a;
            Tb *= a;
            Tc *= a;
            b *= a;
            Tzero = 2. * CC_DEAD_TIME;
#if 0 //by JCNET
            INVCTRL.VdsRef *= a;
            INVCTRL.VqsRef *= a;
            INVCTRL.VdeRef *= a;
            INVCTRL.VqeRef *= a;
            INVCTRL.VdeIntegOut *= a;
            INVCTRL.VqeIntegOut *= a;
#endif
        }
        a = Tzero * 0.5 - b;
        Ta += a;
        Tb += a;
        Tc += a;
    }



    SCCTRL.PwmOnTime.uPhA = (Ta * ScaleGating + 0.5);
    SCCTRL.PwmOnTime.uPhB = (Tb * ScaleGating + 0.5);
    SCCTRL.PwmOnTime.uPhC = (Tc * ScaleGating + 0.5);

}
