/*
 * CTRL_INV_PWM_GENERATE.c
 *
 *  Created on: 2014. 4. 20.
 *      Author: Seth Oh
 */
#include "LGS_Common.h"
#include "MathConst.h"
#include "CTRL_INV_PWM_GENERATE.h"
#include "CC.h"
#include "FAULT.h"
#include "CTRL_MODE.h"

extern	AC_Panel	ACP;
extern	Controller	CTRL;
extern float ScaleGating;

void CTRL_INV_PWM_Generate(void)
{

	volatile float a, b; 				//-	register float a,b;
	volatile float Ta, Tb, Tc, Tzero; 	//Illegal operand combination point : if Ta~Tc == volatile is OK else is NOT OK!
	volatile float DCLinkV2;
	volatile Uns sel;

	DCLinkV2 = BATCTRL.fltDCLinkV.fOut;

	if ((INVERTER.uStatus == SYS_INV_AC_GENERATE) || !CTRL_MODE_DPWM_ON_POSSIBLE())
		sel = 0;
	else
		sel = OPTION.uPWM;


	/*------------------*/
	/* Space Vector PWM */
	/*------------------*/
	switch (sel)
	{
	case 0: // Symetric SVPWM
	case 96: // Symetric SVPWM
		Ta = INVCTRL.VaRef * ACP.INV.tsw_div_vdc;
		Tb = INVCTRL.VbRef * ACP.INV.tsw_div_vdc;
		Tc = INVCTRL.VcRef * ACP.INV.tsw_div_vdc;

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
			INVCTRL.VdsRef *= a;
			INVCTRL.VqsRef *= a;
			INVCTRL.VdeRef *= a;
			INVCTRL.VqeRef *= a;
			INVCTRL.VdeIntegOut *= a;
			INVCTRL.VqeIntegOut *= a;
		}
		a = Tzero * 0.5 - b;
		Ta += a;
		Tb += a;
		Tc += a;
		break;
	case 32: // 60도 불연속 전압 변조
		/* 3-element Sort Algorithm a=Vmax, b=Vmin */
		Ta = INVCTRL.VaRef;
		Tb = INVCTRL.VbRef;
		Tc = INVCTRL.VcRef;
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

		if (a + b >= 0.)
		{
			INVCTRL.VsnRef = 0.5 * DCLinkV2 - a;
			if (INVCTRL.VsnRef < -(0.5 * DCLinkV2)) //Over Modulation
			{
				INVCTRL.VsnRef = -(a + b) * 0.5;
				FLT_Raise(FLTL_OVER_MDL);
			}
		}
		else
		{
			INVCTRL.VsnRef = -0.5 * DCLinkV2 - b;
			if (INVCTRL.VsnRef > 0.5 * DCLinkV2) //Over Modulation
			{
				INVCTRL.VsnRef = -(a + b) * 0.5;
				FLT_Raise(FLTL_OVER_MDL);
			}
		}

		INVCTRL.VanRef = INVCTRL.VaRef + INVCTRL.VsnRef;
		INVCTRL.VbnRef = INVCTRL.VbRef + INVCTRL.VsnRef;
		INVCTRL.VcnRef = INVCTRL.VcRef + INVCTRL.VsnRef;

		// Illegal operand combination point : if Ta~Tc == violate is OK else is NOT OK!
		Ta = INVCTRL.VanRef * ACP.INV.tsw_div_vdc;
		Tb = INVCTRL.VbnRef * ACP.INV.tsw_div_vdc;
		Tc = INVCTRL.VcnRef * ACP.INV.tsw_div_vdc;

		a = CC_tsCC * 0.5;
		Ta += a;
		Tb += a;
		Tc += a;

		/*  Limit */
		b = 2. * CC_DEAD_TIME;
		a = CC_tsCC;
		if (Ta > a)
			Ta = a;
		if (Ta < b)
			Ta = 0;
		if (Tb > a)
			Tb = a;
		if (Tb < b)
			Tb = 0;
		if (Tc > a)
			Tc = a;
		if (Tc < b)
			Tc = 0;

		break;
	case 64: // 120도(ON) 불연속 전압 변조
		/* 3-element Sort Algorithm a=Vmax, b=Vmin */
		Ta = INVCTRL.VaRef;
		Tb = INVCTRL.VbRef;
		Tc = INVCTRL.VcRef;
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
#if 1
		INVCTRL.VsnRef = 0.5 * DCLinkV2 - a;
		if (INVCTRL.VsnRef < -(0.5 * DCLinkV2)) //Over modulation
		{
			FLT_Raise(FLTL_OVER_MDL);
			INVCTRL.VsnRef = -(a + b) * 0.5;
		}
#else
		//-INVCTRL.VsnRef = -(a + b) * 0.5;
		INVCTRL.VsnRef = 0.5 * DCLinkV2 - a;
#endif
		INVCTRL.VanRef = INVCTRL.VaRef + INVCTRL.VsnRef;
		INVCTRL.VbnRef = INVCTRL.VbRef + INVCTRL.VsnRef;
		INVCTRL.VcnRef = INVCTRL.VcRef + INVCTRL.VsnRef;

		// Illegal operand combination point : if Ta~Tc == violate is OK else is NOT OK!
		Ta = INVCTRL.VanRef * ACP.INV.tsw_div_vdc;
		Tb = INVCTRL.VbnRef * ACP.INV.tsw_div_vdc;
		Tc = INVCTRL.VcnRef * ACP.INV.tsw_div_vdc;

		a = CC_tsCC * 0.5;
		Ta += a;
		Tb += a;
		Tc += a;

		/*  Limit */
		b = 2. * CC_DEAD_TIME;
		a = CC_tsCC;
		if (Ta > a)
			Ta = a;
		if (Ta < b)
			Ta = 0;
		if (Tb > a)
			Tb = a;
		if (Tb < b)
			Tb = 0;
		if (Tc > a)
			Tc = a;
		if (Tc < b)
			Tc = 0;

		break;
	default:
		Ta = INVCTRL.VaRef * ACP.INV.tsw_div_vdc;
		Tb = INVCTRL.VbRef * ACP.INV.tsw_div_vdc;
		Tc = INVCTRL.VcRef * ACP.INV.tsw_div_vdc;

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
			FLT_Raise(FLTL_OVER_MDL);
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
			INVCTRL.VdsRef *= a;
			INVCTRL.VqsRef *= a;
			INVCTRL.VdeRef *= a;
			INVCTRL.VqeRef *= a;
			INVCTRL.VdeIntegOut *= a;
			INVCTRL.VqeIntegOut *= a;
		}
		a = Tzero * 0.5 - b;
		Ta += a;
		Tb += a;
		Tc += a;
		break;
	}

	INVCTRL.PwmOnTime.uPhA = (Ta * ScaleGating + 0.5);
	INVCTRL.PwmOnTime.uPhB = (Tb * ScaleGating + 0.5);
	INVCTRL.PwmOnTime.uPhC = (Tc * ScaleGating + 0.5);

}
