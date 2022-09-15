/*
 * Filter.c
 *
 *  Created on: 2012. 11. 8.
 *      Author: destinPower
 */

#include "Filter.h"
#include "MathConst.h"

/*
*********************************************************************************************************
*                                        1st Order IIR Filter Creation
*
* Description:
*
* Arguments  : iir1st * 필터객체
* 			 : float Sampling Time
* 			 : float Low Pass Filter Cut-Off Freq
*
* Returns    : none
*********************************************************************************************************
*/
//void IIR1_Creation( iir1st *this, float SampleTime, float CutOffFreq )
//{
//	float a;
//
//	a = 2 * PI * CutOffFreq;
//#if USE_FASTRTS_LIB_DIV
//	this->fLa = div((2.- a*SampleTime),(2. + a*SampleTime));
//	this->fLb = div((a*SampleTime),(2. + a*SampleTime));
//#else
//	this->fLa = (2.- a*SampleTime)/(2. + a*SampleTime);
//	this->fLb = (a*SampleTime)/(2. + a*SampleTime);
//#endif
//	this->fPrevIn = 0;
//	this->fOut = 0;
//}

void IIR1_Creation_IS( iir1st *this, float SampleTime, float CutOffFreq, float SampleTime_is_mode )
{
	float a;

	a = 2 * PI * CutOffFreq;
	this->fLa = (2.- a*SampleTime)/(2. + a*SampleTime);
	this->fLb = (a*SampleTime)/(2. + a*SampleTime);
	this->fLa_is = (2.- a*SampleTime_is_mode)/(2. + a*SampleTime_is_mode);
	this->fLb_is = (a*SampleTime_is_mode)/(2. + a*SampleTime_is_mode);

	this->fPrevIn = 0;
	this->fOut = 0;
}
#if 0
/*
*********************************************************************************************************
*                                         필터 처리 함수
*
* Description: 1차 IIR Filter를 In에 적용하여 filtering함
*
* Arguments  : iir1st * Filter 의 포인터
* 			 : float -> Filtering 될 입력값
*
* Returns    : float -> Filtering된 출력값
*********************************************************************************************************
*/
float IIR1_Filtering( iir1st *this, float In )
{
    this->fOut = ( this->fLa * this->fOut ) +
    				( this->fLb * (In + this->fPrevIn) );
    this->fPrevIn = In;
	return this->fOut;
}
#endif
/*
*********************************************************************************************************
*                                 IIR1 필터 객체 초기화
*
* Description: 1차 IIR Filter의 출력값을 0으로 만듬
*
* Arguments  : iir1st * Filter 의 포인터
*
* Returns    : none
*********************************************************************************************************
*/
void IIR1_Initialize( iir1st *this )
{
    this->fOut = 0.;
    this->fPrevIn = 0.;
}

/*
*********************************************************************************************************
*                                 IIR1 필터 파라미터 변경
*
* Description: 1st Order IIR Filter 계수 변경
*
* Arguments  : iir1st * Filter 의 포인터
* 			 : float Sampling Time
* 			 : float Low Pass Filter Cut-Off Freq.
*
* Returns    : none
*********************************************************************************************************
*/
#if 0
//-TODO DATA REGION: #pragma CODE_SECTION(IIR1_ChangeCoefficient, ".iram")
void IIR1_ChangeCoefficient( iir1st *this )
{
	this->fLa = this->fLaShadow;
	this->fLb = this->fLbShadow;
}

void IIR1_CalcShadow( iir1st *this, float SampleTime, float CutOffFreq )
{
	float a;

	a = 2 * PI * CutOffFreq;
#ifdef USE_FASTRTS_LIB_DIV
	this->fLaShadow = div((2.- a*SampleTime),(2. + a*SampleTime));
	this->fLbShadow = div((a*SampleTime),(2. + a*SampleTime));
#else
	this->fLaShadow = (2.- a*SampleTime)/(2. + a*SampleTime);
	this->fLbShadow = (a*SampleTime)/(2. + a*SampleTime);
#endif
}
#endif

void IIR1_UpdateCoeff( iir1st *this, float SampleTime, float CutOffFreq )
{
	float a;

	a = 2 * PI * CutOffFreq;
#if USE_FASTRTS_LIB_DIV
	this->fLa = div((2.- a*SampleTime),(2. + a*SampleTime));
	this->fLb = div((a*SampleTime),(2. + a*SampleTime));
#else
	this->fLa = (2.- a*SampleTime)/(2. + a*SampleTime);
	this->fLb = (a*SampleTime)/(2. + a*SampleTime);
#endif
}

/*
*********************************************************************************************************
*                                 All Pass Shift x Degree Filter Creation
*
* Description:
*
* Arguments  : AllPassShiftFilter * 필터객체
* 			 : float Sampling Time
* 			 : Uns DelayDegree
* 			 : float Freq ==> 지연 전압의 주파수
*
* Returns    : none
*********************************************************************************************************
*/
#if 0
void APS_Creation( AllPassShiftFilter *this, float SampleTime, Uns DelayDegree, float Freq )
{
	float ze, Wna, Kden, a;

	switch ( DelayDegree )
	{
		case APS_DEGREE_90:
			ze = 4.95;
			Wna = 2. * PI * Freq * 10.;
			break;
		case APS_DEGREE_30:
			ze = 3.95;
			Wna = 2. * PI * Freq * 30.;
			break;
		default:
			ze = 4.95;
			Wna = 2. * PI * Freq * 10.;
			break;
	}

	a = SampleTime * Wna;
	Kden = 4. + (4. * ze * a) + (a * a);
#if USE_FASTRTS_LIB_DIV
	this->Kxn = div(( 4. - (4. * ze * a) + (a * a) ) , Kden);
	this->Kxn_1 = div(( -8. + 2. * a * a ) , Kden);
#else
	this->Kxn = ( 4. - (4. * ze * a) + (a * a) ) / Kden;
	this->Kxn_1 = ( -8. + 2. * a * a ) / Kden;
#endif
	this->Kxn_2 = 1;
	this->Kyn_1 = this->Kxn_1;
	this->Kyn_2 = this->Kxn;

	this->Xn_1 = 0;
	this->Xn_2 = 0;
	this->Yn_1 = 0;
	this->Yn_2 = 0;
}
#endif

void APS_Creation_IS( AllPassShiftFilter *this, float SampleTime, Uns DelayDegree, float Freq, float SampleTime_is_mode )
{
	float ze, Wna, Kden, a;

	switch ( DelayDegree )
	{
		case APS_DEGREE_90:
			ze = 4.95;
			Wna = 2. * PI * Freq * 10.;
			break;
		case APS_DEGREE_30:
			ze = 3.95;
			Wna = 2. * PI * Freq * 30.;
			break;
		default:
			ze = 4.95;
			Wna = 2. * PI * Freq * 10.;
			break;
	}

	a = SampleTime * Wna;
	Kden = 4. + (4. * ze * a) + (a * a);
	this->Kxn = ( 4. - (4. * ze * a) + (a * a) ) / Kden;
	this->Kxn_1 = ( -8. + 2. * a * a ) / Kden;
	this->Kxn_2 = 1;
	this->Kyn_1 = this->Kxn_1;
	this->Kyn_2 = this->Kxn;

	a = SampleTime_is_mode * Wna;
	Kden = 4. + (4. * ze * a) + (a * a);
	this->Kxn_is = ( 4. - (4. * ze * a) + (a * a) ) / Kden;
	this->Kxn_1_is = ( -8. + 2. * a * a ) / Kden;
	this->Kxn_2_is = 1;
	this->Kyn_1_is = this->Kxn_1;
	this->Kyn_2_is = this->Kxn;

	this->Xn_1 = 0;
	this->Xn_2 = 0;
	this->Yn_1 = 0;
	this->Yn_2 = 0;
}

/*
*********************************************************************************************************
*                                   All Pass Shift Filter
*
* Description:
*
* Arguments  : AllPassShiftFilter * 필터객체
* 			 : float -> Filtering 될 입력값
*
* Returns    : float -> Filtering된 출력값
*********************************************************************************************************
*/
#if 0
//-TODO DATA REGION: #pragma CODE_SECTION(APS_Filtering, ".iram")
float APS_Filtering( AllPassShiftFilter *this, float Xn )
{
	float Yn;

    Yn = ( this->Kxn * Xn ) + ( this->Kxn_1 * this->Xn_1 ) + ( this->Kxn_2 * this->Xn_2 )
    			- ( this->Kyn_1 * this->Yn_1 ) - ( this->Kyn_2 * this->Yn_2 );
    this->Xn_2 = this->Xn_1;
    this->Xn_1 = Xn;
    this->Yn_2 = this->Yn_1;
    this->Yn_1 = Yn;

	return Yn;
}
#endif

float APS_Filtering_IS( AllPassShiftFilter *this, float Xn )
{
	float Yn;

    Yn = ( this->Kxn_is * Xn ) + ( this->Kxn_1_is * this->Xn_1 ) + ( this->Kxn_2_is * this->Xn_2 )
    			- ( this->Kyn_1_is * this->Yn_1 ) - ( this->Kyn_2_is * this->Yn_2 );
    this->Xn_2 = this->Xn_1;
    this->Xn_1 = Xn;
    this->Yn_2 = this->Yn_1;
    this->Yn_1 = Yn;

	return Yn;
}

float APS_Filtering_GC( AllPassShiftFilter *this, float Xn )
{
	float Yn;

    Yn = ( this->Kxn * Xn ) + ( this->Kxn_1 * this->Xn_1 ) + ( this->Kxn_2 * this->Xn_2 )
    			- ( this->Kyn_1 * this->Yn_1 ) - ( this->Kyn_2 * this->Yn_2 );
    this->Xn_2 = this->Xn_1;
    this->Xn_1 = Xn;
    this->Yn_2 = this->Yn_1;
    this->Yn_1 = Yn;

	return Yn;
}

#if 0
/*
*********************************************************************************************************
*                                   Band Pass Filter Creation
*
* Description:
*
* Arguments  : BandPassFilter * 필터객체
* 			 : float Sampling Time
* 			 : float Freq ==> 검출을 원하는 차수의 주파수
*
* Returns    : none
*********************************************************************************************************
*/
void BPF_Creation( BandPassFilter *this, float SampleTime, float Freq )
{
	float ze, Wna, Kden, a;

//	switch ( Freq )
//	{
//		case 300:
//			ze = 0.707;
//			Wna = 2. * PI * Freq;
//			break;
//		case 420:
//			ze = 0.707;
//			Wna = 2. * PI * Freq;
//			break;
//		default:
//			ze = 0.707;
//			Wna = 2. * PI * Freq;
//			break;
//	}

			ze = 0.005;
			Wna = 2. * PI * Freq;
//			Wna = 2. * PI * 300;

	a = SampleTime * Wna;
	Kden = 4. + (4. * ze * a) + (a * a);

	this->Kxn = (4. * ze * a) / Kden;
	this->Kxn_1 = 0.;
	this->Kxn_2 = -this->Kxn;
	this->Kyn_1 = (-2 * a * a + 8) / Kden;
	this->Kyn_2 = (-4 + 4 * ze * a - (a * a)) / Kden;

	this->Xn_1 = 0;
	this->Xn_2 = 0;
	this->Yn_1 = 0;
	this->Yn_2 = 0;
}

/*
*********************************************************************************************************
*                                         Band Pass Filter
*
* Description:
*
* Arguments  : BandPassFilter * 필터객체
* 			 : float -> Filtering 될 입력값
*
* Returns    : float -> Filtering된 출력값
*********************************************************************************************************
*/
//-TODO DATA REGION: #pragma CODE_SECTION(BPF_Filtering, ".iram")
float BPF_Filtering( BandPassFilter *this, float Xn )
{
	float Yn;

    Yn = ( this->Kxn * Xn ) + ( this->Kxn_1 * this->Xn_1 ) + ( this->Kxn_2 * this->Xn_2 )
    			+ ( this->Kyn_1 * this->Yn_1 ) + ( this->Kyn_2 * this->Yn_2 );
    this->Xn_2 = this->Xn_1;
    this->Xn_1 = Xn;
    this->Yn_2 = this->Yn_1;
    this->Yn_1 = Yn;

	return Yn;
}


void HPF_Creation( HighPassFilter *this, float SampleTime, float Freq )
{
	float Ha, Hb, a;


	a = 2. * PI * Freq;
	//TODO 확인
#if USE_FASTRTS_LIB_DIV
	Ha = div(2.-a*SampleTime, 2.+a*SampleTime);
	Hb = div(2., 2.+a*SampleTime);
#else
	Ha = (2.-a*SampleTime/ 2.+a*SampleTime);
	Hb = (2./ 2.+a*SampleTime);
#endif
	this->Kxn = Hb; // Coeffient of Xn( Input )
	this->Kxn_1 = -Hb; // Coeffient of Xn_1( Previous Input )
	this->Kyn_1 = Ha; // Coeffient of Yn_1( Previous Output )
	this->Xn_1 = 0; // Previous Input
	this->Yn_1 = 0; // Previous Output
}

//-TODO DATA REGION: #pragma CODE_SECTION(HPF_Filtering, ".iram")
float HPF_Filtering( HighPassFilter *this, float Xn )
{
	float Yn;
	Yn = this->Kyn_1*this->Yn_1 + this->Kxn*Xn + this->Kxn_1*this->Xn_1;

	this->Yn_1 = Yn;
	this->Xn_1 = Xn;

	return Yn;
}
#endif

