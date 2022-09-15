/*
 * ExADC.h
 *
 *  Created on: 2012. 12. 17.
 *      Author: destinPower
 */

#ifndef EXADC_H_
#define EXADC_H_

#include "LGS_Common.h"
#if 0 // old
typedef enum
{
	/*------- AD7606 (1) -------------*/
	/* CN A1 */
	INV_V1,
	INV_V2,
	INV_V3,
	/* CN A2 */
	OUTPUT_V1,
	OUTPUT_V2,
	OUTPUT_V3,
	/* CN A3 */
	DCLINK_V,
	/* CN A4 */
	BAT_V
}ADC_A_ITEMS;

typedef enum
{
	/*------- AD7606 (2) -------------*/
	/* CN A4 */
	INPUT_V1,
	INPUT_V2,
	INPUT_V3,
	/* CN A6 ASI */
	OUT_I1,
	OUT_I2,
	OUT_I3,

	BAT_I,
	REV_1
}ADC_B_ITEMS;

typedef enum
{
	/*------- AD7656 -----------------*/
	REV_2,
	REV_3,
	REV_4,

	INV_I1,
	INV_I2,
	INV_I3
}ADC_C_ITEMS;
#endif


public void XINTF_ADC_Create();
public void XINTF_ADC_READ_ALL();
public Uint16 XINTF_ADC_getVLevel(Uint16 adcCh);/* ADC에 입력 되는 실제 전압을 리턴 한다. */
public void XINTF_ADC_MUX_TOGGLE();
public Uint16 XINTF_ADC_MUX_STATE();

extern volatile int ADC_A_buf[8];
extern volatile int ADC_B_buf[8];
extern volatile int ADC_C_buf[6];
extern float fADC_A_res[8];
extern float fADC_B_res[8];
extern float fADC_C_res[6];

#endif /* EXADC_H_ */
