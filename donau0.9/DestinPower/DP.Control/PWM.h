/*
 * PWM.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef PWM_H_
#define PWM_H_

#include "LGS_Common.h"

typedef enum
{
	PWM_CON,
	PWM_PEBTEMP,
	PWM_SCC //by JCNET
} PwmID;

/**************************************************************************/
/* Structure definition for PwmOnTime                                     */
/**************************************************************************/
typedef struct _PwmOnTime
{
	float		uPhA;
	float		uPhB;
	float		uPhC;
	float		uPhF;
	Uns		uPeriod;
	Bool	bEnb;
} PwmOnTime;

/**************************************************************************/
/* PWM Methods                                                            */
/**************************************************************************/
void PWM_Create( void ); /* PWM 초기화 */
void PWM_SetOnTime(PwmID ID, PwmOnTime *pOnTime);
void PWM_Disable(PwmID ID);
void PWM_SetPeriod(PwmID ID, PwmOnTime *pOnTime);

extern Uns CC_uPeriod;

//-150529 #define PWM_CC_PERIOD_MIN  7500  // 10kHz. 100us * 150Mhz / 2
//- 뻗음. #define PWM_CC_PERIOD_MIN  3000  // 3000 25kHz.  40us * 150Mhz / 2
#define PWM_CC_PERIOD_MIN  6000  // 3000  12.5kHz.  80us * 150Mhz / 2
#define PWM_CC_PERIOD_MAX  24975 // 18750 4kHz. 250us * 150Mhz / 2(Up-Doun count 이므로)
								 // 24975 3kHz

extern //Pwm PWM_ADDR;//+
PwmOnTime		Con;
extern PwmOnTime       Inv_Pwm_Info; //by JCNET
#endif /* PWM_H_ */
