/*
 * Odt.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef ODT_H_
#define ODT_H_

//-#include <std.h>
//+
#include "LGS_Common.h"

#define ODT_FINISH	 1 /* On Delay 완료 */
#define ODT_BROKEN	 0 /* On Delay 실패 */
#define ODT_ING		 2 /* On Delay 진행중 */

/**********************************************************************/
/* Structure Definition for ODT                                       */
/**********************************************************************/
// uHoldTime과 uPeriod는 msec단위임.
typedef struct _Odt
{
	//TODO Check Here. Uns를 Uint32로 변경함.
	Uint32 uHoldTime;
	Uns uPeriod;
	Uns uCount;
	Bool bLiveFlag;
}Odt;

Odt *Odt_(Odt *this, Uint32 uHoldTime_msec, Uns uPeriod_msec);
Uns ODT_Update(Odt *this, Bool TrueFalseStatement);
void ODT_Initialize(Odt *this);
void ODT_HoldTimeChange(Odt *this, Uns uHoldTime_msec);

#endif /* ODT_H_ */
