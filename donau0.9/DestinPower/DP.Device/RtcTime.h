/*
 * RtcTime.h
 *
 *  Created on: 2012. 11. 12.
 *      Author: destinPower
 */

#ifndef RTCTIME_H_
#define RTCTIME_H_

#include "LGS_Common.h"

public void RTCTIME_Create(void);
public void RTCTIME_Update(void);
public Bool RTCTIME_Set(RtcTime *pSetVal);
public TimePack RTCTIME_GetPackValue( void );

#endif /* RTCTIME_H_ */
