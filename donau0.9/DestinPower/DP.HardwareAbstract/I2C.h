/*
 * I2C.h
 *
 *  Created on: 2012. 11. 8.
 *      Author: destinPower
 */

#ifndef I2C_H_
#define I2C_H_

#include "LGS_Common.h"

void I2C_Init(void);
Bool I2C_SET_RTC_Time(RtcTime* pTime);
Bool I2C_GET_RTC_Time(RtcTime* pTime);
Bool I2C_GET_Temp(UInt16* pRetData);

interrupt void i2c_int1a_isr(void);     // I2C-A

#endif /* I2C_H_ */
