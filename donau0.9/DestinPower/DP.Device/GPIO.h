/*
 * GPIO.h
 *
 *  Created on: 2014. 3. 5.
 *      Author: destinPower
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "LGS_Common.h"

typedef enum _SSW_Status { GPIO_STATIC_SW_OFF = 0, GPIO_STATIC_SW_ON = 1 } SSW_Status;

public void GPIO_StaticSwitch(SSW_Status OnOff);
public void GPIO_StaticSwitchOn();
public void GPIO_StaticSwitchOff();
public Bool GPIO_GetStaticSwitchOn();
public SSW_Status GPIO_GetStaticSwitch();

public void GPIO_UpdateStatus();

#endif /* GPIO_H_ */
