/*
 * Comm_CommonData.h
 *
 *  Created on: 2013. 1.
 *      Author:
 */

#ifndef COMMDATA_H_
#define COMMDATA_H_

#include <stdint.h> //for intN_t. but 28346 does not support uint8_t

//#define SLAVE_BMS_NUM		64
#define SLAVE_BMS_NUM		4

#define MAKE_16BIT(a, b)	( ( ((a)&(0xFF))<<8 ) | ( ((b)&(0xFF)) ) )

enum{
	COMM_C0,
	COMM_C1,
	COMM_I0,
	COMM_MB0,
	COMM_MB1,
	COMM_MB2,
	COMM_MB3,
	COMM_MB10,

	COMM_MAX
};


#endif /* COMMDATA_H_ */
