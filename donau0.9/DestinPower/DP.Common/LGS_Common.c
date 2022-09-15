/*
 * LGS_Common.c
 *
 *  Created on: 2012. 11. 19.
 *      Author: destinPower
 */

#include "LGS_Common.h"

/*
 * 소프트웨어 동작 준비 완료! 플래그.
 */
volatile Bool bSystemStarted;
Bool bUpdateFlag;

// sqrt = 202.04
// sqrt7 = 207.7 ( 정확도: 97.2% )
float sqrt7(float x)
{
	//return sqrt(x);

	UInt32 i = *(UInt32*) &x;
	// adjust bias
	i  += (UInt32)127 << (UInt32)23;
	// approximation of square root
	i >>= 1;
	return *(float*) &i;
}

