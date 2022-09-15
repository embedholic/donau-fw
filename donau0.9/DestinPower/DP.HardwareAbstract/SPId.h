/*
 * SPId.h
 *
 *  Created on: 2013. 1. 22.
 *      Author: destinPower
 */

#ifndef SPID_H_
#define SPID_H_

#include "LGS_Common.h"

#define DO_CS_A (GpioDataRegs.GPBDAT.bit.GPIO54)
#define DO_CS_B (GpioDataRegs.GPBDAT.bit.GPIO55)


#define DI_CS_A (GpioDataRegs.GPBDAT.bit.GPIO51)
#define DI_CS_B (GpioDataRegs.GPBDAT.bit.GPIO52)
#define DI_CS_C (GpioDataRegs.GPBDAT.bit.GPIO53)

void DO_CS_A_SET();
void DO_CS_A_CLE();
void DO_CS_B_SET();
void DO_CS_B_CLE();
void DI_CS_A_SET();
void DI_CS_A_CLE();
void DI_CS_B_SET();
void DI_CS_B_CLE();
void DI_CS_C_SET();
void DI_CS_C_CLE();

#define DIO_CS_STATE() (!DO_CS_A || !DO_CS_B || \
						!DI_CS_A || !DI_CS_B || !DI_CS_C)

//#define DI_A_DIAG GpioDataRegs.GPBDAT.bit.GPIO41
//#define DI_B_DIAG GpioDataRegs.GPBDAT.bit.GPIO42
//#define DO_A_DIAG GpioDataRegs.GPBDAT.bit.GPIO81
//#define DO_B_DIAG GpioDataRegs.GPBDAT.bit.GPIO82
//#define DO_C_DIAG GpioDataRegs.GPBDAT.bit.GPIO83
//
//#define DIO_DIAG (!DO_A_DIAG || !DO_B_DIAG || \
//				   !DI_A_DIAG || !DI_B_DIAG || !DI_C_DIAG)

public void SPID_init();

#endif /* SPID_H_ */
