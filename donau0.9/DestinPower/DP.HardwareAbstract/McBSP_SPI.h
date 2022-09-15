/*
 * McBSP_SPI.h
 *
 *  Created on: 2013. 1. 11.
 *      Author: destinPower
 */

#ifndef MCBSP_SPI_H_
#define MCBSP_SPI_H_

#include "LGS_Common.h"

#define NVSRAM_ADDR_STX 0
#define NVSRAM_ADDR_END 32000 // original 32767.

public void MCBSP_SPI_init(void);
public void MCBSP_SPI_xmit(int a);
public void MCBSP_SPI_read(int* a);
#if McBSP_SPI_GPIO_MODE == 1
public void MCBSP_SPI_byteRead(UInt16 address,UInt16* data);
public void MCBSP_SPI_byteWrite(UInt16 address,UInt16 data);
#else
public Bool MCBSP_SPI_byteRead(UInt16 address,UInt16* data);
public Bool MCBSP_SPI_byteWrite(UInt16 address,UInt16 data);
#endif
public void MCBSP_SPI_error(void);

#endif /* MCBSP_SPI_H_ */
