/*======================================================================
	File name	:	DSP2834x_M25P40.h

	Originator	:	Digital Control Systems Group
					SyncWorks

	Target		:	TMS320C28346

	Version		:	1.10
======================================================================*/

/*======================================================================
	History		:
		2010-03-22,		Version 1.00
		2012	:	June
					*Fixed bugs when HEX file size is big
					*Added Update sequence after download to Sector 4~7
======================================================================*/

#ifndef DSP2834X_M25P40_H_
#define DSP2834X_M25P40_H_


#ifdef DSP2834x_M25P40_GLOBAL

#define DSP2834x_M25P40_EXT

#include "DSP2834x_Device.h"
#include "DSP2834x_Examples.h"

#define WREN_EX_FLASH		0x0006
#define WRDI_EX_FLASH		0x0004
#define RDID_EX_FLASH		0x009F
#define RDSR_EX_FLASH		0x0005
#define WRSR_EX_FLASH		0x0001
#define READ_EX_FLASH		0x0003
#define FAST_READ_EX_FLASH	0x000B
#define PP_EX_FLASH			0x0002
#define SE_EX_FLASH			0x00D8
#define BE_EX_FLASH			0x00C7
#define DP_EX_FLASH			0x00B9
#define RES_EX_FLASH		0x00AB
#define DUMMY_EX_FLASH		0xFFFF
#define STATUS_REG_EX_FLASH	0x0000

#define M25P40_API_VERSION		1.0L
#define M25P40_API_VERSION_HEX	0x0100

#define ADDRESS_HIGH(w)		((w>>16) & 0x00ff)
#define ADDRESS_MIDDLE(w)	((w>>8) & 0x00ff)
#define ADDRESS_LOW(w)		((w) & 0x00ff)

#define M25P40_CS_ON()	GpioDataRegs.GPACLEAR.bit.GPIO19 =1;
#define M25P40_CS_OFF() GpioDataRegs.GPASET.bit.GPIO19 =1;

#else
#define DSP2834x_M25P40_EXT extern

#endif

DSP2834x_M25P40_EXT Uint16 Spi8Driver(Uint16 Data);
DSP2834x_M25P40_EXT Uint16 AtoI(char In);
DSP2834x_M25P40_EXT void InitExFlash(void);
DSP2834x_M25P40_EXT void WriteEnableExFlash(void);
DSP2834x_M25P40_EXT void WriteDisableExFlash(void);
DSP2834x_M25P40_EXT Uint16 ReadStatusRegisterExFlash(void);
DSP2834x_M25P40_EXT void WriteStatusRegisterExFlash(Uint16 StatusRegister);
DSP2834x_M25P40_EXT Uint16 ReadExFlash(Uint32 Address);
DSP2834x_M25P40_EXT Uint16 FastReadExFlash(Uint32 Address);
DSP2834x_M25P40_EXT void PageProgramExFlash(Uint32 Address, Uint16 Data);
DSP2834x_M25P40_EXT void SectorEraseExFlash(Uint32 Address);
DSP2834x_M25P40_EXT void BulkEraseExFlash(void);
DSP2834x_M25P40_EXT void DeepPowerDownExFlash(void);
DSP2834x_M25P40_EXT void RES_ExFlash(void);
DSP2834x_M25P40_EXT float32 ApiVersionExFlash(void);
DSP2834x_M25P40_EXT Uint16 ApiVersionHexExFlash(void);


/*======================================================================
	End of file.
======================================================================*/





#endif /* DSP2834X_M25P40_H_ */
