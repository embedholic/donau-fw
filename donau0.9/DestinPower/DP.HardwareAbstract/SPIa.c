/*
 * SPIa.c - FLASH MEMORY(BOOT,DATA)
 *
 * DATA FLASH : 4 MB - S25FL032
 *
 *  Created on: 2012. 11. 9.
 *      Author: destinPower
 */

#include "SPIa.h"
#include "DSP28x_Project.h"
#include "slld.h"
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>

private void SPIA_bootFlash_DIS();
private void SPIA_dataFlash_DIS();
private void SPIA_registerSetting(SPI_SELECT spiSelect);


public void SPIA_init()
{
	SPIA_bootFlash_ENB();
}

void SPIA_fifo_init()
{
// Initialize SPI FIFO registers
    SpiaRegs.SPIFFTX.all=0xE040;
    SpiaRegs.SPIFFRX.all=0x204f;
    SpiaRegs.SPIFFCT.all=0x0;
}

private void SPIA_registerSetting(SPI_SELECT spiSelect)
{
	if(spiSelect == BOOT_FLASH)
	{
		/* Initialize internal SPI peripheral module */
		SpiaRegs.SPICCR.all = 0x0000;
		SpiaRegs.SPICTL.all = 0x0000;
		SpiaRegs.SPIBRR = 0x0000;
		SpiaRegs.SPIPRI.all = 0x0000;

		SpiaRegs.SPICCR.bit.SPISWRESET=0;				// Reset SPI
		SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;			// SPI_LOAD_FALLING
		SpiaRegs.SPICCR.bit.SPICHAR = 7;				// 8bit ������ ũ�� ���
		SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;			// ������ ���
		SpiaRegs.SPICTL.bit.TALK = 1;					// ��� Ȱ��ȭ
		SpiaRegs.SPIBRR = 29;							// 10Mbps
		SpiaRegs.SPIPRI.bit.FREE = 1;
		SpiaRegs.SPICCR.bit.SPISWRESET=1;
	}
	else if(spiSelect == DATA_FLASH)
	{
		SpiaRegs.SPICCR.all = 0x0000;
		SpiaRegs.SPICTL.all = 0x0000;
		SpiaRegs.SPIBRR = 0x0000;
		SpiaRegs.SPIPRI.all = 0x0000;

		SpiaRegs.SPICCR.bit.SPISWRESET=0;				// Reset SPI
		SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;			// RISING EDGE
		SpiaRegs.SPICCR.bit.SPICHAR = 7;				// 8bit ������ ũ�� ���
		SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;			// ������ ���
		SpiaRegs.SPICTL.bit.TALK = 1;					// ��� Ȱ��ȭ

		SpiaRegs.SPICTL.bit.CLK_PHASE  = 0;				// with Delay

		SpiaRegs.SPIBRR = 9;							// SPI ��żӵ� ����: 300MHz/(SPIBRR+1) = 30Mhz
		SpiaRegs.SPIPRI.bit.FREE = 1;
		SpiaRegs.SPICCR.bit.SPISWRESET=1;
	}
}

public void SPIA_bootFlash_ENB()
{
	/* �ŷڼ��� ���̱� ���ؼ��� SPI�� �۾� ���� �ƴ� ��쿡 ���� �����ϵ��� �����ؾ� �Ѵ�. */
	SPIA_registerSetting(BOOT_FLASH);
	SPIA_dataFlash_DIS();

	GpioDataRegs.GPBDAT.bit.GPIO39 = 1; // WP#
	//-GpioDataRegs.GPADAT.bit.GPIO19 = 0; // CS�� slld ���� ����
}
public void SPIA_dataFlash_ENB()
{
	SPIA_registerSetting(DATA_FLASH);
	SPIA_bootFlash_DIS();

	GpioDataRegs.GPADAT.bit.GPIO29 = 1; // WP#
	//-GpioDataRegs.GPADAT.bit.GPIO40 = 0; // CS�� slld ���� ����
}
private void SPIA_bootFlash_DIS()
{
	GpioDataRegs.GPBDAT.bit.GPIO39 = 0; // WP#
	GpioDataRegs.GPADAT.bit.GPIO19 = 1; // CS#
}
private void SPIA_dataFlash_DIS()
{
	GpioDataRegs.GPADAT.bit.GPIO29 = 0; // WP#

#if	MCU_PCB_REV_VER < MCU_PCB_REV_B_G
	GpioDataRegs.GPADAT.bit.GPIO27 = 1; // CS#
#else
	GpioDataRegs.GPBDAT.bit.GPIO40 = 1; // CS#
#endif
}


// 28346�� �ּ� Byte������ 2�̹Ƿ�, 2����Ʈ�� ������ ���� 1����Ʈ�� ����Ͽ� Write ��. 2����Ʈ�� ��� ����� ��� ���ο��� ������ ó������� ��.
// TODO test
public Bool SPIA_dataFlash_writeInt16(UInt32 address/*24bit*/,char* const pData, Uint16 size)
{
	DEVSTATUS s;
	int i = 0;
	char uWriteBuf[512];

	if( size > 256 )
		return FALSE;

	for( i=0; i<size*2; i++)
	{
		if( i % 2 )
			uWriteBuf[i*2+1] = (pData[i] >> 8) & 0xFF ;
		else
			uWriteBuf[i*2] = pData[i] & 0xFF ;
	}

	// 0~255
	if( slld_PPOp(address, uWriteBuf, size, &s) != SLLD_OK)
	{
		return FALSE;
	}

	// 256~512
	if( slld_PPOp(address+size, uWriteBuf+(size), size, &s) != SLLD_OK)
	{
		return FALSE;
	}

	if(s != dev_not_busy )
	{
		return FALSE;
	}

	return TRUE;
}


public Bool SPIA_dataFlash_readByteInt16(UInt32 address,char* const pData, Uint16 size)
{
	int i;
	char uReadBuf[512];

	if( size > 256 )
		return FALSE;



	if(slld_ReadOp(address, uReadBuf, 256) != SLLD_OK)
	{
		return FALSE;
	}

	if(slld_ReadOp(address+size, uReadBuf+256, 256) != SLLD_OK)
	{
		return FALSE;
	}

	// Merge
	for( i=0; i<size*2; i++)
	{
		if( i % 2 )
			pData[i/2+1] = (uReadBuf[i] >> 8) & 0xFF ;
		else
			pData[i/2] = uReadBuf[i] & 0xFF ;
	}


	return TRUE;
}


// 28346�� �ּ� Byte������ 2�̹Ƿ�, 2����Ʈ�� ������ ���� 1����Ʈ�� ����Ͽ� Write ��. 2����Ʈ�� ��� ����� ��� ���ο��� ������ ó������� ��.
public Bool SPIA_dataFlash_writeByte(UInt32 address/*24bit*/,char* const pData, Uint16 size)
{
	DEVSTATUS s;

	if( size > 256 )
		return FALSE;

	if( slld_PPOp(address, pData, size, &s) != SLLD_OK)
	{
		return FALSE;
	}

	if(s != dev_not_busy )
	{
		return FALSE;
	}

	return TRUE;
}

public Bool SPIA_dataFlash_readByte(UInt32 address,char* const pData, Uint16 size)
{
	if( size > 256 )
		return FALSE;

	if(slld_ReadOp(address, pData, size) != SLLD_OK)
	{
		return FALSE;
	}

	return TRUE;
}

public Bool SPIA_dataFlash_Erase()
{
	DEVSTATUS s;

	/* Erase - ���ʹ���. ��ü ������ Bulk Erase BEop ���. */
	if(slld_SEOp(0x000000, &s) != SLLD_OK)
	{
		return FALSE;
	}

	return TRUE;
}

/*
	 dev_status_unknown = 0,
	 dev_not_busy,
	 dev_program_error,
	 dev_erase_error,
	 dev_suspend,
	 dev_busy
 */
public int SPIA_dataFlash_ReadStatus()
{
	DEVSTATUS dfDevStatus = dev_status_unknown;

	if( slld_StatusGet(&dfDevStatus) == SLLD_OK )
	{
		return dfDevStatus;
	}

	dfDevStatus = dev_status_unknown;
	return dfDevStatus;

}
