/*
 * NvSRAM.c
 *
 *  Created on: 2013. 2. 6.
 *      Author: destinPower
 */
/*
 * Write 실패 확률에 대하여, 어디서 호출하는지 판단하여, Retry Count를 유연하게 설정할 수 있도록
 * 변경해야 하며, time Critical한 function에서 호출 하는 경우에는 retry count를 작게 잡고, write failed에 대한 처리 방안을
 * function내에서 처리하도록 해야 한다.
 */

#include "NvSRAM.h"
#include "McBSP_SPI.h"
#include <ti/sysbios/BIOS.h>
#include "Fault.h"

/*
 * 파라미터 구역
 *
 * addr
 * 0						0xFF  : Reserved				(255Byte)
 * 0x100					0x17FF : PARAM					(5.8KB)
 * 0x1800					0x18FF : PARAM INFO& SysCmd 	(255Byte)
 * 0x1900					0x19FF : Reserved				(512Byte)
 * 0x2000					0x47FF : EVENT 					(10KByte)
 * 0x5000					0x2CFF : BUFFER 				(11KByte)
 * 0x2D00					0x7FFF : Reserved
 * ----------------------------
 */
#define NVSRAM_ADDR_PARAM_STX 			0x10
#define NVSRAM_ADDR_PRM_PCS				0x10 	// Max 3056 Byte
#define NVSRAM_ADDR_PRM_VER				0xC00 	// Max 3071 Byte
#define NVSRAM_ADDR_PARAM_END 			0x17FF

#define NVSRAM_ADDR_PARA_GROUP_COUNT_INFO  	0x1800 // Max 16Byte
#define NVSRAM_ADDR_PARA_VERSION 			0x1810 // Max 4Byte
#define NVSRAM_ADDR_PARA_CHECKSUM  			0x1814 // Max 4Byte
#define NVSRAM_ADDR_SYS_RUN 				0x1818 // Max 4Byte
#define NVSRAM_ADDR_FAULT 					0x181C // Max 4Byte

#define NVSRAM_ADDR_PARA_RESERVED				0x1900

#define NVSRAM_ADDR_EVENT_STX 			0x2000
#define NVSRAM_ADDR_EVENT_INIT 			0x2000
#define NVSRAM_ADDR_EVENT_HEAD 			0x2004
#define NVSRAM_ADDR_EVENT_FULL 			0x2008 // 1 바이트만 사용(BOOL)
#define NVSRAM_ADDR_EVENT_DATA 			0x200A
#define NVSRAM_ADDR_EVENT_END 			0x47FF

#define NVSRAM_ADDR_CHECK_RAM_STX			0x4800
#define NVSRAM_ADDR_CHECK_RAM_END			0x47FD
//-#define NVSRAM_ADDR_CHECK_RAM_CS1			0x47FE
//-#define NVSRAM_ADDR_CHECK_RAM_CS2			0x47FF

#define NVSRAM_ADDR_BUFFER_STX 			0x5000
#define NVSRAM_ADDR_BUFFER_END 			0x7FFF

Bool NvSRAM_fail_flag = FALSE;

#if DBUG_MODE || DEBUG_MODE
public Bool NvSRAM_Write(UInt16 address,UInt16 data, Bool bValid);
#else
private Bool NvSRAM_Write(UInt16 address,UInt16 data, Bool bValid);
#endif

public void NvSRAM_Create()
{
	//Check NvSRAM
//	Int16 iVerify=0;
//	Int16 i = 0;
//
//	for(i=NVSRAM_ADDR_CHECK_RAM_STX; i<NVSRAM_ADDR_CHECK_RAM_END;i++)
//	{
//		if( !NvSRAM_Write(NVSRAM_ADDR_CHECK_RAM_STX, iVerify++,TRUE) )
//		{
//			FLT_Raise(FLTH_NVSRAM);
//		}
//	}
}
/*private*/ Bool NvSRAM_Read(UInt16 address,UInt16* data)
{
	if(address > NVSRAM_ADDR_END )
		return FALSE;

	return MCBSP_SPI_byteRead(address,data);
}

/*
 * address: 0~32000
 * data: 	int data( 1 byte만 사용 )
 * bValid:	Write 후 Read 하여 저장되었는지도 확인함.
 *
 */
#if DBUG_MODE || DEBUG_MODE
public Bool NvSRAM_Write(UInt16 address,UInt16 data, Bool bValid)
#else
private Bool NvSRAM_Write(UInt16 address,UInt16 data, Bool bValid)
#endif

{
	UInt16 iValid;
	UInt16 iRetryCount = 0;

	if(address > NVSRAM_ADDR_END )
	{
#if DEBUG_MODE == 1
		error();

		if(data > 255)
			error();
#endif
		return FALSE;
	}

	while(1)
	{
		MCBSP_SPI_byteWrite(address, data & 0xFF);

		if(!bValid)
			return TRUE;
		if(NvSRAM_Read(address,&iValid))
		{
			if( data != iValid)
			{
				if( iRetryCount++ > 5)
				{
#if DEBUG_MODE == 1
					error();
#endif
					return FALSE;
				}

				continue;
			}
			else//+
				break;//+

		}
	}
	return TRUE;
}

#if DEBUG_MODE == 1
private void checkDataRange(UInt16 data)
{

	if(data > 255)
	{
		error();
	}
}
#endif

/*========================================================================
 *
 * FOR PARAMETER
 *
 * =======================================================================
 */
// PARA_GROUP_INFO_ADDR
public Bool WriteGroupInfoToNvSram(UInt idx, UInt16 data)
{
#if DEBUG_MODE == 1
	if(idx >= 16)
		error();
	checkDataRange(data);
#endif
	return NvSRAM_Write(NVSRAM_ADDR_PARA_GROUP_COUNT_INFO+idx, data, TRUE );
}
// PARA_VERSION_INFO_ADDR
//idx:0 LSB, idx:1 MSB
public Bool WriteParaVersionToNvSram(UInt idx, UInt16 data)
{
#if DEBUG_MODE == 1
	if(idx > 1)
		error();
	checkDataRange(data);
#endif
	return NvSRAM_Write(NVSRAM_ADDR_PARA_VERSION+idx, data, TRUE );
}
// PARA_CHECKSUM_NV_ADDR
// idx:0 LSB, idx:1 MSB
public Bool WriteParaCheckSumToNvSram(UInt idx, UInt16 data)
{
#if DEBUG_MODE == 1
	if(idx > 1)
		error();
	checkDataRange(data);
#endif
	idx += NVSRAM_ADDR_PARA_CHECKSUM;
	return NvSRAM_Write(idx, data, TRUE );
}
// PARA_GROUP_INFO_ADDR
public void ReadGroupInfoFromNvSram(UInt idx, UInt16* data)
{
#if DEBUG_MODE == 1
	if(idx >= 16)
		error();
#endif
	NvSRAM_Read(NVSRAM_ADDR_PARA_GROUP_COUNT_INFO+idx, data);
}

// PARA_VERSION_INFO_ADDR
// idx:0 LSB, idx:1 MSB
public void ReadParaVersionFromNvSram(UInt idx, UInt16* data)
{
#if DEBUG_MODE == 1
	if(idx > 1)
		error();
#endif
	NvSRAM_Read(NVSRAM_ADDR_PARA_VERSION+idx, data);
}

// PARA_CHECKSUM_NV_ADDR
// idx:0 LSB, idx:1 MSB
public void ReadParaCheckSumFromNvSram(UInt idx, UInt16* data)
{
#if DEBUG_MODE == 1
	if(idx > 1)
		error();
#endif
	NvSRAM_Read(NVSRAM_ADDR_PARA_CHECKSUM+idx, data);
}
public UInt16 PARA_getParaVersion()
{
	UInt16 temp;
	UInt16 retVal;

	ReadParaVersionFromNvSram(0,&retVal);
	temp = retVal & 0xFF;

	ReadParaVersionFromNvSram(1,&retVal);
	temp += (retVal & 0xFF)<<8;

	return temp;
}

/*
 * nvstcPRM_PCS_NV
 */
public Bool WritePRM_PCS(UInt16 addr, UInt16 data)
{
#if DEBUG_MODE == 1
	checkDataRange(data);
#endif
	data = data & 0xFF;
	addr += NVSRAM_ADDR_PRM_PCS;
	return NvSRAM_Write(addr, data, TRUE );
}
/*
 * nvstcPRM_PCS_NV
 */
public void ReadPRM_PCS(UInt16 addr, UInt16 *data)
{
	addr += NVSRAM_ADDR_PRM_PCS;
	NvSRAM_Read(addr, data );
	return;
}
#if 0
/*
 * nvstcPRM_VER_NV
 */
public Bool WritePRM_VER(UInt16 addr, UInt16 data)
{
#if DEBUG_MODE == 1
	checkDataRange(data);
#endif
	data = data & 0xFF;

	addr += NVSRAM_ADDR_PRM_VER;
	return NvSRAM_Write(addr, data, TRUE );
}
/*
 * nvstcPRM_VER_NV
 */
public void ReadPRM_VER(UInt16 addr, UInt16 *data)
{

	addr += NVSRAM_ADDR_PRM_VER;
	NvSRAM_Read(addr, data );
	return;
}
#endif

/*========================================================================
 *
 * FOR SYSTEM
 *
 * =======================================================================
 */
int bNvSRAM_SetOnCommand = -1;
int bNvSRAM_CheckOnCommand = -1;

public void NvSRAM_SetRunCommand(Bool ON_OFF)
{
	if( bNvSRAM_SetOnCommand != ON_OFF )
		bNvSRAM_SetOnCommand = ON_OFF;
	else
		return; // 이미 저장되었으면 skip

	if ( ON_OFF == ON )
	{
		NvSRAM_Write(NVSRAM_ADDR_SYS_RUN, 0x05, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_SYS_RUN+1, 0x50, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_SYS_RUN+2, 0x0A, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_SYS_RUN+3, 0xA0, FALSE);
	}
	else
	{
		NvSRAM_Write(NVSRAM_ADDR_SYS_RUN, 0x0, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_SYS_RUN+1, 0x0, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_SYS_RUN+2, 0x0, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_SYS_RUN+3, 0x0, FALSE);
	}

	bNvSRAM_CheckOnCommand = -1;
}
/*
 * NvSRAM에 저장된 Run 상태를 리턴한다.
 * TRUE: ON, FALSE: OFF
 * TESTED
 */
public Bool NvSRAM_CheckRunCommand()
{
	UInt16 b1,b2,b3,b4;
	b1 = b2 = b3 = b4 = 0;

	if( bNvSRAM_CheckOnCommand != -1 )
		return bNvSRAM_CheckOnCommand;

	NvSRAM_Read(NVSRAM_ADDR_SYS_RUN,&b1);
	NvSRAM_Read(NVSRAM_ADDR_SYS_RUN+1,&b2);
	NvSRAM_Read(NVSRAM_ADDR_SYS_RUN+2,&b3);
	NvSRAM_Read(NVSRAM_ADDR_SYS_RUN+3,&b4);

	if ( (b1&0xFF) == 0x05 && (b2&0xFF) == 0x50 && (b3&0xFF) == 0x0A &&(b4&0xFF) == 0xA0 )
	{
		bNvSRAM_CheckOnCommand = 1;
		return TRUE;
	}
	bNvSRAM_CheckOnCommand = 0;
	return FALSE;
}

/*========================================================================
 *
 * FOR FAULT
 *
 * =======================================================================
 */
public void NvSRAM_SetFaultCommand(Bool ON_OFF)
{
	if ( ON_OFF == ON )
	{
		NvSRAM_Write(NVSRAM_ADDR_FAULT, 0x05, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_FAULT+1, 0x50, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_FAULT+2, 0x0A, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_FAULT+3, 0xA0, FALSE);
	}
	else
	{
		NvSRAM_Write(NVSRAM_ADDR_FAULT, 0x0, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_FAULT+1, 0x0, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_FAULT+2, 0x0, FALSE);
		NvSRAM_Write(NVSRAM_ADDR_FAULT+3, 0x0, FALSE);
	}
}
/*
 * NvSRAM에 저장된 Run 상태를 리턴한다.
 * TRUE: ON, FALSE: OFF
 * TESTED
 */
public Bool NvSRAM_CheckFaultCommand()
{
	UInt16 b1,b2,b3,b4;

	NvSRAM_Read(NVSRAM_ADDR_FAULT,&b1);
	NvSRAM_Read(NVSRAM_ADDR_FAULT+1,&b2);
	NvSRAM_Read(NVSRAM_ADDR_FAULT+2,&b3);
	NvSRAM_Read(NVSRAM_ADDR_FAULT+3,&b4);

	if ( (b1&0xFF) == 0x05 && (b2&0xFF) == 0x50 && (b3&0xFF) == 0x0A &&(b4&0xFF) == 0xA0 )
	{
		return TRUE;
	}
	return FALSE;
}

/*========================================================================
 *
 * FOR EVENT
 *
 * =======================================================================
 */
public void NvSRAM_SetEventInit()
{
	NvSRAM_Write(NVSRAM_ADDR_EVENT_INIT, 0xAA, FALSE);
	NvSRAM_Write(NVSRAM_ADDR_EVENT_INIT+1, 0xAA, FALSE);
	NvSRAM_Write(NVSRAM_ADDR_EVENT_INIT+2, 0x55, FALSE);
	NvSRAM_Write(NVSRAM_ADDR_EVENT_INIT+3, 0x55, FALSE);
}

public Bool NvSRAM_CheckEventInit()
{
	UInt16 b1,b2,b3,b4;

	NvSRAM_Read(NVSRAM_ADDR_EVENT_INIT,&b1);
	NvSRAM_Read(NVSRAM_ADDR_EVENT_INIT+1,&b2);
	NvSRAM_Read(NVSRAM_ADDR_EVENT_INIT+2,&b3);
	NvSRAM_Read(NVSRAM_ADDR_EVENT_INIT+3,&b4);

	if ( (b1&0xFF) == 0xAA && (b2&0xFF) == 0xAA && (b3&0xFF) == 0x55 &&(b4&0xFF) == 0x55 )
	{
		return TRUE;
	}

	return FALSE;
}

public void NvSRAM_SetEventHead(UInt16 uLSB, UInt16 uMSB)
{
	NvSRAM_Write(NVSRAM_ADDR_EVENT_HEAD, uMSB, FALSE);
	NvSRAM_Write(NVSRAM_ADDR_EVENT_HEAD+1, uLSB , FALSE);
}

public UInt16 NvSRAM_GetEventHead()
{
	UInt16 b1,b2,b3;
	b1 = b2 = b3 = 0;
	NvSRAM_Read(NVSRAM_ADDR_EVENT_HEAD  ,&b1); // MSB
	NvSRAM_Read(NVSRAM_ADDR_EVENT_HEAD+1,&b2); // LSB

	b3 = ((b1 & 0xFF)<<8) + ((b2&0xFF));
	return b3;
}
public void NvSRAM_SetCacheFull(Bool TrueFalse)
{
	NvSRAM_Write(NVSRAM_ADDR_EVENT_FULL, (UInt16)TrueFalse, FALSE);
}
public Bool NvSRAM_isCacheFull()
{
	UInt16 b1 = 0;
	NvSRAM_Read(NVSRAM_ADDR_EVENT_FULL  ,&b1); // MSB

	return (Bool)(b1 & 0xFF);
}

Bool reiteration = FALSE;
public void NvSRAM_StoreEvent(UInt16 *addr,UInt16 hisIdLSB, UInt16 hisIdMSB, UInt16 timeB1,UInt16 timeB2,UInt16 timeB3,UInt16 timeB4 )
{
	UInt16 idx;
	idx = NVSRAM_ADDR_EVENT_DATA + *addr;

	NvSRAM_Write(idx, 	hisIdMSB& 0xFF , TRUE);
	NvSRAM_Write(idx+1, hisIdLSB& 0xFF , TRUE);
	NvSRAM_Write(idx+2, timeB1& 0xFF , TRUE);
	NvSRAM_Write(idx+3, timeB2& 0xFF , TRUE);
	NvSRAM_Write(idx+4, timeB3& 0xFF , TRUE);
	NvSRAM_Write(idx+5, timeB4& 0xFF , TRUE);

	*addr += 6;
	// 1024 * 6 = (6144(0x1800)) + 0x200A = 0x380A
}

public void NvSRAM_GetEvent(UInt16 addr,UInt16 *hisId, UInt16 *timeB1,UInt16 *timeB2,UInt16 *timeB3,UInt16 *timeB4 )
{
	UInt16 b1,b2,idx;
	idx = NVSRAM_ADDR_EVENT_DATA + addr;

	NvSRAM_Read(idx  ,&b1); // MSB
	NvSRAM_Read(idx+1,&b2); // LSB
	*hisId = ((b1 & 0xFF)<<8) + (b2 & 0xFF);

	NvSRAM_Read(idx+2,timeB1);
	NvSRAM_Read(idx+3,timeB2);
	NvSRAM_Read(idx+4,timeB3);
	NvSRAM_Read(idx+5,timeB4);

	return;
}
public UInt16 NvSRAM_GetEventId(UInt16 addr)
{
	UInt16 b1,b2,b3, idx;
	idx = NVSRAM_ADDR_EVENT_DATA + addr;

	NvSRAM_Read(idx  ,&b1); // MSB
	NvSRAM_Read(idx+1,&b2); // LSB
	b3 = ((b1 & 0xFF)<<8) + (b2 & 0xFF);

	return b3;
}
