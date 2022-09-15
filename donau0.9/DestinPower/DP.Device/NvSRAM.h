/*
 * NvSRAM.h
 *
 *  Created on: 2013. 2. 6.
 *      Author: destinPower
 */

#ifndef NVSRAM_H_
#define NVSRAM_H_

#include "LGS_Common.h"



public void NvSRAM_Create();
/* FOR PARAM */
public Bool WriteGroupInfoToNvSram(UInt idx, UInt16 data);
public Bool WriteParaVersionToNvSram(UInt idx, UInt16 data);
public Bool WriteParaCheckSumToNvSram(UInt idx, UInt16 data);
public void ReadGroupInfoFromNvSram(UInt idx, UInt16* data);
public void ReadParaVersionFromNvSram(UInt idx, UInt16* data);
public void ReadParaCheckSumFromNvSram(UInt idx, UInt16* data);
public UInt16 PARA_getParaVersion();

public Bool WritePRM_PCS(UInt16 addr, UInt16 data);
public void ReadPRM_PCS(UInt16 addr, UInt16 *data);
#if 0
public Bool WritePRM_VER(UInt16 addr, UInt16 data);
public void ReadPRM_VER(UInt16 addr, UInt16 *data);
#endif

/* FOR SYSTEM */
public void NvSRAM_SetRunCommand(Bool ON_OFF);
public Bool NvSRAM_CheckRunCommand();

/* FOR FAULT */
public void NvSRAM_SetFaultCommand(Bool ON_OFF);
public Bool NvSRAM_CheckFaultCommand();

/* FOR EVENT */
public void NvSRAM_SetEventInit();
public Bool NvSRAM_CheckEventInit();
public void NvSRAM_SetEventHead(UInt16 uLSB, UInt16 uMSB);
public UInt16 NvSRAM_GetEventHead();
public void NvSRAM_SetCacheFull(Bool TrueFalse);
public Bool NvSRAM_isCacheFull();
public void NvSRAM_StoreEvent(UInt16 *addr,UInt16 hisIdLSB, UInt16 hisIdMSB, UInt16 timeB1,UInt16 timeB2,UInt16 timeB3,UInt16 timeB4 );
public void NvSRAM_GetEvent(UInt16 addr,UInt16 *hisId, UInt16 *timeB1,UInt16 *timeB2,UInt16 *timeB3,UInt16 *timeB4 );
public UInt16 NvSRAM_GetEventId(UInt16 addr);

//for diagnostic
extern Bool NvSRAM_fail_flag;

#if DBUG_MODE || DEBUG_MODE
public Bool NvSRAM_Write(UInt16 address,UInt16 data, Bool bValid);
#endif

#endif /* NVSRAM_H_ */
