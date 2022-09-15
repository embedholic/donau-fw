/*
 * MCCB.h
 *
 *  Created on: 2013. 2. 15.
 *      Author: destinPower
 */

#ifndef MCCB_H_
#define MCCB_H_

#include "LGS_Common.h"

#define CBMC_CLOSED '1'
#define CBMC_OPEN 	'0'

typedef enum eCBMC_CTRLLIST {
#if 0 //by JCNET
	/*DDO Port*/CTRL_DS1_ON, CTRL_DS1_OFF, CTRL_DS1_UVR,  CTRL_DDO_REV4/* STBLN Sag&Swell 시 출력	*/,
//JCNET  두개 추가	CTRL_MC1_ONOFF, CTRL_MC2_ONOFF
//CTRL_MC1A_ONOFF는 기존에 있는것 사용
	CTRL_MC1A_ONOFF, CTRL_MC1_ONOFF, CTRL_MC2_ONOFF,// JCNET: 일단 converter에서 사용하는 두 스위치 이름을 정의...
	CTRL_SCFAN_CTRL,
	/*ADO Port*/CTRL_CB4_OFF, CTRL_CB4_ON, CTRL_MC8_ONOFF, CTRL_MC9_ONOFF, CTRL_CB3_UVR, CTRL_CB3_OFF, CTRL_CB3_ON, CTRL_SSEFAN_CTRL/* HIL SCR On/Off */, CTRL_REV4, CTRL_REV5, /* CN_D2 DOUT7 부터 */
#else
	CTRL_MC1_ON , CTRL_REV1,   CTRL_REV2,        CTRL_CB3_OFF ,  CTRL_MC1_OFF ,
	CTRL_MC2_OFF, CTRL_MC2_ON ,CTRL_MC1A_ONOFF ,
#endif
	CTRL_NONE
}CBMC_CTRL;
typedef enum eCBMC_STATUSLIST { // SR: 시동 스위치. 열쇠
#if 0 //by JCNET
	/* DDI */
    STATUS_MC1A, STATUS_MC1, STATUS_MC2, // by JCNET 임시 위치 !!! TODO
	STATUS_DS1,	STATUS_CB2,	STATUS_SMPS,	STATUS_SPD1,	STATUS_EPO,	STATUS_SSW_OT, STATUS_MC1A2A, STATUS_GFD_DDIA_END,
	/* ADI 1~8 (CN_D2 DIN-9) */
	STATUS_DOOR, STATUS_CB3, STATUS_EX_MCCB1, STATUS_EX_MCCB2, STATUS_EX_MCCB3, STATUS_SSW_FUSE, STATUS_MCB7, STATUS_RETEMP,
	/* ADI 9~16 FOR FR*/
	STATUS_TRTEMP, STATUS_CB4, STATUS_MC8, STATUS_MC9, STATUS_SPD4, STATUS_REV14, STATUS_ADI_REV15, STATUS_ADI_REV16,
#else
	STATUS_MC1     , STATUS_MC1A, STATUS_INDUCTOR, STATUS_FUSE_REC, STATUS_POWER_FAULT,
	STATUS_FUSE_INV, STATUS_DOOR, STATUS_MC2,      STATUS_SSW_OT     , STATUS_REV1,
	STATUS_REV2    , STATUS_CB1 , STATUS_CB2,      STATUS_CB3     , STATUS_REV3,
	STATUS_REV4    ,
	STATUS_NONE
#endif
}CBMC_STATUS;

typedef union _CbmcStatus
{
	UInt32	UnsVal; /* 주의! CBMC_STATUS enum의 순서랑 동일해야 함! */
	struct
	{
#if 0 //by JCNET
		//DDI
		Uns	bDS1    	:1;
		Uns	bCB2    	:1;
		Uns	bSMPS    	:1;
		Uns	bSPD1    	:1;
		Uns	bEPO    	:1;
		Uns	bSSW_OT  	:1;
		Uns bMC1A		:1;
		Uns bGFD		:1;
		//ADI - Chip 1
		Uns	bDoor    	:1;
		Uns	bCB3    	:1;
		Uns	bExMCCB1  	:1; // hill AO11
		Uns	bExMCCB2   	:1;
		Uns	bExMCCB3   	:1;
		Uns	bSSWFUSE   	:1;
		Uns	bMCB7    	:1; // hill AO15
		Uns	bRETEMP    	:1;

		//ADI - Chip 2
		Uns	bTRTEMP    	:1;  /*9: */
		Uns	bCB4    	:1;  /*10: */
		Uns	bMC8    	:1;  /*11: */
		Uns	bMC9    	:1;  /*12: */
		Uns	bSPD4    	:1;  /*13: */
		Uns	bADI_REV14 	:1;  /*14: */
		Uns	bADI_REV15 	:1;
		Uns	bADI_REV16 	:1;
#else
        Uns bMC1        :1; //DI1.1
        Uns bMC1A       :1; // 2
        Uns bInductor   :1; // 3
        Uns bFuseRec    :1; // 4
        Uns bPowerFault :1; // 5
        Uns bFuseInv    :1; // 6
        Uns bDoor       :1; // 7
        Uns bMC2        :1; // 8
        Uns bSSWOH      :1; // 9, SSW over heat fault
        Uns bREV1       :1; // 10
        Uns bREV2       :1; // 11
        Uns bCB1        :1; // 12
        Uns bCB2        :1; // 13
        Uns bCB3        :1; // 14
        Uns bREV3       :1; // 15
        Uns bREV4       :1; // 16
#endif
	} BitVal;
} CbmcStatus;

typedef union _CbmcControl
{
	Uns	UnsVal;
	struct
	{
#if 0 //by JCNET
		//DDO
		Uns	bDS1_ON		:1;
		Uns	bDS1_OFF	:1;
		Uns	bDS1_UVR	:1;
		Uns bDDO_REV4	:1;
		Uns bMC1A_ONOFF	:1;
		Uns bSCFAN_CTRL	:1;

		//ADO
		Uns bCB4_OFF		:1;
		Uns bCB4_ON			:1;
		Uns bMC8_ONOFF		:1;
		Uns bMC9_ONOFF		:1;
		Uns bCB3_UVR		:1;
		Uns bCB3_OFF		:1;
		Uns bCB3_ON			:1;
		Uns bSSEFAN_CTRL	:1;
		Uns bREV_2			:1;
		Uns bREV_3			:1;
#else
		Uns bMC1_ON :1;
		Uns bREV1   :1;
		Uns bREV2   :1;
		Uns bCB3_OFF:1;
		Uns bMC1_OFF:1;
		Uns bMC2_OFF:1;
		Uns bMC2_ON :1;
		Uns bMC1A_ONOFF:1;
#endif
	} BitVal;
} CbmcControl;
typedef struct _Cbmc
{
	CbmcStatus 	Status;
	CbmcStatus 	PrevStatus;
	CbmcControl	Command;
} Cbmc;


public void MCCB_Create(void);
public void MCCB_UpdateAllStatus(void);
public UInt32 MC_GetStatus(CBMC_STATUS ID);
public Bool MC_UpdateStatus(CBMC_CTRL ID, Bool OpenClosed);
public void MC_UpdateStatusForTest(CBMC_CTRL ID, Bool OpenClosed);


extern UInt32 u32CBMCStatus; // Call: MCCB_Create(), MCCB_UpdateAllStatus(), ResDIOSTATUS()
extern Cbmc MCCB_MIRROR;
extern Cbmc MCCB;
extern void MCB_Abnormal_Operation_Check(void);
#endif /* MCCB_H_ */
