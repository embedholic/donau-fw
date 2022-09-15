/*
 * CAN_GBI.c
 *
 *  Created on: 2013. 9. 5.
 *      Author: destinPower
 */

#include "CAN_GBI.h"
#include "CAN.h"
#include "DSP2834x_Device.h"
#include <ti/sysbios/knl/Task.h>
#include "string.h"
#include "trace.h"
#include "Fault.h"
#include "PRM_PCS.h"


// Global variables used in this system
struct ECAN_REGS ECanaShadow;
GBI_COMM_TX_DATA g_gbiTxData[8/*PEBB_COUNT_MAX*/];
GBI_COMM_RX_DATA g_gbiRxData[8/*PEBB_COUNT_MAX*/]; // 최대 8대. 1M
CAN_RX_FRAME	 m_RxFrame;

static GBI_CAN_TX_STATUS m_stTxStatus_gbi;

void CAN_InitMailbox_A(void);
void CAN_GBI_InitVar(void);
interrupt void ISR_CAN_A_Rx(void);

extern void InitECana(void);
extern void InitECanaGpio(void);

void CAN_GBI_Create()
{
	// ECAN-A : eGBI
	InitECanaGpio();
	InitECana();						// CAN 초기화 : CAN2.0A 모드, 250kbps 속도
	CAN_InitMailbox_A();
	CAN_GBI_InitVar();

	// Vector Remapping
	EALLOW;
	PieCtrlRegs.PIEIER9.bit.INTx5 = 1;			// PIE 인터럽트(ECAN0INTA) Enable
	IER |= M_INT9; 								// CPU 인터럽트(INT9) Enable
	PieVectTable.ECAN0INTA = &ISR_CAN_A_Rx;
	EDIS;
}

void CAN_GBI_AddInfoNode(void)
{
#if 0
	TRC_AddNode((InfoID)0, TRC_UNS, &g_gbiRxData[0].rxM0.STATE.byte16);
	TRC_AddNode((InfoID)1, TRC_UNS, &g_gbiRxData[0].rxM1.currentR);
	TRC_AddNode((InfoID)2, TRC_UNS, &g_gbiRxData[0].rxM1.currentS);
	TRC_AddNode((InfoID)3, TRC_UNS, &g_gbiRxData[0].rxM1.currentT);
	TRC_AddNode((InfoID)4, TRC_UNS, &g_gbiRxData[0].rxM2.currentR_rect);
	TRC_AddNode((InfoID)5, TRC_UNS, &g_gbiRxData[0].rxM2.currentS_rect);
	TRC_AddNode((InfoID)6, TRC_UNS, &g_gbiRxData[0].rxM2.currentT_rect);
	TRC_AddNode((InfoID)7, TRC_UNS, &g_gbiRxData[0].rxM3.tempR);
	TRC_AddNode((InfoID)8, TRC_UNS, &g_gbiRxData[0].rxM3.tempS);
	TRC_AddNode((InfoID)9, TRC_UNS, &g_gbiRxData[0].rxM3.tempT);
	TRC_AddNode((InfoID)10, TRC_UNS, &g_gbiRxData[0].rxM4.tempHeatSync);
	TRC_AddNode((InfoID)11, TRC_UNS, &g_gbiRxData[0].rxM4.vdc);

	TRC_AddNode((InfoID)20, TRC_UNS, &g_gbiRxData[1].rxM0.STATE.byte16);
	TRC_AddNode((InfoID)21, TRC_UNS, &g_gbiRxData[1].rxM1.currentR);
	TRC_AddNode((InfoID)22, TRC_UNS, &g_gbiRxData[1].rxM1.currentS);
	TRC_AddNode((InfoID)23, TRC_UNS, &g_gbiRxData[1].rxM1.currentT);
	TRC_AddNode((InfoID)24, TRC_UNS, &g_gbiRxData[1].rxM2.currentR_rect);
	TRC_AddNode((InfoID)25, TRC_UNS, &g_gbiRxData[1].rxM2.currentS_rect);
	TRC_AddNode((InfoID)26, TRC_UNS, &g_gbiRxData[1].rxM2.currentT_rect);
	TRC_AddNode((InfoID)27, TRC_UNS, &g_gbiRxData[1].rxM3.tempR);
	TRC_AddNode((InfoID)28, TRC_UNS, &g_gbiRxData[1].rxM3.tempS);
	TRC_AddNode((InfoID)29, TRC_UNS, &g_gbiRxData[1].rxM3.tempT);
	TRC_AddNode((InfoID)30, TRC_UNS, &g_gbiRxData[1].rxM4.tempHeatSync);
	TRC_AddNode((InfoID)31, TRC_UNS, &g_gbiRxData[1].rxM4.vdc);
#endif
}

void CAN_GBI_InitVar()
{
	int i = 0;

	memset(&(g_gbiTxData), 0, sizeof(g_gbiTxData));
	memset(&(g_gbiRxData), 0, sizeof(g_gbiRxData));

	m_stTxStatus_gbi.mb0period_ms	= 300;

	for( i = 0; i<PEBB_COUNT_MAX; i++)
	{
		g_gbiTxData[i].C0.C0_0U.BIT.GATE_ONOFF = 0;
		g_gbiTxData[i].C0.C0_0U.BIT.MC4_ONOFF = 0;
		g_gbiTxData[i].C0.pebbId = i;
		g_gbiTxData[i].bTxNew = 1;
	}
}

void CAN_GBI_ManualOperation(int OnOff)
{
	int i = 0;
	for( i = 0; i<PEBB_COUNT_MAX; i++)
	{
		g_gbiTxData[i].C0.C0_0U.BIT.MANUAL_OP = OnOff;
		g_gbiTxData[i].bTxNew = 1;
	}
}

void CAN_GBI_FanSpeed(int speed)
{
	int i = 0;
	for( i = 0; i<PEBB_COUNT_MAX; i++)
	{
		g_gbiTxData[i].C0.C0_1U.BIT.FanSpeed = speed;
		g_gbiTxData[i].bTxNew = 1;
	}
	PRM_PCS[CTRL_COSPHI_P_6].iValue = speed;
}

void check_txPeriod(uint16_t *tick, uint16_t period, GBI_CAN_MSG type)
{
	int i=0;

	if( (*tick != 0) && (period != 0) && (*tick >= period))
	{

#if 0
		CAN_GBI_SendCmd(&(g_gbiTxData[0]), type, 0);
#else
		for(i=0; i<PEBB_COUNT_MAX; i++)
		{
			//-if( g_gbiTxData[i].bTxNew )
			//140717
				CAN_GBI_SendCmd(&(g_gbiTxData[i]), type, i);
		}
#endif
		*tick = 0;
	}
}

#if 1 //-0
void addGbiTxTick(GBI_CAN_TX_STATUS *pstTxStatus, int acc)
{
	if(pstTxStatus->mb0tick_ms < pstTxStatus->mb0period_ms) pstTxStatus->mb0tick_ms += acc;
}
#endif
// 5ms timer : 다른 곳에서 호출 시켜줘야 한다.
// Call : c_int25
void CAN_GbiTimer_5ms(void)
{
	addGbiTxTick(&m_stTxStatus_gbi, 5);
}

#if 1 //-0
void CAN_GBI_Tx_proc(GBI_CAN_TX_STATUS *pstTxStatus)
{
	// TODO 주기적인 송신 중지시 주석처리.
	check_txPeriod(&(pstTxStatus->mb0tick_ms),	pstTxStatus->mb0period_ms,	M_BOX1);
}
#endif

void CAN_GBI_proc(void)
{
	CAN_GBI_Tx_proc(&m_stTxStatus_gbi);
	//-CAN_Rx_proc(&m_stRxStatus);
}

#if DEBUG_MODE != NORMAL
	unsigned int iTxCount = 0;
	unsigned int iRxCount = 0;
#endif
int CAN_GBI_SendCmd(GBI_COMM_TX_DATA *pData, GBI_CAN_MSG cmd, Uint16 pebbId)
{
	int Status = 0;
	int waitcnt=0;

	// Mutex Lock
#if DEBUG_MODE != NORMAL
	iTxCount++;
#endif

#if 0
	// Fault Reset CMD는 모든 PEBB이 받도록 한다.
	if( cmd == M_BOX1 )
	{
		if( pData->C0.C0_0U.BIT.FAULT_RESET )
		{
			pebbId = 0xAA;
		}
	}
#endif

	switch(cmd)
	{
	case M_BOX1:
		pData->C0.pebbId = pebbId;
//-		ECanbMaoxes.MBOX0.MSGID.bit.STDMSGID = 0x100; // June Added. 그러나 여기에서 넣은 것은 적용되지 않음..
		ECanaMboxes.MBOX0.MDL.byte.BYTE0 = (pData->C0.C0_0U.c0_0u >> 8 & 0xFF) ;
		ECanaMboxes.MBOX0.MDL.byte.BYTE1 = (pData->C0.C0_0U.c0_0u & 0xFF);
		ECanaMboxes.MBOX0.MDL.byte.BYTE2 = (pData->C0.C0_1U.c0_1u >> 8 & 0xFF) ;
		ECanaMboxes.MBOX0.MDL.byte.BYTE3 = (pData->C0.C0_1U.c0_1u & 0xFF);
		ECanaMboxes.MBOX0.MDH.byte.BYTE4 = pData->C0.MoCtrlDelay;
		ECanaMboxes.MBOX0.MDH.byte.BYTE5 = (PRM_PCS[CTRL_CC_PERIOD].iValue  >> 8 & 0xFF);
		ECanaMboxes.MBOX0.MDH.byte.BYTE6 = (PRM_PCS[CTRL_CC_PERIOD].iValue & 0xFF);
		ECanaMboxes.MBOX0.MDH.byte.BYTE7 = pData->C0.pebbId; // 0xAA = 모든 PEBB
		break;
	default :
		Status = -1;
		return Status;
	}

	// 메일박스 데이터 전송 요청
	ECanaShadow.CANTRS.all = 0;
	ECanaShadow.CANTRS.bit.TRS0 = 1;
	ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;

	// 메일박스 데이터 송신 완료 플래그 대기
	do{
		ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
		Task_sleep(2);
		if((waitcnt++)>20){
			waitcnt=0;
			Status = -1; // timeout
			break;
		}
	}while(!ECanaShadow.CANTA.bit.TA0);

	// 메일박스 데이터 송신 완료 플래그 초기화
	ECanaShadow.CANTA.all = 0;
	ECanaShadow.CANTA.bit.TA0 = 1; // 0이 아님에 주의.
	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

	if( cmd == M_BOX1 )
	{
		if( pData->C0.C0_0U.BIT.FAULT_RESET && Status != -1  )
		{
			pData->C0.C0_0U.BIT.FAULT_RESET = 0;

			if( pebbId < PEBB_COUNT_MAX)
			{
				FLT_PEBB[pebbId].data.STATE.BIT.EMERGENCY = 0;
				FLT_PEBB[pebbId].data.STATE.BIT.HEATSYNC_OT = 0;
				FLT_PEBB[pebbId].data.STATE.BIT.IGBT_FAN_FAULT = 0;
				FLT_PEBB[pebbId].data.STATE.BIT.IGBT_FUSE_FAULT = 0;
				FLT_PEBB[pebbId].data.STATE.BIT.IGBT_MC_FAULT = 0;
				FLT_PEBB[pebbId].data.STATE.BIT.IGBT_OVERTEMP = 0;
				FLT_PEBB[pebbId].data.STATE.BIT.IGBT_R_FAULT = 0;
				FLT_PEBB[pebbId].data.STATE.BIT.IGBT_S_FAULT = 0;
				FLT_PEBB[pebbId].data.STATE.BIT.IGBT_T_FAULT = 0;
				FLT_PEBB[pebbId].data.STATE.BIT.PARAMETER_ERR = 0;
			}

			FLT_Clear(FLTH_PEBB_OC);
			FLT_Clear(FLTH_PEBB_OVERTEMP);
			FLT_Clear(FLTH_PEBB_FAN_FAULT);
			FLT_Clear(FLTH_PEBB_FUSE_FAULT);
			FLT_Clear(FLTH_PEBB_IGBT);
		}
	}

	if( Status != -1 )
		pData->bTxNew = 0;

	return Status;
}



#define MAKE_16BIT(a, b)	( ( ((a)&(0xFF))<<8 ) | ( ((b)&(0xFF)) ) )
#define PEBB_RANGE_CHECK if( pebb_addr <0 || pebb_addr >= PEBB_COUNT_MAX )	break
interrupt void ISR_CAN_A_Rx(void)
{
	int pebb_addr;

#if DEBUG_MODE != NORMAL
	iRxCount++;
#endif

	m_RxFrame.msgid   = ECanaMboxes.MBOX1.MSGID.bit.STDMSGID;
	m_RxFrame.data[0] = (ECanaMboxes.MBOX1.MDL.byte.BYTE0 & 0xFF);
	m_RxFrame.data[1] = (ECanaMboxes.MBOX1.MDL.byte.BYTE1 & 0xFF);
	m_RxFrame.data[2] = (ECanaMboxes.MBOX1.MDL.byte.BYTE2 & 0xFF);
	m_RxFrame.data[3] = (ECanaMboxes.MBOX1.MDL.byte.BYTE3 & 0xFF);
	m_RxFrame.data[4] = (ECanaMboxes.MBOX1.MDH.byte.BYTE4 & 0xFF);
	m_RxFrame.data[5] = (ECanaMboxes.MBOX1.MDH.byte.BYTE5 & 0xFF);
	m_RxFrame.data[6] = (ECanaMboxes.MBOX1.MDH.byte.BYTE6 & 0xFF);
	m_RxFrame.data[7] = (ECanaMboxes.MBOX1.MDH.byte.BYTE7 & 0xFF);

	pebb_addr = m_RxFrame.data[0];

	switch(m_RxFrame.msgid)
	{
	case 0x200:
		PEBB_RANGE_CHECK;
		//byte[1] is reserved.
		g_gbiRxData[pebb_addr].rxM0.STATE.byte16 = MAKE_16BIT(m_RxFrame.data[2],m_RxFrame.data[3]);
		g_gbiRxData[pebb_addr].rxM0.rev1 = MAKE_16BIT(m_RxFrame.data[4],m_RxFrame.data[5]);
		g_gbiRxData[pebb_addr].rxM0.rev2 = MAKE_16BIT(m_RxFrame.data[6],m_RxFrame.data[7]);

		// Set FLT info
		FLT_PEBB[pebb_addr].data.STATE.byte16 = g_gbiRxData[pebb_addr].rxM0.STATE.byte16;
		break;
	case 0x201:
		PEBB_RANGE_CHECK;
		g_gbiRxData[pebb_addr].rxM1.currentR = MAKE_16BIT(m_RxFrame.data[2],m_RxFrame.data[3]);
		g_gbiRxData[pebb_addr].rxM1.currentS = MAKE_16BIT(m_RxFrame.data[4],m_RxFrame.data[5]);
		g_gbiRxData[pebb_addr].rxM1.currentT = MAKE_16BIT(m_RxFrame.data[6],m_RxFrame.data[7]);
		break;
	case 0x202:
		PEBB_RANGE_CHECK;
		g_gbiRxData[pebb_addr].rxM2.currentR_rect = MAKE_16BIT(m_RxFrame.data[2],m_RxFrame.data[3]);
		g_gbiRxData[pebb_addr].rxM2.currentS_rect = MAKE_16BIT(m_RxFrame.data[4],m_RxFrame.data[5]);
		g_gbiRxData[pebb_addr].rxM2.currentT_rect = MAKE_16BIT(m_RxFrame.data[6],m_RxFrame.data[7]);
		break;
	case 0x203:
		PEBB_RANGE_CHECK;
		g_gbiRxData[pebb_addr].rxM3.tempR = MAKE_16BIT(m_RxFrame.data[2],m_RxFrame.data[3]);
		g_gbiRxData[pebb_addr].rxM3.tempS = MAKE_16BIT(m_RxFrame.data[4],m_RxFrame.data[5]);
		g_gbiRxData[pebb_addr].rxM3.tempT = MAKE_16BIT(m_RxFrame.data[6],m_RxFrame.data[7]);

		if(pebb_addr == 0)
			PRM_PCS[BATT_PEBB_0_TEMP_S].iValue =g_gbiRxData[pebb_addr].rxM3.tempS;
		else if(pebb_addr == 1)
			PRM_PCS[BATT_PEBB_1_TEMP_S].iValue =g_gbiRxData[pebb_addr].rxM3.tempS;
		break;
	case 0x204:
		PEBB_RANGE_CHECK;
		g_gbiRxData[pebb_addr].rxM4.tempHeatSync = MAKE_16BIT(m_RxFrame.data[2],m_RxFrame.data[3]);
		g_gbiRxData[pebb_addr].rxM4.vdc = MAKE_16BIT(m_RxFrame.data[4],m_RxFrame.data[5]);
		g_gbiRxData[pebb_addr].rxM4.rev = MAKE_16BIT(m_RxFrame.data[6],m_RxFrame.data[7]);

		if(pebb_addr == 0)
			PRM_PCS[BATT_PEBB_0_HEATSYNC].iValue = g_gbiRxData[pebb_addr].rxM4.tempHeatSync;
//		else if(pebb_addr == 1)
//			PRM_PCS[BATT_PEBB_1_HEATSYNC].iValue = g_gbiRxData[pebb_addr].rxM4.tempHeatSync;
//		else if(pebb_addr == 2)
//			PRM_PCS[BATT_5].iValue = g_gbiRxData[pebb_addr].rxM4.tempHeatSync;
//		else if(pebb_addr == 3)
//			PRM_PCS[BATT_6].iValue = g_gbiRxData[pebb_addr].rxM4.tempHeatSync;


		break;
	default:
		break;
	}



	// CAN 메일박스에 데이터가 수신되면 해당 CANRMP 레지스터 비트를 클리어.
	ECanaShadow.CANRMP.all = 0;
	ECanaShadow.CANRMP.bit.RMP1 = 1;
	ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Acknowledge interrupt to PIE
}

void CAN_InitMailbox_A(void)
{
	// 0번 MailBox 설정 (Tx)
	ECanaMboxes.MBOX0.MSGID.bit.IDE = 0;			// ID 확장 여부 설정 : 11Bit ID 사용
	ECanaMboxes.MBOX0.MSGID.bit.AAM = 0;			// 응답모드 설정(송신 메일박스만 유효함) : Normal transmit mode
	ECanaMboxes.MBOX0.MSGID.bit.STDMSGID = 0x100;	// Mailbox ID 설정 : CAN2.0A 기준 11Bit ID
	ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 8;			// MailBox Data-Length 설정(최대 8Byte 설정 가능) : 8 Byte
	ECanaMboxes.MBOX0.MDL.all = 0x00000000;			// MailBox 데이터 초기화
	ECanaMboxes.MBOX0.MDH.all = 0x00000000;			// MailBox 데이터 초기화

	// 1번 MailBox 설정 (Rx)
	ECanaMboxes.MBOX1.MSGID.bit.IDE = 0;			// ID 확장 여부 설정 : 11Bit ID 사용
	//ECanbMboxes.MBOX1.MSGID.bit.AME = 0;			// Acceptance Mask 설정 : 사용 안함
	ECanaMboxes.MBOX1.MSGID.bit.AME = 1;			// Acceptance Mask 설정 : 사용함 -> LAM 추가
	// All CANID -> Pass
	ECanaLAMRegs.LAM1.bit.LAMI = 1;					// 1==> : In case of 11bit addr, the first 11bit(28:18) is used.
	ECanaLAMRegs.LAM1.bit.LAM_H = (0x7FF<<2);		// 16:28
	ECanaLAMRegs.LAM1.bit.LAM_L = 0xFFFF;			// In fact, it's ignored.
	ECanaMboxes.MBOX1.MSGID.bit.STDMSGID = 0x200;	// Mailbox ID 설정 : CAN2.0A 기준 11Bit ID
	ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 8;			// MailBox Data-Length 설정(최대 8Byte 설정 가능) : 8 Byte
	ECanaMboxes.MBOX1.MDL.all = 0x00000000;			// MailBox 데이터 초기화
	ECanaMboxes.MBOX1.MDH.all = 0x00000000;			// MailBox 데이터 초기화

	// 메일박스 송수신 설정
	ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
	ECanaShadow.CANMD.bit.MD0 = 0;					// MailBox 0번 : 송신
	ECanaShadow.CANMD.bit.MD1 = 1;					// MailBox 1번 : 수신
	ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;

	// 메일박스 Enable/Disable 설정
	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
	ECanaShadow.CANME.bit.ME0 = 1;					// MailBox 0번 : Enable
	ECanaShadow.CANME.bit.ME1 = 1;					// MailBox 1번 : Enable
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;

	EALLOW;
	// 메일박스 인터럽트 마스크 설정
	ECanaShadow.CANMIM.all = ECanaRegs.CANMIM.all;
	ECanaShadow.CANMIM.bit.MIM1 = 1;				// MailBox 1번 인터럽트 : Enable
	ECanaRegs.CANMIM.all = ECanaShadow.CANMIM.all;

	// 메일박스 인터럽트 라인 설정
	ECanaShadow.CANMIL.all = ECanaRegs.CANMIL.all;
	ECanaShadow.CANMIL.bit.MIL1 = 0;				// MailBox 1번 : 0번 인터럽트 라인
	ECanaRegs.CANMIL.all = ECanaShadow.CANMIL.all;

	// 인터럽트 라인 Enable/Disable 설정
	ECanaShadow.CANGIM.all = ECanaRegs.CANGIM.all;
	ECanaShadow.CANGIM.bit.I0EN = 1;				// 0번 인터럽트 라인 : Enable
	ECanaRegs.CANGIM.all = ECanaShadow.CANGIM.all;
	EDIS;

	// for test
//    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
//    ECanaShadow.CANMC.bit.STM = 1;    // Configure CAN for self-test mode
//    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

	PieCtrlRegs.PIEIER9.bit.INTx5 = 1;				// PIE 인터럽트(ECAN0INTA) Enable
	//PieCtrlRegs.PIEIER9.bit.INTx7 = 1;				// PIE 인터럽트(ECAN0INTB) Enable
	IER |= M_INT9; 									// CPU 인터럽트(INT9) Enable
}
