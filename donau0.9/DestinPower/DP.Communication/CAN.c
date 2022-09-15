/*
 * CAN.c
 *	Communication with Master BMS via eCAN
 *  Created on: 2013. 5. .
 *      Author: destinPower
 */
//#include <stdlib.h>
#include "CAN.h"
#include "string.h"
#include "CommData.h"
#include "DSP2834x_Device.h"
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>
#include "CC.h"
#include "Fault.h"
#include "PRM_PCS.h"
#include "CTRL_GEN.h"
#include "MCCB.h"
#include "EVE_DC_CHARGER.h"

//#include "DSP2834x_Examples.h"
#define CPU_RATE    3.333L     // for 300MHz CPU clock speed (SYSCLKOUT)
#define CPU_FRQ_300MHZ    1
// DO NOT MODIFY THIS LINE.
#define DELAY_US(A)  DSP28x_usDelay(((((long double) A * 1000.0L) / (long double)CPU_RATE) - 9.0L) / 5.0L)
#define DELAY_NS(A)  DSP28x_usDelay(((((long double) A) / (long double)CPU_RATE) - 9.0L) / 5.0L)
#if DSP28_ECANA
extern void InitECan(void);
extern void InitECanb(void);
extern void InitECanGpio(void);
extern void InitECanbGpio(void);
#endif // endif DSP28_ECANA


static ST_CAN_TX_STATUS m_stTxStatus;
static CAN_RX_FRAME	m_stRxFrame;

// Global variables used in this system
struct ECAN_REGS ECanbShadow;

void CAN_Create(void)
{
	// ECAN-B : External (PMS)
	InitECanbGpio();					// GPIO�� CAN ��� ������ ����
	InitECanb();						// CAN �ʱ�ȭ : CAN2.0A ���, 500kbps �ӵ�

	CAN_InitMailbox();
	CAN_InitVar();

	// Vector Remapping
	EALLOW;
	PieVectTable.ECAN0INTB = &ISR_CAN_B_Rx;
	EDIS;
}

void CAN_InitVar(void)
{
	memset(&(m_stTxStatus), 0, sizeof(m_stTxStatus));
	m_stTxStatus.c0period_ms	= 10;		//tx per XXperiod_ms
	m_stTxStatus.c1period_ms	= 1000;
    m_stTxStatus.c0tick_ms	= 0;
    m_stTxStatus.c1tick_ms	= 0;



}

void CAN_InitMailbox(void)
{
	// 0�� MailBox ���� (Tx)
	ECanbMboxes.MBOX0.MSGID.bit.IDE = 0;			// ID Ȯ�� ���� ���� : 11Bit ID ���
	ECanbMboxes.MBOX0.MSGID.bit.AAM = 0;			// ������ ����(�۽� ���Ϲڽ��� ��ȿ��) : Normal transmit mode
	ECanbMboxes.MBOX0.MSGID.bit.STDMSGID = 0x10;	// Mailbox ID ���� : CAN2.0A ���� 11Bit ID
	ECanbMboxes.MBOX0.MSGCTRL.bit.DLC = 8;			// MailBox Data-Length ����(�ִ� 8Byte ���� ����) : 8 Byte
	ECanbMboxes.MBOX0.MDL.all = 0x00000000;			// MailBox ������ �ʱ�ȭ
	ECanbMboxes.MBOX0.MDH.all = 0x00000000;			// MailBox ������ �ʱ�ȭ

	// 1�� MailBox ���� (Rx)
	ECanbMboxes.MBOX1.MSGID.bit.IDE = 0;			// ID Ȯ�� ���� ���� : 11Bit ID ���

	//ECanbMboxes.MBOX1.MSGID.bit.AME = 0;			// Acceptance Mask ���� : ��� ����
	ECanbMboxes.MBOX1.MSGID.bit.AME = 1;			// Acceptance Mask ���� : ����� -> LAM �߰�
	// All CANID -> Pass
	ECanbLAMRegs.LAM1.bit.LAMI = 1;					// 1==> : In case of 11bit addr, the first 11bit(28:18) is used.
	ECanbLAMRegs.LAM1.bit.LAM_H = (0x7FF<<2);		// 16:28
	ECanbLAMRegs.LAM1.bit.LAM_L = 0xFFFF;			// In fact, it's ignored.
	ECanbMboxes.MBOX1.MSGID.bit.STDMSGID = 0x555;	// Mailbox ID ���� : CAN2.0A ���� 11Bit ID
	ECanbMboxes.MBOX1.MSGCTRL.bit.DLC = 8;			// MailBox Data-Length ����(�ִ� 8Byte ���� ����) : 8 Byte
	ECanbMboxes.MBOX1.MDL.all = 0x00000000;			// MailBox ������ �ʱ�ȭ
	ECanbMboxes.MBOX1.MDH.all = 0x00000000;			// MailBox ������ �ʱ�ȭ

	// ���Ϲڽ� �ۼ��� ����
	ECanbShadow.CANMD.all = ECanbRegs.CANMD.all;
	ECanbShadow.CANMD.bit.MD0 = 0;					// MailBox 0�� : �۽�
	ECanbShadow.CANMD.bit.MD1 = 1;					// MailBox 1�� : ����
	ECanbRegs.CANMD.all = ECanbShadow.CANMD.all;

	// ���Ϲڽ� Enable/Disable ����
	ECanbShadow.CANME.all = ECanbRegs.CANME.all;
	ECanbShadow.CANME.bit.ME0 = 1;					// MailBox 0�� : Enable
	ECanbShadow.CANME.bit.ME1 = 1;					// MailBox 1�� : Enable
	ECanbRegs.CANME.all = ECanbShadow.CANME.all;

	EALLOW;
	// ���Ϲڽ� ���ͷ�Ʈ ����ũ ����
	ECanbShadow.CANMIM.all = ECanbRegs.CANMIM.all;
	ECanbShadow.CANMIM.bit.MIM1 = 1;				// MailBox 1�� ���ͷ�Ʈ : Enable
	ECanbRegs.CANMIM.all = ECanbShadow.CANMIM.all;

	// ���Ϲڽ� ���ͷ�Ʈ ���� ����
	ECanbShadow.CANMIL.all = ECanbRegs.CANMIL.all;
	ECanbShadow.CANMIL.bit.MIL1 = 0;				// MailBox 1�� : 0�� ���ͷ�Ʈ ����
	ECanbRegs.CANMIL.all = ECanbShadow.CANMIL.all;

	// ���ͷ�Ʈ ���� Enable/Disable ����
	ECanbShadow.CANGIM.all = ECanbRegs.CANGIM.all;
	ECanbShadow.CANGIM.bit.I0EN = 1;				// 0�� ���ͷ�Ʈ ���� : Enable
	ECanbRegs.CANGIM.all = ECanbShadow.CANGIM.all;
	EDIS;

	//PieCtrlRegs.PIEIER9.bit.INTx5 = 1;			// PIE ���ͷ�Ʈ(ECAN0INTA) Enable
	PieCtrlRegs.PIEIER9.bit.INTx7 = 1;				// PIE ���ͷ�Ʈ(ECAN0INTB) Enable
	IER |= M_INT9; 									// CPU ���ͷ�Ʈ(INT9) Enable
}



int debug_rx_cnt = 0;

// ISR���� System_printf Ȥ�� flush ȣ�� ���� ����.
interrupt void ISR_CAN_B_Rx(void)
{
	//-int slave_mbms_addr, offset;
	volatile Int16 iTemp, temp;
	debug_rx_cnt++;

#if 1
	m_stRxFrame.msgid = ECanbMboxes.MBOX1.MSGID.bit.STDMSGID;

	if( m_stRxFrame.msgid != 0x110 )
	{
		// CAN ���Ϲڽ��� �����Ͱ� ���ŵǸ� �ش� CANRMP �������� ��Ʈ�� Ŭ����.
		ECanbShadow.CANRMP.all = 0;
		ECanbShadow.CANRMP.bit.RMP1 = 1;
		ECanbRegs.CANRMP.all = ECanbShadow.CANRMP.all;

		PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Acknowledge interrupt to PIE
		return;
	}
#endif

	m_stRxFrame.data[0] = (ECanbMboxes.MBOX1.MDL.byte.BYTE0 & 0xFF); // EVE ID
	m_stRxFrame.data[1] = (ECanbMboxes.MBOX1.MDL.byte.BYTE1 & 0xFF); // Identity
	m_stRxFrame.data[2] = (ECanbMboxes.MBOX1.MDL.byte.BYTE2 & 0xFF); //
	m_stRxFrame.data[3] = (ECanbMboxes.MBOX1.MDL.byte.BYTE3 & 0xFF); //
	m_stRxFrame.data[4] = (ECanbMboxes.MBOX1.MDH.byte.BYTE4 & 0xFF);
	m_stRxFrame.data[5] = (ECanbMboxes.MBOX1.MDH.byte.BYTE5 & 0xFF);
	m_stRxFrame.data[6] = (ECanbMboxes.MBOX1.MDH.byte.BYTE6 & 0xFF);
	m_stRxFrame.data[7] = (ECanbMboxes.MBOX1.MDH.byte.BYTE7 & 0xFF);

	// ID �˻� skip

	if( m_stRxFrame.data[1] == 0x14 )
	{
		//EVE.
		temp = MAKE_16BIT( m_stRxFrame.data[2], m_stRxFrame.data[3] );

		EVE.bStatus = temp & 0x1;
		EVE.bFaultStatus = (temp & 0x2) >> 1;
		EVE.bInitCharge = (temp & 0x4) >> 2;

	//  EVE.bStatus = m_stRxFrame.data[2] & 0x1;
	//  EVE.bFaultStatus = (m_stRxFrame.data[2] & 0x2) >> 1;
	//  EVE.bInitCharge = (m_stRxFrame.data[3] & 0x4) >> 2;

		//EVE.Charge_Power = m_stRxFrame.data[3];
		EVE.Charge_Power = MAKE_16BIT(m_stRxFrame.data[4],m_stRxFrame.data[5]) * 0.1;
		EVE.bCAN_Fail_RxChekbit = m_stRxFrame.data[7] & 0x1;
		//...
	}

//		iTemp = ( m_stRxFrame.data[4] << 8 ) | (m_stRxFrame.data[5] & 0xFF);


	// CAN ���Ϲڽ��� �����Ͱ� ���ŵǸ� �ش� CANRMP �������� ��Ʈ�� Ŭ����.
	ECanbShadow.CANRMP.all = 0;
	ECanbShadow.CANRMP.bit.RMP1 = 1;
	ECanbRegs.CANRMP.all = ECanbShadow.CANRMP.all;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Acknowledge interrupt to PIE
}


int txFrameEx(UInt16 a, UInt16 b, UInt16 c, UInt16 d, UInt16 e, UInt16 f, UInt16 g, UInt16 h )
{
	int Status = 0;
	int waitcnt=0;


	//ECanbMboxes.MBOX0.MSGID.bit.STDMSGID = 0x555; // June Added. �׷��� ���⿡�� ���� ���� ������� ����..
	ECanbMboxes.MBOX0.MDL.byte.BYTE0 = (a 	& 0x00FF) >> 0;
	ECanbMboxes.MBOX0.MDL.byte.BYTE1 = (b 	& 0x00FF) >> 0;
	ECanbMboxes.MBOX0.MDL.byte.BYTE2 = (c 	& 0x00FF) >> 0;
	ECanbMboxes.MBOX0.MDL.byte.BYTE3 = (d 	& 0x00FF) >> 0;
	ECanbMboxes.MBOX0.MDH.byte.BYTE4 = (e 	& 0x00FF) >> 0;
	ECanbMboxes.MBOX0.MDH.byte.BYTE5 = (f 	& 0x00FF) >> 0;
	ECanbMboxes.MBOX0.MDH.byte.BYTE6 = (g 	& 0x00FF) >> 0;
	ECanbMboxes.MBOX0.MDH.byte.BYTE7 = (h 	& 0x00FF) >> 0;

	// ���Ϲڽ� ������ ���� ��û
	ECanbShadow.CANTRS.all = 0;
	ECanbShadow.CANTRS.bit.TRS0 = 1;
	ECanbRegs.CANTRS.all = ECanbShadow.CANTRS.all;

	// ���Ϲڽ� ������ �۽� �Ϸ� �÷��� ���
	do{
		ECanbShadow.CANTA.all = ECanbRegs.CANTA.all;
		Task_sleep(1);
		if((waitcnt++)>20){
			// RESET
			waitcnt=0;
			Status = -1; // send time out
			break;
		}
	}while(!ECanbShadow.CANTA.bit.TA0);

	// ���Ϲڽ� ������ �۽� �Ϸ� �÷��� �ʱ�ȭ
	ECanbShadow.CANTA.all = 0;
	ECanbShadow.CANTA.bit.TA0 = 1;
	ECanbRegs.CANTA.all = ECanbShadow.CANTA.all;

	return Status;
}

unsigned int debug_tx_cnt = 0;
#define EQ_SNT_DATA_CHK(a) sent_succ_tempByte[a] != tempByte[a]
#define EQ_SNT_PACK() EQ_SNT_DATA_CHK(0) || EQ_SNT_DATA_CHK(1) || EQ_SNT_DATA_CHK(2) || EQ_SNT_DATA_CHK(3) || EQ_SNT_DATA_CHK(4) || EQ_SNT_DATA_CHK(5) /* ... ~7 */
void txFrame(int type)
{
	int tempByte[8] = {0};
	static int sent_succ_tempByte[6] = {-1,-1, };
	static int forceTxCnt = 0;

	if(forceTxCnt++ >= 100)
	{
		forceTxCnt = 0;
		sent_succ_tempByte[0] = -1;
	}

	switch(type)
	{
	case 0:
		tempByte[0] = (EVE.bCMD_On & 0x1);
		tempByte[0] |= (EVE.bCMD_FaultReset & 0x1) << 1;
		tempByte[0] |= (EVE.bCMDCharge & 0x1) << 2;
		tempByte[0] |= (EVE.bCAN_Fail_TxChekbit & 0x1) << 3;

		tempByte[1] = (EVE.bMC8_OnOff & 0x1);
		tempByte[1] |= (EVE.bMC8_Status & 0x1) << 1;
		tempByte[1] |= (EVE.bMC9_OnOff & 0x1) << 2;
		tempByte[1] |= (EVE.bMC9_Status & 0x1) << 3;

		tempByte[2] = (PRM_PCS[EVE_MAXIMUM_CHARGE_VOLTAGE].iValue >> 8) & 0xFF;
		tempByte[3] = (PRM_PCS[EVE_MAXIMUM_CHARGE_VOLTAGE].iValue) & 0xFF;

		tempByte[4] = (PRM_PCS[EVE_START_CHARGE_VOLTAGE].iValue >> 8) & 0xFF;
		tempByte[5] = (PRM_PCS[EVE_START_CHARGE_VOLTAGE].iValue) & 0xFF;

		if( EQ_SNT_PACK() )
		{
			if( txFrameEx(0xFF,0x0,tempByte[0],tempByte[1],tempByte[2],tempByte[3],tempByte[4],tempByte[5]) == 0 )
			{
				debug_tx_cnt++;

				// send succ .
				sent_succ_tempByte[0] = tempByte[0];
				sent_succ_tempByte[1] = tempByte[1];
				sent_succ_tempByte[2] = tempByte[2];
				sent_succ_tempByte[3] = tempByte[3];
				sent_succ_tempByte[4] = tempByte[4];
				sent_succ_tempByte[5] = tempByte[5];

				EVE.bCMD_FaultReset = FALSE; // Toggle type command
			}
		}
		break;
	case 1:
		break;
	}

}
void addTxTick(ST_CAN_TX_STATUS *pstTxStatus, int acc)
{
	if(pstTxStatus->c0tick_ms < pstTxStatus->c0period_ms) pstTxStatus->c0tick_ms += acc;
	if(pstTxStatus->c1tick_ms < pstTxStatus->c1period_ms) pstTxStatus->c1tick_ms += acc;
}


// 5ms timer : �ٸ� ������ ȣ�� ������� �Ѵ�.
// Call : c_int25
void CAN_Timer_5ms(void)
{
	addTxTick(&m_stTxStatus, 5);
}

// Call : CAN_Tx_proc
void check_n_txFrame(uint16_t *tick, uint16_t period, int type)
{
	if( (*tick != 0) && (period != 0) && (*tick >= period))
	{
		txFrame(type);
		*tick = 0;
	}
}

// Call : CAN_proc
void CAN_Tx_proc(ST_CAN_TX_STATUS *pstTxStatus)
{
	check_n_txFrame(&(pstTxStatus->c0tick_ms),	pstTxStatus->c0period_ms, 0);
	check_n_txFrame(&(pstTxStatus->c1tick_ms),	pstTxStatus->c1period_ms, 1);
}

// it should be in a while loop
// Call : _task_CAN
void CAN_proc(void)
{
#if 1
	CAN_Tx_proc(&m_stTxStatus);
#endif

}
