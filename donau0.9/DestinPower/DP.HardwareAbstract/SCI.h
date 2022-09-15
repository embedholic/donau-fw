/*
 * SCI.h
 *
 *  Created on: 2012. 11. 15.
 *      Author: destinPower
 */

#ifndef SCI_H_
#define SCI_H_

#include "LGS_Common.h"
#include "CIRC.h"

typedef enum
{
	SCI_RS232,
	//-UARTB_RS485
	SCI_COUNT=1
} UartID;

#define SCI_BAUD_3     3        /* 9600 baud */
#define SCI_BAUD_4     4        /* 19200 baud */
#define SCI_BAUD_5     5        /* 38400 baud */
#define SCI_BAUD_6     6        /* 56000 baud */
#define SCI_BAUD_7     7        /* 115200 baud */

#define	SCI_RXTX_BUF_SIZE 	256

extern CIRC_Obj		circSCI_RXBUF[SCI_COUNT];
extern CIRC_Obj		circSCI_TXBUF[SCI_COUNT];

public void SCI_create(Uns ID, Uns BaudID,Uns nWaitTime);
public void SCI_setBaudRate(Uns ID, Uns BaudID);
public CIRC_Handle SCI_getCircRxBuffer(Uns ID);
public CIRC_Handle SCI_getCircTxBuffer(Uns ID);

/* Task ���� tx char �� �۹��� ���� �ϸ� txReady Ȯ�� �� ���ۿ��� �����͸� ������
 * sendChar�� �����ϵ��� �ؾ� �Ѵ�. */
public Bool SCI_haveTxBufferChar(Uns ID);	/* ���� ������ ������ ���� Ȯ�� */
public Bool SCI_txReady(Uns ID);			/* SCI ���� ���� ���� ���� */
public Bool SCI_getTxBufferChar(Uns ID,char* ch); /* ���� ���ۿ��� ������ �������� */
public Bool SCI_sendChar(Uns ID,char ch); 	/* SCI �� char ���� */

/* Application���� �����͸� ������ ��� �Ʒ��� �ڵ带 ����Ѵ�. */
public Bool SCI_write(Uns ID,char ch);		/* SCI ���� ���ۿ� ������ ���� */
public Bool SCI_writeString(Uns ID, char* string, unsigned short size);
public Bool SCI_read(Uns ID,char* ch);		/* SCI ���� ������ ������ �������� */
public Bool SCI_readn(Uns ID,char* buf, unsigned short* size);
public Bool SCI_haveRxBufferChar(Uns ID);	/* ���� ���ۿ� ������ ���� Ȯ�� */

public void SendFlush(); /* �۽� ������ ��� �����͸� write�Ѵ�. (����: �۾� �Ϸ�� ���� �������� ����.) */

#endif /* SCI_H_ */
