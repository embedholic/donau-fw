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

/* Task 에서 tx char 가 퍼버에 존재 하면 txReady 확인 후 버퍼에서 데이터를 가져와
 * sendChar로 전송하도록 해야 한다. */
public Bool SCI_haveTxBufferChar(Uns ID);	/* 전송 버퍼의 데이터 유무 확인 */
public Bool SCI_txReady(Uns ID);			/* SCI 쓰기 가능 상태 리턴 */
public Bool SCI_getTxBufferChar(Uns ID,char* ch); /* 전송 버퍼에서 데이터 가져오기 */
public Bool SCI_sendChar(Uns ID,char ch); 	/* SCI 에 char 전송 */

/* Application에서 데이터를 전송할 경우 아래의 코드를 사용한다. */
public Bool SCI_write(Uns ID,char ch);		/* SCI 전송 버퍼에 데이터 쓰기 */
public Bool SCI_writeString(Uns ID, char* string, unsigned short size);
public Bool SCI_read(Uns ID,char* ch);		/* SCI 수신 버퍼의 데이터 가져오기 */
public Bool SCI_readn(Uns ID,char* buf, unsigned short* size);
public Bool SCI_haveRxBufferChar(Uns ID);	/* 수신 버퍼에 데이터 유무 확인 */

public void SendFlush(); /* 송신 버퍼의 모든 데이터를 write한다. (주의: 작업 완료시 까지 리턴하지 않음.) */

#endif /* SCI_H_ */
