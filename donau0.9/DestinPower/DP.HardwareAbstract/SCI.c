/*
 * SCI.c
 *
 *  Created on: 2012. 11. 15.
 *      Author: destinPower
 */

#include "SCI.h"
#include "DSP28x_Project.h"

#if DEBUG_MODE == 1
#include "DBUG.h"
#endif

//FIXME SCI�� �߰��� ��� DEFINE(SCI_REGS)�� ������� �ʵ��� �����ؾ� �Ѵ�.
#if USE_SCI_MODE == 1
#define SCI_REGS SciaRegs
#else
#define SCI_REGS ScibRegs
#endif

CIRC_Obj circSCI_RXBUF[SCI_COUNT];
CIRC_Obj circSCI_TXBUF[SCI_COUNT];
char ARRAY_CIRC_RX_BUFFER[SCI_RXTX_BUF_SIZE];
char ARRAY_CIRC_TX_BUFFER[SCI_RXTX_BUF_SIZE];

//-private Bool SCI_getRxBufferChar(Uns ID,char* ch);


public void SCI_create(Uns ID, Uns BaudID,Uns nWaitTime)
{
 	CIRC_new(&circSCI_RXBUF[ID],ARRAY_CIRC_RX_BUFFER,SCI_RXTX_BUF_SIZE);
	CIRC_new(&circSCI_TXBUF[ID],ARRAY_CIRC_TX_BUFFER,SCI_RXTX_BUF_SIZE);

	if ( ID == SCI_RS232 )
	{
		SCI_REGS.SCICTL1.bit.SWRESET = 0;		// SCI ����Ʈ���� ����
		SCI_REGS.SCICCR.bit.SCICHAR = 7;		// SCI �ۼ��� Charcter-length ���� : 8bit
		SCI_REGS.SCICCR.bit.LOOPBKENA = 0;		// SCI ������ �׽�Ʈ ��� Enable
		SCI_REGS.SCICTL1.bit.RXENA = 1;			// SCI ���� ��� Enable
		SCI_REGS.SCICTL1.bit.TXENA = 1;			// SCI �۽� ��� Enable
		SCI_REGS.SCICTL2.bit.TXINTENA = 1;		// SCI �۽� ���ͷ�Ʈ Disable(1) Enable(0)
		SCI_REGS.SCICTL2.bit.RXBKINTENA = 1;	// SCI ���� ���ͷ�Ʈ Enable

		SCI_setBaudRate(ID,BaudID);

		SCI_REGS.SCIPRI.bit.FREE = 1;			// SCI ���ķ��̼� ���� ���
		SCI_REGS.SCICTL1.bit.SWRESET = 1;		// SCI ����Ʈ���� ���� ����
	}
}

public CIRC_Handle SCI_getCircRxBuffer(Uns ID)
{
	return &circSCI_RXBUF[ID];
}
public CIRC_Handle SCI_getCircTxBuffer(Uns ID)
{
	return &circSCI_TXBUF[ID];
}

public void SCI_setBaudRate(Uns ID, Uns BaudID)
{
	if ( ID == SCI_RS232 )             // LSPCLK/(BRR +1) x 8
	{
		switch (BaudID)
	   	{
	   		case SCI_BAUD_3:
	   		    //COM_9600
	   			SCI_REGS.SCIHBAUD = 0x000F;
	   			SCI_REGS.SCILBAUD = 0x0041;

				break;
	   		case SCI_BAUD_4:
	   		    //COM_19200
	   			SCI_REGS.SCIHBAUD = 0x0007;
	   			SCI_REGS.SCILBAUD = 0x00A0;

				break;
	   		case SCI_BAUD_5:
	   		    //COM_38400;
	   			SCI_REGS.SCIHBAUD = 0x0003;
	   			SCI_REGS.SCILBAUD = 0x00CF;

				break;
	   		case SCI_BAUD_6:
	   		    //COM_57600;
	   			SCI_REGS.SCIHBAUD = 0x0002;
	   			SCI_REGS.SCILBAUD = 0x008A;

				break;
			case SCI_BAUD_7:
	   		    //COM_115200;
				SCI_REGS.SCIHBAUD = 0x0001; // ���� 8��Ʈ
				SCI_REGS.SCILBAUD = 0x0044; // ���� 8��Ʈ

				break;
		}
	}
}

public Bool SCI_sendChar(Uns ID,char ch)
{
	if ( ID == SCI_RS232 )
	{
		SCI_REGS.SCITXBUF = ch;
	}

	return TRUE;
}
public Bool SCI_txReady(Uns ID)
{
	//IC CHECK
	return ( SCI_REGS.SCICTL2.bit.TXRDY );
}


/*
 * CIRC Buffering
 */
//private Bool SCI_getRxBufferChar(Uns ID,char* ch)
//{
//	*ch=CIRC_readChar(&circSCI_RXBUF[ID]);
//	return TRUE;
//}
public Bool SCI_getTxBufferChar(Uns ID,char* ch)
{
	*ch=CIRC_readChar(&circSCI_TXBUF[ID]);
	return TRUE;
}

public Bool SCI_write(Uns ID,char ch)
{
	return CIRC_writeChar(&circSCI_TXBUF[ID], ch);
}

public Bool SCI_writeString(Uns ID, char* string, unsigned short size)
{
	return CIRC_write(&circSCI_TXBUF[ID],string, &size);
}
public Bool SCI_read(Uns ID,char* ch)
{
	if( !SCI_haveRxBufferChar(ID) )
		return FALSE;

	*ch=CIRC_readChar(&circSCI_RXBUF[ID]);
	return TRUE;
}

public Bool SCI_readn(Uns ID,char* buf,  unsigned short* size)
{
	return CIRC_read(&circSCI_RXBUF[ID], buf, size);
}

public Bool SCI_haveRxBufferChar(Uns ID)
{
	return 	!(CIRC_isEmpty(&circSCI_RXBUF[ID]));

}
public Bool SCI_haveTxBufferChar(Uns ID)
{
	return 	!(CIRC_isEmpty(&circSCI_TXBUF[ID]));
}

public void SendFlush()
{
	char ch1;
	   while(SCI_haveTxBufferChar(SCI_RS232))
	   {
			if(SCI_txReady(SCI_RS232))
			{
				SCI_getTxBufferChar(SCI_RS232, &ch1);
				SCI_sendChar(SCI_RS232,ch1);
			}
	   }
}


#if 0
int bufrx[100];
int idxrx = 0;
#endif

interrupt void SCIRXINTA_ISR(void)     // SCI-A - RX
{
	static char ch;

#if DEBUG_MODE == 1
//	static char count = 0;
//	count++;
//	if(count > 2 )
//	{
//		TEST_LedToggle(14);
//		count = 0;
//	}

#endif
	//FIXME SCI�� �߰��� ��� ���ͷ�Ʈ ��ƾ�� DEFINE(SCI_REGS)�� ������� �ʵ��� �����ؾ� �Ѵ�.
	ch =SCI_REGS.SCIRXBUF.bit.RXDT;
	CIRC_writeChar(&circSCI_RXBUF[SCI_RS232],ch);

//	if(idxrx > 99)
//		idxrx=0;
//	bufrx[idxrx++] = ch;

	SCI_REGS.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
	SCI_REGS.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
	PieCtrlRegs.PIEACK.bit.ACK9 = ON;    // Issue PIE ack, INT9
}

interrupt void SCITXINTA_ISR(void)     // SCI-A - TX
{
#if DEBUG_MODE == 1
//	static char count = 0;
//	count++;
//	if(count > 2 )
//	{
//		TEST_LedToggle(15);
//		count = 0;
//	}

#endif
	//FIXME SCI�� �߰��� ��� ���ͷ�Ʈ ��ƾ�� DEFINE(SCI_REGS)�� ������� �ʵ��� �����ؾ� �Ѵ�.
	SCI_REGS.SCIFFTX.bit.TXFFINTCLR = 1; // Clear SCI Interrupt flag
	PieCtrlRegs.PIEACK.bit.ACK9=ON;      // Issue PIE ACK
}
