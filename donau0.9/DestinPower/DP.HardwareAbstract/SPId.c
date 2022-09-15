/*
 * SPId.c (Digital Out)
 *
 *  Created on: 2013. 1. 22.
 *      Author: destinPower
 */

#include "SPId.h"
#include "DSP28x_Project.h"						// Device Headerfile and Examples Include File

void DO_CS_A_SET(){ DO_CS_A = 0; }
void DO_CS_A_CLE(){ DO_CS_A = 1; }
void DO_CS_B_SET(){ DO_CS_B = 0; }
void DO_CS_B_CLE(){ DO_CS_B = 1; }
void DI_CS_A_SET() { DI_CS_A = 0; }
void DI_CS_A_CLE() { DI_CS_A = 1; }
void DI_CS_B_SET() { DI_CS_B = 0; }
void DI_CS_B_CLE() { DI_CS_B = 1; }
void DI_CS_C_SET() { DI_CS_C = 0; }
void DI_CS_C_CLE() { DI_CS_C = 1; }

public void SPID_init()
{
	//default DO setting
	SpidRegs.SPICCR.bit.SPISWRESET=0;				// Reset SPI

	SpidRegs.SPICCR.bit.CLKPOLARITY = 0;			// 1:Falling EDGE 0:Rising EDGE
													// 0���� ���� �� DI A,B,C, DO A ����. �׷��� DO-B�� ���� ���� ����.
													// DI/DO�� ���� Polarity�� �޶� ���� �� �� ������.
	SpidRegs.SPICTL.bit.CLK_PHASE  = 0;

	SpidRegs.SPICCR.bit.SPICHAR = 7;				// 8bit ������ ũ�� ���
	SpidRegs.SPICTL.bit.MASTER_SLAVE = 1;			// ������ ���
	SpidRegs.SPICTL.bit.TALK = 1;					// ��� Ȱ��ȭ

	SpidRegs.SPIBRR = 59;				// SPI ��żӵ� ����: 300MHz/(SPIBRR+1) = 5Mhz
	SpidRegs.SPIPRI.bit.FREE = 1;
	SpidRegs.SPICCR.bit.SPISWRESET=1;

	//-SPID_Init_DO();
	DO_CS_A_CLE();
	DO_CS_B_CLE();
	DI_CS_A_CLE();
	DI_CS_B_CLE();
	DI_CS_C_CLE();
}
