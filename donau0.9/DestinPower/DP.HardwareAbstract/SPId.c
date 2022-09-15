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
													// 0으로 설정 시 DI A,B,C, DO A 동작. 그러나 DO-B는 동작 하지 않음.
													// DI/DO에 따라 Polarity가 달라서 동작 전 재 설정함.
	SpidRegs.SPICTL.bit.CLK_PHASE  = 0;

	SpidRegs.SPICCR.bit.SPICHAR = 7;				// 8bit 데이터 크기 사용
	SpidRegs.SPICTL.bit.MASTER_SLAVE = 1;			// 마스터 모드
	SpidRegs.SPICTL.bit.TALK = 1;					// 통신 활성화

	SpidRegs.SPIBRR = 59;				// SPI 통신속도 설정: 300MHz/(SPIBRR+1) = 5Mhz
	SpidRegs.SPIPRI.bit.FREE = 1;
	SpidRegs.SPICCR.bit.SPISWRESET=1;

	//-SPID_Init_DO();
	DO_CS_A_CLE();
	DO_CS_B_CLE();
	DI_CS_A_CLE();
	DI_CS_B_CLE();
	DI_CS_C_CLE();
}
