/*
 * CAN.h
 *	Communication with Master BMS via eCAN
 *  Created on: 2013. 5. .
 *      Author: destinPower
 */
#include <xdc/std.h>
#include "CommData.h"

void CAN_Create(void);
void CAN_InitVar(void);
void CAN_InitMailbox(void);
//void ISR_CAN_A_Rx(void);
void ISR_CAN_B_Rx(void);
void CAN_proc(void);
void CAN_Timer_5ms(void);
int txFrameEx(UInt16 a, UInt16 b, UInt16 c, UInt16 d, UInt16 e, UInt16 f, UInt16 g, UInt16 h );

typedef struct
{
	union
	{
		float		f32Data;
		uint16_t	ui16Data[2];
	}UNI_VAR;
}UNI_VAR_DATA;


typedef struct
{
	uint16_t	msgid;
	uint16_t 	data[8];
//	uint16_t 	rxflag;
} CAN_RX_FRAME;

typedef struct
{
	uint16_t	c0tick_ms;		uint16_t	c0period_ms;
	uint16_t	c1tick_ms;		uint16_t	c1period_ms;
} ST_CAN_TX_STATUS;


