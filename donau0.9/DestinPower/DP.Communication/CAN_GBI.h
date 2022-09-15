/*
 * CAN_GBI.h
 *
 *  Created on: 2013. 9. 5.
 *      Author: destinPower
 */

#ifndef CAN_GBI_H_
#define CAN_GBI_H_

#include "LGS_Common.h"

typedef enum  {
	NONE,
	M_BOX1
}GBI_CAN_MSG;

#if 1 //-0
typedef struct
{
	uint16_t	mb0tick_ms;		uint16_t	mb0period_ms;
} GBI_CAN_TX_STATUS;
#endif

typedef struct
{
	union
	{
		uint16_t	c0_0u;
		struct
		{
			uint16_t		FAULT_RESET		: 1;
			uint16_t		GATE_ONOFF  	: 1;
			uint16_t		MC4_ONOFF      	: 1;
			uint16_t		MANUAL_OP  		: 1;
			uint16_t		MO_STATE        : 1;
			uint16_t		rev5        	: 1;
			uint16_t		rev6        	: 1;
			uint16_t		rev7        	: 1;

			uint16_t		rev0_0        	: 1;
			uint16_t		rev1_1        	: 1;
			uint16_t		rev2_2        	: 1;
			uint16_t		rev3_3        	: 1;
			uint16_t		rev4_4        	: 1;
			uint16_t		rev5_5        	: 1;
			uint16_t		rev6_6        	: 1;
			uint16_t		rev7_7        	: 1;
		}BIT;
	}C0_0U;
	union
	{
		uint16_t	c0_1u; //<-uint8_t
		struct
		{
			uint16_t		FanSpeed			: 8; //<-uint8_t

			uint16_t		rev0_0        	: 1;
			uint16_t		rev1_1        	: 1;
			uint16_t		rev2_2        	: 1;
			uint16_t		rev3_3        	: 1;
			uint16_t		rev4_4        	: 1;
			uint16_t		rev5_5        	: 1;
			uint16_t		rev6_6        	: 1;
			uint16_t		rev7_7        	: 1;
		}BIT;
	}C0_1U;
	uint16_t	pebbId; //<-uint8_t
	uint16_t	MoCtrlDelay; //<-uint8_t

} GBI_COMM_C0;

typedef struct
{
	uint16_t pebbID;
	uint16_t currentR;
	uint16_t currentS;
	uint16_t currentT;
} GBI_COMM_RX_MSG_M1;

typedef struct
{
	uint16_t currentR_rect;
	uint16_t currentS_rect;
	uint16_t currentT_rect;

} GBI_COMM_RX_MSG_M2;

typedef struct
{
	uint16_t tempR;
	uint16_t tempS;
	uint16_t tempT;
}GBI_COMM_RX_MSG_M3;


typedef struct
{
	uint16_t tempHeatSync;
	uint16_t vdc;
	uint16_t rev;
}GBI_COMM_RX_MSG_M4;

typedef struct
{
	union
	{
		uint16_t byte16;
		struct
		{
			uint16_t		EMERGENCY  			: 1;
			uint16_t		MC_STATE   			: 1;
			uint16_t		TOTAL_GATE_STATE 	: 1;
			uint16_t		GATE_MUX 			: 1;
			uint16_t		MASTER_GATE 		: 1;
			uint16_t		IGBT_R_FAULT 		: 1;
			uint16_t		IGBT_S_FAULT 		: 1;
			uint16_t		IGBT_T_FAULT 		: 1;

			uint16_t		IGBT_OVERTEMP		: 1;
			uint16_t		IGBT_FAN_FAULT		: 1;
			uint16_t		IGBT_FUSE_FAULT		: 1;
			uint16_t		IGBT_MC_FAULT		: 1; // 정확히는 MC OFF FAULT
			uint16_t		PARAMETER_ERR		: 1; // 파라미터가 초기화 되었을 경우.
			uint16_t		HEATSYNC_OT       	: 1;
			uint16_t		REV        			: 2;
		}BIT;
	}STATE;
	uint16_t rev1;
	uint16_t rev2;
}GBI_COMM_RX_MSG_M0;

typedef struct
{
	GBI_COMM_C0 C0;
	uint16_t bTxNew; // 주기적 Send 함수 내에서 이 값이 1일 경우에만 send한다.
}GBI_COMM_TX_DATA;

typedef struct
{
	GBI_COMM_RX_MSG_M0 rxM0; //0x200
	GBI_COMM_RX_MSG_M1 rxM1; // 0x201
	GBI_COMM_RX_MSG_M2 rxM2; // 0x202
	GBI_COMM_RX_MSG_M3 rxM3; // 0x203
	GBI_COMM_RX_MSG_M4 rxM4; // 0x204
}GBI_COMM_RX_DATA;

void CAN_GBI_Create();
void CAN_GBI_proc(void);
void CAN_GBI_ManualOperation(int OnOff);
void CAN_GBI_FanSpeed(int speed);
void CAN_GbiTimer_5ms(void);
void CAN_GBI_AddInfoNode(void);
int CAN_GBI_SendCmd(GBI_COMM_TX_DATA *pData, GBI_CAN_MSG cmd, Uint16 pebbId);

extern GBI_COMM_TX_DATA g_gbiTxData[8];
extern GBI_COMM_RX_DATA g_gbiRxData[8];


#endif /* CAN_GBI_H_ */
