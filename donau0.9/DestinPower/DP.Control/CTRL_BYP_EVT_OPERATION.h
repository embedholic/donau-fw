/*
 * CTRL_BYP_EVT_OPERATION.h
 *
 *  Created on: 2016. 6. 30.
 *      Author: Yang_PC
 */

#ifndef DESTINPOWER_DP_CONTROL_CTRL_BYP_EVT_OPERATION_H_
#define DESTINPOWER_DP_CONTROL_CTRL_BYP_EVT_OPERATION_H_

#include "MCB.h"
#include "MCCB.h"

// 연속적으로 OC가 검출되면 고장발생
// 인자 설명
//     a ==> float : 순시전류
//     b ==> float : Max Level
//     c ==> float : Min Level

/*
#define CkeckOC(a, b, c) \
		if ( a >= b || a <= c ) \
		++(a##_OC_Count); \
		else \
		a##_OC_Count = 0;
*/

#define BYP_OC_COUNT_MAX	3
//#define BYP_EVT_OPERATION

extern void CTRL_BYP_EVT_OPERATION_Create(void);
extern void CTRL_BYP_EVT_OPERATION_Proceed(void);
//extern void CTRL_BYP_EVT_OPERATION_SCR_OC(void);

extern Bool bByp_Evt_Operating;
extern Bool bSSW_CB2_Disable;
extern Bool bSSW_CB2_Disable_Init;

typedef enum _Byp_Evt_State{
	BYP_EVT_SCR_OVERLOAD,
	BYP_EVT_BYP_OVERLOAD,
	BYP_EVT_NORMAL_STATE
}Byp_Evt_State;

extern Byp_Evt_State BYP_EVT_STATE;

#endif /* DESTINPOWER_DP_CONTROL_CTRL_BYP_EVT_OPERATION_H_ */
