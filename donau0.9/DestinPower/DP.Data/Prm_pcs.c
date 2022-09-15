/*
 * Prm_pcs.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#include "parameter.h"
#include "prm_pcs.h"
#include "EADC.h"
#include "trace.h"
#include "cc.h"
#include "SYSTEM.h"
//-#include "rs485.h"
#include "system.h"
#include "FastRms.h"
//-#include "CosphiControl.h"
#include "rms.h"
#include "trace.h"
#include "CAN_GBI.h"
#include "MO.h"
#include "CTRL_INV_PRC.h"
#include "CTRL_INV_DROOP.h"
#include "CTRL_INV_PQC.h"
#include "CTRL_INV_SYNC.h"
#include "CTRL_INV_VUC.h"
#include "CTRL_INV_VI.h"
#include "CTRL_INV_SEAMLESS.h"
#include "CTRL_INV_SYNC.h"
#include "CTRL_FILTER.h"
#include "Fault.h"
#include "CTRL_MODE.h"
#include "GPIO.h"
#include "MCB.h"
#include "SYS_ESLV.h"
#include "MCCB.h"
#include "CTRL_GEN.h"

Parameter_Table_Item PRM_PCS_TABLE[PARAM_NUM] =
{
	{		2,		1.0,         0,    1},	//SYS_MO_ENABLE,
	{	  123,		1.0,         0,    0},	//SYS_PEBB_MANUAL_OP,
	{	  100,		1.0,         0,    0},	//SYS_PEBB_ONOFF_WAIT_TIME,

	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB2_ON_LOAD_OFFSET,
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB2_OFF_LOAD_OFFSET,
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB3_ON_LOAD_OFFSET,  %
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB3_OFF_LOAD_OFFSET, %
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB4_ON_LOAD_OFFSET,  %
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB4_OFF_LOAD_OFFSET, %
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB5_ON_LOAD_OFFSET,  %
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB5_OFF_LOAD_OFFSET, %
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB6_ON_LOAD_OFFSET,  %
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB6_OFF_LOAD_OFFSET, %
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB7_ON_LOAD_OFFSET,  %
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB7_OFF_LOAD_OFFSET, %
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB8_ON_LOAD_OFFSET,  %
	{	  0,		0.1,         0,    0},	//SYS_SLAVE_PEBB8_OFF_LOAD_OFFSET, %


	{	  82,		1.0,         0,    0},	//SYS_REV_0, ( 120 DPWM ON LVL )
	{	  105,		1.0,         0,    0},	//SYS_REV_1, ( 120 DPWM OFF LVL ) - 한번 DPWM모드 진입하면 OFF시까지 꺼지지 않도록.
	{	  85,		1.0,         0,    0},	//SYS_REV_2, ( DPWM ON Temp LVL )
#if STABLEEN_HILL_ENABLE == 1
	{      1,       1.0,         0,    0},  //SYS_REV_3, ( DPWM OFF Temp LVL )  DBUG3: MC virtual mode 0: virtual
#else
//by JCNET!	{	  75, 	    1.0,         0,    0},	//SYS_REV_3, ( DPWM OFF Temp LVL )  DBUG3: MC virtual mode 0: virtual
	{     75,       1.0,         0,    0},  //SYS_REV_3, ( DPWM OFF Temp LVL )  DBUG3: MC virtual mode 0: virtual
#endif
	{	  0,		1.0,         0,    0},	//SYS_REV_4, ( Flexible Li )
	{	  0,		1.0,         0,    0},	//SYS_REV_5, ( Static Switch Control )
	{	  17,		1.0,         0,    0},	//SYS_OPTION,(System Option Bit 0: DC GRD, 1: DPWM ON 2:MC5 강제ON 3: START-STOP사용 여부 4: ES-LV 타입 재투입방지 5: 재투입 방지 후 자동 ON)
											// 6: BMS DC V 를 배터리측 모니터링 전압으로 사용 여부(Reserved) 7: RUN State에서 자동 Discharge 수행(Local모드)
											// 8: 2nd FR TYPE에서 Door Open을 경고로 설정할 것인지 여부
											// 9: Emergency Stop DI 를 Backfeed protection로 사용할 것인지
#if OP_MODE == STBLN_300K
	{		1,		1.0,         0,    1},	//INV_TRANSFORMER,
#elif OP_MODE == STBLN_500K || OP_MODE >= STBLN_1250K
	{		x,		1.0,         0,    1},	//INV_TRANSFORMER,
#else
//by JCNET	{		1,		1.0,         0,    1},	//INV_TRANSFORMER,
	{       0,      1.0,         0,    1},  //INV_TRANSFORMER,
#endif

	{ OP_MODE,		1.0,         0,    1},	//INV_CAPACITY,
	{	  125,		1.0,         0,    0},	//INV_OC_LEVEL,  // fixed
	{	 1000,		0.1,         0,    0},	//INV_CURRENT_LMT,
	{	   90,		1.0,         0,    0},	//INV_CAB_T_MAX,
	{	  100,		1.0,         0,    0},	//INV_CAB_T_MIN,==> PEBB HeatSync

	{ GRID_RATED_VOLTAGE, 1.0,   0,    1},	//GRID_RATED_VOLT,

	{	   50,		1.0,         0,    1},	//GRID_RATED_FREQ,
	{	  110,		1.0,         0,    0},	//GRID_OV_LEVEL1,
	{	   90,		1.0,         0,    0},	//GRID_UV_LEVEL1,
	{	   25,		0.1,         0,    0},	//GRID_OF_LEVEL1,
	{	   25,		0.1,         0,    0},	//GRID_UF_LEVEL1,
	{		0,		1.0,         0,    0},	//GRID_FDPR_ENB,
	{		0,		1.0,         0,    0},	//GRID_PWR_GRADIENT_ENB,
	{	  600,		1.0,         0,    0},	//GRID_PWR_GRADIENT_RAMP,
	{	   10,		1.0,         0,    0},	//GRID_TIME_SHIFT,
	{		0,		1.0,         0,    0},	//GRID_GATE_WAY_ENB,-> MC LESS
	{	    0,		1.0,         0,    1},	//GRID_LEVEL2_PROTECTION_ENABLE,
	{	  120,		1.0,         0,    0},	//GRID_OV_LEVEL2,
	{	   80,		1.0,         0,    0},	//GRID_UV_LEVEL2,
	{	   25,		0.1,         0,    0},	//GRID_UF_LEVEL2,
	{	    0,		1.0,         0,    0},	//GRID_OV_LEVEL1_TRIP_TIME,
	{	  100,		1.0,         0,    0},	//GRID_OV_LEVEL2_TRIP_TIME,
	{	    0,		1.0,         0,    0},	//GRID_UV_LEVEL1_TRIP_TIME,
	{	  100,		1.0,         0,    0},	//GRID_UV_LEVEL2_TRIP_TIME,
	{	    0,		1.0,         0,    0},	//GRID_UF_LEVEL1_TRIP_TIME,
	{	  100,		1.0,         0,    0},	//GRID_UF_LEVEL2_TRIP_TIME,
	{	    0,		1.0,         0,    0},	//GRID_OF_LEVEL1_TRIP_TIME, 14.10.30 100-> 0로 수정됨.
	{	    0,		1.0,         0,    0},	//GRID_FRT_ENB,
	{	    0,		1.0,         0,    0},	//GRID_FRT_OV_ENB,
	{	   40,		1.0,		 0,	   0},	//GRID_FDPR_GRADIENT_LEVEL,
	{	    5,	   0.01,		 0,	   0},	//GRID_FDPR_DEACTIVATION_FREQ,
	{	   15,		0.1,         0,    0},	//GRID_OF_LEVEL2,
	{	  100,		1.0,         0,    0},	//GRID_OF_LEVEL2_TRIP_TIME,
	{	    1,		1.0,         0,    0},	//GRID_RECONNECT_COND_ENB,

	{	 2009,		1.0,         0,    0},	//TIME_YEAR,
	{	    9,		1.0,         0,    0},	//TIME_MONTH,
	{	   16,		1.0,         0,    0},	//TIME_DAY,
	{	   14,		1.0,         0,    0},	//TIME_HOUR,
	{		5,		1.0,         0,    0},	//TIME_MINUTE,
	{		0,		1.0,         0,    0},	//TIME_SECOND,

#if DBUG_MODE == 0
	{	    0,		1.0,         0,    0},	//DGT_DI1, ( GI Mode Master / Slave Set )
	{	    0,		1.0,         0,    0},	//DGT_DO1, Test 모드일 때 GBI Fan speed control, FR: PCS ID
#else
	{	    0,		1.0,         0,    0},	//DGT_DI1,
	{	    0,		1.0,         0,    0},	//DGT_DO1,
#endif
	{	    2,		1.0,         0,    0},	//DGT_PWR_CTRL_MODE,
	{	    0,		1.0,         0,    0},	//DGT_CAN_ID
	{	    1,		1.0,         0,    0},	//DGT_REV_00 //151008 재투입 방지 300초 -> 1초(Disable)
//	{	    1,		1.0,         0,    0},	//DGT_MOD_RTU_ID // TODO

	{	   36,	   	0.01,         0,    0},	//ANL_AI1_OFFSET,
	{	   36,      0.01,         0,    0},	//ANL_AI1_GAIN,
	{	   36,	   	0.01,         0,    0},	//ANL_AI2_OFFSET,
	{	   55,  	0.01,         0,    0},	//ANL_AI2_GAIN,
	{	   75,	   	0.01,         0,    0},	//ANL_AI3_OFFSET,
	{	   75,      0.01,         0,    0},	//ANL_AI3_GAIN,
	{	 	0,	   	0.01,         0,    0},	//ANL_AI4_OFFSET,
	{	  	0,     	0.01,         0,    0},	//ANL_AI4_GAIN,

	{	  500,	   0.01,         0,    0},	//CTRL_VC_P_GAIN,
	{	 1000,	   0.01,         0,    0},	//CTRL_VC_I_GAIN,
	{	  100,		1.0,         0,    1},	//CTRL_VOLT_LPF,
	{	   19,	   0.01,         0,    0},	//CTRL_CC_P_GAIN,
	{	 3695,	   0.01,         0,    0},	//CTRL_CC_I_GAIN,
	{	    1,		1.0,         0,    0},	//CTRL_CC_DI_DT,
	{	 2500,		1.0,         0,    0},	//CTRL_RAMP,

	{FILTER_INDUCTANCE_LI,	1.0, 0,    0},	//CTRL_LI,
	{	    0,		0.1,         0,    0},	//CTRL_REV_00, [450]
	{	  200,		1.0,         0,    1},	//CTRL_CC_PERIOD,(10)	//	MG & STABLEEN 5kHz
	{	   20,	   0.01,         0,    0},	//CTRL_PLL_P_GAIN, LOGOS -> 0.05
	{	    8,	   0.01,         0,    0},	//CTRL_PLL_I_GAIN, LOGOS -> 0.02 (0.1:장비기동못함 Grid Freq 흔들림)
	{		0,	    1.0,         0,    0},	//CTRL_AUT_FAULT_RST, // 13.3.4 시험 중이므로, 일단 0
	{	   85,		1.0,         0,    0},	//CTRL_PEBB_HS_OVERTEMP,
	{	   	0,		1.0,         0,    1},	//CTRL_POWER_COMPEN,
	{	   	0,		1.0,         0,    0},	//CTRL_TEST_MODE, 0 ->1 by JCNET 11
	{	   	0,		1.0,         0,    0},	//CTRL_OPTION,
	{	    4,		1.0,         0,    0},	//CTRL_VOLT_DEVIATION_TIME,
											// 14.12.02 3->2 (BypassV 이용 Grid Fault 테스트 하면서 수정->규정 시간 내 고장 발생 하도록.)
											// 150916 검출 타입을 20ms->10ms 로 줄임. MC Less Type에서는 1로 해야 규정 시간 내(타이트 함) 고장 발생. 평상시에는 4로 사용.
	{	    0,		0.1,         0,    0},	//CTRL_REACTIVE_POWER,
	{	    0,		1.0,         0,    0},	//CTRL_BYP_V_GRID_TEST_MODE, (20) BYP V - FAULT TEST MODE
	{	   30,		1.0,         0,    0},	//CTRL_INV_IIR1ST_P
	{	   30,		1.0,         0,    0},	//CTRL_INV_IIR1ST_Q
	{	  100,		1.0,         0,    0},	//CTRL_REV_REMOTEPLIMIT,
	{	  200,		1.0,         0,    0},	//CTRL_TEST_PWM_VQE_REF, => INVCTRL.VqeRef + -> 테스트 모드 1에서만 사용 된다. 전압을 얼마로 만들 것인지. 레퍼런스.
	{	    0,		1.0,         0,    0},	//CTRL_REV_04_1,
	{	   20,		1.0,         0,    0},	//CTRL_CURR_UNBAL_LIMIT,
	{	    0,		1.0,         0,    0},	//CTRL_REV_05,
	{	   10,		1.0,         0,    0},	//CTRL_PWR_LIMIT_RAMP,
	{	 3000,		1.0,         0,    0},	//CTRL_SYNC_PI_MAG_P,		--> / 1000 : TODO HILL GAIN이 30임. 현장에서 맞춰야 함. -> 160113 "/ 100" 으로 수정
	{	    0,		1.0,         0,    0},	//CTRL_SYNC_PI_MAG_I, (30) --> / 1000 : TODO HILL GAIN이 30임. 현장에서 맞춰야 함. -> 160113 "/ 100" 으로 수정
	{	   50,	   0.01,         0,    0},	//CTRL_IIVCD_P_GAIN,
	{	  160,	   0.01,         0,    0},	//CTRL_IIVCD_I_GAIN,
	{	  200,	   0.01,         0,    0},	//CTRL_IIVCQ_P_GAIN,
	{	  500,	   0.01,         0,    0},	//CTRL_IIVCQ_I_GAIN,
	{		0,	    1.0,         0,    1},	//CTRL_ENABLE_ANTI_ISLANDING,
	{	    0,	   0.01,		 0,    0},	//CTRL_APS_LINE_DEADBAND,
	{	   10,	   0.01,		 0,    0},	//CTRL_REACTIVE_POWER_LIMIT,
	{	   20,	    0.1,		 0,    0},	//CTRL_ANTI_ISLANDING_K_FACTOR,
	{		1,		1.0,		 0,	   1}, // CTRL_PR_V_COMPEN,
	{	   10,		1.0,		 0,	   0}, // CTRL_SEAMLESS_CNT,(40) SEAMLESS 기능: UV OV 체크 카운트: 연속해서 UV 또는 OV가 이 카운트 이상 체크되면 SEAMLESS 절환 시도함. 141030 70->10으로 수정.
	{	    0,		1.0,		 0,	   0}, // CTRL_SEAMLESS_UV_CNT_STATUS,
	{	    0,		1.0,		 0,	   0}, // CTRL_SEAMLESS_OV_CNT_STATUS,
	{	  100,		0.1,		 0,	   0}, // CTRL_REV_12,
	{	   15,		0.1,		 0,	   0}, // CTRL_GRID_FS_RMS_CUTOFF_HZ,
	{	   85,		0.01,		 0,	   0}, // COSPHI_FACTOR_LIMIT,  140904 추가.
	{	    0,		1.0,		 0,	   0}, // reserved 46 ,
    {	    0,		0.1,		 0,	   0}, // CTRL_REV_15(R),
	{	    0,		1.0,		 0,	   0}, // CTRL_COSPHI_CONTROL_MODE,
	{	 1000,	  0.001,		 0,	   0}, // CTRL_COSPHI_POWER_FACTOR_INTERNAL,
	{	    0,	    0.1,		 0,	   0}, // CTRL_COSPHI_REACTIVE_POWER_INTERNAL(50)
	{	    1,	  0.001,		 0,	   0}, // CTRL_COSPHI_POWER_FACTOR_ACTUAL
	{	 1000,	  0.001,		 0,	   0}, // CTRL_COSPHI_POWER_FACTOR_RPC
	{	    0,	    0.1,		 0,	   0}, // CTRL_COSPHI_REACTIVE_POWER_RPC
	{	  985,	  0.001,		 0,	   0}, // CTRL_COSPHI_STRAY_RATIO,
	{		1,		1.0,		 0,	   0}, // CTRL_POS_SEQ_PLL_ENB,
	{	   20,	    0.1,		 0,	   0}, // CTRL_FRT_K_FACTOR,
	{	 4000,		1.0,         0,    0}, // CTRL_FRT_IQ_RAMP_TIME,
	{	 2000,		1.0,         0,    0}, // CTRL_FRT_IQ_RAMP_USE_TIME,
	{	 1000,		0.1,		 0,	   0}, // CTRL_INV_FS_RMS_CUTOFF_HZ,
	{	 1000,		0.1,		 0,	   0}, // CTRL_FREQ_CUTOFF_HZ,(60)

	{	  380,		  1,		 0,	   0}, // CTRL_QV_CTRL_TARGET_V,
	{	   31,		0.1,		 0,	   0}, // CTRL_QV_CTRL_K_FACTOR,
	{	 	1,		  1,		 0,	   0}, // CTRL_QV_CTRL_DEADBAND,

	{	   60,	    1.0,		 0,	   0}, // CTRL_QV_CTRL_RAMP_TIME,
	{	   10,	    1.0,		 0,	   0}, // CTRL_FRT_ASYNC_LEVEL,
	{	   10,		  1,		 0,	   0}, // CTRL_COSPHI_P_PN_RAMP_TIME,
	{	 	50,	  	1,		 	0,	   0}, // PARAM_TEMP_MIN
	{	    60,		1, 		 	0,	   0}, // PARAM_TEMP_MAX
	{	 	40,	  	1,		 	0,	   0}, // PARAM_FAN_START_LOAD
	{	   100,		1, 		 	0,	   0}, // PARAM_P_DE_LV1_TEMP(70)
	{	   100,	  	1,		 	0,	   0}, // PARAM_P_DE_LV2_TEMP
	{	   100,		1, 		 	0,	   0}, // PARAM_P_DE_CLEAR_TEMP
	{	   100,	  	1,		 	0,	   0}, // PARAM_P_DE_LV1-70% load
	{	   100,		1, 		 	0,	   0}, // PARAM_P_DE_LV2-50% load
	{	   180,	  	1,		 	0,	   0}, // CTRL_COSPHI_PF_5-PARAM_FAN_STOP_HOLD_TIME (171101)
	{	   	0,		1, 		 	0,	   0}, // CTRL_COSPHI_P_5
	{	 	0,	  	0.1,		 0,	   0}, // CTRL_COSPHI_PF_6
	{	   	0,		0.1, 		 0,	   0}, // CTRL_COSPHI_P_6
	{	 	0,	  	0.1,		 0,	   0}, // CTRL_COSPHI_PF_7
	{	   	0,		0.1, 		 0,	   0}, // CTRL_COSPHI_P_7(80)
	{	 	0,	  	0.1,		 0,	   0}, // CTRL_COSPHI_PF_8
	{	   	0,		0.1, 		 0,	   0}, // CTRL_COSPHI_P_8
	{	 	0,	  	0.1,		 0,	   0}, // CTRL_COSPHI_PF_9
	{	   	0,		0.1, 		 0,	   0}, // CTRL_COSPHI_P_9
	{	 	0,	  	0.1,		 0,	   0}, // CTRL_COSPHI_PF_10
	{	    0,		0.1, 		 0,	   0}, // CTRL_COSPHI_P_10

	{	 1000,	    1.0,		 0,	   0}, // CTRL_IINV_VOLTAGE_BUILDUP_TIME,
	{	   50,		1.0,		 0,	   0}, // CTRL_IINV_PLL_STABILIZE_TIME,	// STABLEEN은 복전 시 빠르게 GC로 복귀 해야 하기 때문에 확인 주기 짧음
	{	   20,	   0.01,		 0,	   0}, // CTRL_IINV_PLL_P_GAIN,
	{	   50,	   0.01,		 0,	   0}, // CTRL_IINV_PLL_I_GAIN,(90)
	{	  500,	   0.01,		 0,	   0}, // CTRL_IINV_SYNC_TOLERANCE_THETA,
	{	 1000,		  1, 		 0,	   0}, // CTRL_FRT_IQ_HOLD_TIME
	{	    0,		0.1, 		 0,	   0}, // CTRL_REV_0

	{		3,		1.0,         0,    0},	//TRC_TRACE_MODE,
	{		15,		1.0,         0,    0},	//TRC_TRACE_SAMPLING,
	{		40,	    1.0,         0,    0},	//TRC_TRACE1,
	{		41,	    1.0,         0,    0},	//TRC_TRACE2,
	{		42,	    1.0,         0,    0},	//TRC_TRACE3,
	{		163,	1.0,         0,    0},	//TRC_TRACE4,
	{		164,	1.0,         0,    0},	//TRC_TRACE5,
	{		165,	1.0,         0,    0},	//TRC_TRACE6,
	{		23,		1.0,         0,    0},	//TRC_TRACE7,
	{		24,		1.0,         0,    0},	//TRC_TRACE8,
	{		25,		1.0,         0,    0},	//TRC_TRACE9,
	{		0,		1.0,         0,    0},	//TRC_TRACE10,
	{		1,		1.0,         0,    0},	//TRC_TRACE11,
	{		3,		1.0,         0,    0},	//TRC_TRACE12,
	{		0,		1.0,         0,    1},	//TRC_TRACE13,
	{		1,		1.0,         0,    1},	//TRC_TRACE14,
	{		2,		1.0,         0,    1},	//TRC_TRACE15,
	{		3,		1.0,         0,    1},	//TRC_TRACE16,
	{		4,		1.0,         0,    1},	//TRC_TRACE17,
	{		5,		1.0,         0,    1},	//TRC_TRACE18,
	{		6,		1.0,         0,    1},	//TRC_TRACE19,
	{		7,		1.0,         0,    1},	//TRC_TRACE20,
	{		8,		1.0,         0,    1},	//TRC_TRACE21,
	{		9,		1.0,         0,    1},	//TRC_TRACE22,
	{		10,		1.0,         0,    1},	//TRC_TRACE23,
	{		11,		1.0,         0,    1},	//TRC_TRACE24,
	{		0,		1.0,         0,    1},	//TRC_TRACE25,
	{		1,		1.0,         0,    1},	//TRC_TRACE26,
	{		2,		1.0,         0,    1},	//TRC_TRACE27,
	{		3,		1.0,         0,    1},	//TRC_TRACE28,
	{		4,		1.0,         0,    1},	//TRC_TRACE29,
	{		5,		1.0,         0,    1},	//TRC_TRACE30,
	{		6,		1.0,         0,    1},	//TRC_TRACE31,
	{		7,		1.0,         0,    1},	//TRC_TRACE32,
	{		8,		1.0,         0,    1},	//TRC_TRACE33,
	{		9,		1.0,         0,    1},	//TRC_TRACE34,
	{		10,		1.0,         0,    1},	//TRC_TRACE35,
	{		11,		1.0,         0,    1},	//TRC_TRACE36,
	{		0,		1.0,         0,    1},	//TRC_TRACE37,
	{		1,		1.0,         0,    1},	//TRC_TRACE38,
	{		2,		1.0,         0,    1},	//TRC_TRACE39,
	{		3,		1.0,         0,    1},	//TRC_TRACE40,
	{		4,		1.0,         0,    1},	//TRC_TRACE41,
	{		5,		1.0,         0,    1},	//TRC_TRACE42,
	{		6,		1.0,         0,    1},	//TRC_TRACE43,
	{		7,		1.0,         0,    1},	//TRC_TRACE44,
	{		8,		1.0,         0,    1},	//TRC_TRACE45,
	{		9,		1.0,         0,    1},	//TRC_TRACE46,
	{		10,		1.0,         0,    1},	//TRC_TRACE47,
	{		11,		1.0,         0,    1},	//TRC_TRACE48,

	{		0,		0.1,         0,    0},	//OFS_DCLINK_VOLTAGE
	{		0,		0.1,         0,    0},	//OFS_BAT_CURRENT,
	{		0,		0.1,         0,    0},	//OFS_BATT_VOLTAGE,
	{		0,		0.1,         0,    0},	//OFS_INV_I_L1,
	{		0,		0.1,         0,    0},	//OFS_INV_I_L3,
	{		0,		0.1,         0,    0},	//OFS_INV_I_L2-> INV_I_L2
	{		0,		0.1,         0,    0},	//OFS_STACK_TEMP,
	{		0,		0.1,         0,    0},	//OFS_CAB_TEMP,
	{		0,		0.1,         0,    0},	//OFS_BATT_CHG_CURRENT,

#if STABLEEN_HILL_ENABLE == 1
	/* HILL Gain */
	{	 20000,	  0.0001,        0,    0},	//GN_DCLINK_VOLTAGE
	{	 20000,	  0.0001,        0,    0},	//GN_BAT_CURRENT, // 방전시 게인
	{	 20000,	  0.0001,        0,    0},	//GN_BAT_VOLTAGE
	{	 20000,	  0.0001,        0,    0},	//GN_INV_V_L1,
	{	 20000,	  0.0001,        0,    0},	//GN_INV_V_L2,
	{	 20000,	  0.0001,        0,    0},	//GN_INV_V_L3,
	{	 20000,	  0.0001,        0,    0},	//GN_INV_I_L1,
	{	 20000,	  0.0001,        0,    0},	//GN_INV_I_L3,
	{	 20000,	  0.0001,        0,    0},	//GN_GRID_V_L1,
	{	 20000,	  0.0001,        0,    0},	//GN_GRID_V_L2,
	{	 20000,	  0.0001,        0,    0},	//GN_GRID_V_L3,
	{	 20000,	  0.0001,        0,    0},	//GN_INV_I_L2, -> INV_I_2
	{	 10000,	  0.0001,        0,    0},	//GN_STACK_TEMP,
	{	 10000,	  0.0001,        0,    0},	//GN_CAB_TEMP,
	{	 10000,	  0.0001,        0,    0},	//GN_BATT_CHG_CURRENT,
#else
	{	 10000,	  0.0001,        0,    0},	//GN_DCLINK_VOLTAGE
	{	 10000,	  0.0001,        0,    0},	//GN_BAT_CURRENT, // 방전시 게인
	{	 10000,	  0.0001,        0,    0},	//GN_BAT_VOLTAGE
	{	 10000,	  0.0001,        0,    0},	//GN_INV_V_L1,
	{	 10000,	  0.0001,        0,    0},	//GN_INV_V_L2,
	{	 10000,	  0.0001,        0,    0},	//GN_INV_V_L3,
	{	 10000,	  0.0001,        0,    0},	//GN_INV_I_L1,
	{	 10000,	  0.0001,        0,    0},	//GN_INV_I_L3,
	{	 10000,	  0.0001,        0,    0},	//GN_GRID_V_L1,
	{	 10000,	  0.0001,        0,    0},	//GN_GRID_V_L2,
	{	 10000,	  0.0001,        0,    0},	//GN_GRID_V_L3,
	{	 10000,	  0.0001,        0,    0},	//GN_INV_I_L2, -> INV_I_2
	{	 10000,	  0.0001,        0,    0},	//GN_GRID_I_L1,
	{	 10000,	  0.0001,        0,    0},	//GN_GRID_I_L2,
	{	 10000,	  0.0001,        0,    0},	//GN_GRID_I_L3,
#endif
//13.3.5 for BC------->
	{	   800,		   1,        0,    0},	//1 BATT_OV_LEVEL,
	{	   550,		   1,        0,    0},	//2 BATT_UV_LEVEL,
	{	   150,		   1,        0,    0},	//3 BATT_OC_LEVEL,
	{	    50,		   1,        0,    0},	//4 BATT_DC_SIDE_V_ABNORMAL_LEVEL,

	{	   750,		   1,        0,    0},	//5 BATT_V_RANGE_MAX,
	{	   570,		   1,        0,    0},	//6 BATT_V_RANGE_MIN,

	{	     1 /*LOCAL by JCNET 2*/,	   	   1,        0,    1},	//7 BATT_ORDER_SOURCE,
	{	     0,	   	   1,        0,    0},	//8 BATT_REF_VAL_MODE,
	{	     0,	   	   1,        0,    1},	//9 BATT_LOCAL_CONTROL_MODE,
	{	     1 /*CHARGE by JCNET 0*/,	   	   1,        0,    0},	//10 BATT_LOCAL_POWER_FLOW,
	{	     0,	   	   1,        0,    1},	//11 BATT_LOCAL_CONTROL_OPTION,
	{     6700,		 0.1,        0,    0},	//12 BATT_LOCAL_IVC_V_REF_CHG,
	{     5700,		 0.1,        0,    0},	//13 BATT_LOCAL_IVC_V_REF_DCHG,
	{	     0,	   	 0.1,        0,    0},	//14 BATT_LOCAL_ICC_I_REF_CHG,
	{	     0,	   	 0.1,        0,    0},	//15 BATT_LOCAL_ICC_I_REF_DCHG,
	{	  	 0,		 0.1,        0,    0},	//16 BATT_LOCAL_PCC_P_REF_CHG,
	{	  	 0,		 0.1,        0,    0},	//17 BATT_LOCAL_PCC_P_REF_DCHG,

	{	     0,	   	   1,        0,    1},	//18 BATT_REMOTE_CONTROL_MODE,
	{	     0,	   	   1,        0,    0},	//19 BATT_REMOTE_POWER_FLOW,
	{	     0,	   	   1,        0,    1},	//20 BATT_REMOTE_CONTROL_OPTION 1: Openloop,
	{	  	 0,		 0.1,        0,    0},	//21 BATT_REMOTE_PCC_P_REF
	{	  	 0,		 0.1,        0,    0},	//22 BATT_REMOTE_PCC_Q_REF

	{	  5000,		0.01,        0,    0},	//23 BATT_IVC_VDC_RAMP,

	{	 32000,		 0.1,        0,    0},	//24 BATT_IVC_PCC_CURRENT_LMT_CHG,
	{	 32000,		 0.1,        0,    0},	//25 BATT_IVC_PCC_CURRENT_LMT_DCHG,~

	{	   100,		0.01,        0,    0},	//26 BATT_ICC_P_GAIN,
	{	   100,	    0.01,        0,    0},	//27 BATT_ICC_I_GAIN,

	{	    10,   0.0001,       0,    0},	//28 BATT_PCC_P_GAIN
	{	    15,	    0.01,        0,    0},	//29 BATT_PCC_I_GAIN
	{	  3000,	    0.01,        0,    0},	//30 BATT_PCC_PQ_CUTOFF_HZ,
	{	     5,		   1,        0,    0},	//31 BATT_PCC_PWM_DISABLE_SEC
	{	     1,		   1,        0,    0},	//32 BATT_PEBB_FAULT_MODE, TODO 파라미터 변경
	{	     1,		   1,        0,    1},	//33 RESYNC_W_PI_MAX,

	{	     0,		   1,        0,    0},	//34 BATT_PEBB_FAULT_ID,
	{	     0,		   1,        0,    0},	//35 BATT_REBB_FAULT_RST,
	{	     0,		   1,        0,    0},	//36 BATT_PEBB_0_TEMP_S,
	{	     0,		   1,        0,    0},	//37 BATT_PEBB_1_TEMP_S,
	{	     0,		   1,        0,    0},	//38 BATT_PEBB_0_HEATSYNC,
	{	     0,		   1,        0,    0},	//39 BATT_PEBB_1_HEATSYNC,

	{        0,		   1,        0,    0},	//40 GEN_CONNECT_CHK_STX_POWER
	{	     0,		   1,        0,    0},	//41 GEN_CONNECT_I_DIDT
	{	     0,		   1,        0,    0},	//42 BATT_3, Source Order status
	{	     0,		   1,        0,    0},	//43 BATT_4, Ref Value
	{	     0,		   1,        0,    0},	//44 BATT_5, 5
	{	     0,		   1,        0,    0},	//45 BATT_6, 6
	{	   	 0,		 0.01,        0,    0},	//46 BATT_CUTOFF_CURRENT_CHG
	{	 15000,		 0.01,        0,    0},	//47 VCON_P_GAIN 전압제어 p-gain
	{	    50,		 0.01,        0,    0},	//48 BATT_8
	{	     1,		 0.01,        0,    0},	//49===BATT_CUTOFF_VOLTAGE_DCHG,
	{	     1,		 0.01,        0,    0},	//50===BATT_VDCREF_CHG,(10)
	{	     1,		 0.01,        0,    0},	//51===BATT_VDCREF_DCHG,
	{   	10,		 0.01,        0,    0},	//52===BATT_CYCLING_CNT,
	{  	    10,		 0.01,        0,    0},	//53===BATT_INIT_STATE,
	{       10,		 0.01,        0,    0},	//54===BATT_CHG_TIME,
	{      200,		 0.01,        0,    0},	//55===BATT_DCHG_TIME,
	{    15000,		 0.01,        0,    0},	//56 BATT_START_IDLE_TIME 전압제어 1차 i-gain
	{	  2000, 	 0.01,        0,    0},	//57===BATT_CHG_PWR,
	{	     0,	       1,        0,    0},	//58===BATT_DCHG_PWR, ==> Islanding : 1.5
	{	    50,    0.001,        0,    0},	//59===IGAIN BATT_END_COND_HOLD_TIME,
	{	    10,		0.01,        0,    0},	//60===NGAIN BATT_VDC_RAMP,(20)
	{	     0,	   	   1,        0,    0},	//61===BATT_VF_CHECK_TIME,
	{	    10,	   	0.01,        0,    0},	//62===MGAIN BATT_CURRENT_REF_OFFSET,
	{	  2000,	   	0.001,        0,    0},	//63===DROOP.GAIN.N BATT_CHG_DCHG_MODE,
	{	    10,    0.001,        0,    0},	//64===DROOP.GAIN.M BATT_CONST_PWR_CHG,
	{	   500,	       1,        0,    0},	//65===RAMP.v_ref BATT_CONST_PWR_DCHG,
	{	   500,		   1,        0,    0},	//66===RAMP.w_ref BATT_DCC_P_GAIN,
	{	    20,	       1,        0,    0},	//67===v droop ramp V_DROOP_RAMP,
	{	    10,		 0.1,        0,    0},	//68 CC_CUT_OFF_FREQ,
	{	     5,		   1,        0,    0},	//69 SYNC_W_PI_KP_DIVISION(TODO 0이할 경우 1이 되도록!)
	{	   330,		   1,	     0,    0},	//70 GI_V_REF ( varient 0.1 -> 1 ) 1603
	{	   100,		   1,        0,    0},	//71 BESS71
	{	     0,        1,        0,    0},	//72 BESS72
	{	     1,	       1,        0,    0},	//73 BESS73
#if DBUG_MODE == 0
#if 0 //by JCNET
	{	     2,	       1,        0,    1},	//74 -> CONTROLLER_SEL CSI(0), VSI(1) P+R(2)
	{	     2,	       1,        0,    1},	//75 -> GC_IS_MODE     GC(0), IS(1), GC+IS(2,Auto GI)
	{	     0,	       1,        0,    1},	//76 -> DROOP_ENB DROOP DISABLE(0), ENABLE(1)
#else
    {        0,        1,        0,    1},  //74 -> CONTROLLER_SEL CSI(0), VSI(1) P+R(2)
    {        0,        1,        0,    1},  //75 -> GC_IS_MODE     GC(0), IS(1), GC+IS(2,Auto GI)
    {        0,        1,        0,    1},  //76 -> DROOP_ENB DROOP DISABLE(0), ENABLE(1)
#endif
#else
	{	     0,	       1,        0,    0},	//74 -> CONTROLLER_SEL CSI(0), VSI(1) P+R(2)
	{	     2,	       1,        0,    0},	//75 -> GC_IS_MODE     GC(0), IS(1), GC+IS(2,Auto GI)
	{	     0,	       1,        0,    0},	//76 -> DROOP_ENB DROOP DISABLE(0), ENABLE(1)
#endif
	{	     0,	       1,        0,    0},	//77 -> TEST_MODE2 동기 상태에서 대기 활성화 (PCS 시험 시에 이용 가능)
	{	  1000,	   0.001,        0,    0},	//78 -> 계통연계 Q i-gain BATT_PRE_C_SKIP,	(Battery Present Cycle Skip)
	{	   100,		0.01,        0,    0},	//79,
	{	   100,		0.01,        0,    0},	//80
	{	     0,		   1,        0,    0},	//81 VI_ENB
	{	    10,		   1,        0,    0},	//82 VI_R_P (%)
	{	     3,		   1,        0,    0},	//83 VI_R_PI (%)
	{	   -10,		   1,        0,    0},	//84 VI_L_P (%)
	{	     0,		   1,        0,    0},	//85 VI_L_PI (%)
	{	   500,	   	   1,        0,    0},	//86 VI_T_MSEC
	{	   110,		   1,        0,    0},	//87 VI_LEVEL (%)
	{	     0,		   1,        0,    0},	//88 VUC_ENB
	{	   100,     0.01,        0,    0},	//89 VUC_PI_KP
	{	     1,	    0.01,        0,    0},	//90 VUC_PI_KI
	{	   300,      0.1,        0,    0},	//91 VUC_PEAK_LPF
	{	   150,	    0.01,        0,    0},	//92 VC_PR_CUTOFF_IS
	{	    20,		   1,        0,    0},	//93 BYP_MODE	0:NONE 1:BYP_ENB 2: ES-LV 타입 GC->OFF->GI->AUTO GC 활성화
	{	     1,		   1,        0,    0},	//94 SEAMLESS_ENB
	{	   200,      1.0,        0,    1},	//95 IS_CCP			//	MG & STABLEEN 5kHz
	{	   160,		 1.0,        0,    0},	//96 IS_OV_LEVEL1,
	{	    80,		 1.0,        0,    0},	//97 IS_UV_LEVEL1,
	{	    30,		 0.1,        0,    0},	//98 IS_OF_LEVEL1,
	{	    30,		 0.1,        0,    0},	//99 IS_UF_LEVEL1,
	{	   115,		 1.0,        0,    0},	//100 GRID_OV_LEVEL_INSTANT,
	{	    85,		 1.0,        0,    0},	//101 GRID_UV_LEVEL_INSTANT,
	{	   110,		 1.0,        0,    0},	//102 BYP_OV_LEVEL1,
	{	    90,		 1.0,        0,    0},	//103 BYP_UV_LEVEL1,
	{	    30,		 0.1,        0,    0},	//104 GRID_OF_LEVEL2_SEAMLESS, 1603
	{	    30,		 0.1,        0,    0},	//105 GRID_UF_LEVEL2_SEAMLESS, 1603
	{	   130,		 1.0,        0,    0},	//106 GRID_OV_LEVEL2_INSTANT, 1603
	{	    82,		 1.0,        0,    0},	//107 GRID_UV_LEVEL2_INSTANT, 1603
	{	     1,		   1,        0,    0},	//108 DROOP_FREQ (%)
	{	     3,		   1,        0,    0},	//109 DROOP_VOLT (%)
	{	     3,		   1,        0,    0},	//110 DROOP_TR_Z (%)
	{	     2,    0.001,        0,    0},	//111 SYNC_W_PI_KP,			// 151104 배영상 책임 요청. SYNC_W_PI_KP_DIVISION추가
	{	     0,	   0.001,        0,    0},	//112 SYNC_W_PI_KI,
	{	    15,		   1,        0,    0},	//113 SYNC_W_PI_MAX (%)		// BYP30-두대 병렬 Sync시에는 1로 해야 OC발생 안함.
	{	    10,	     1.0,        0,    1},	//114 SYNC_W_IIR_CUTOFF,
	{	     5,	     1.0,        0,    1},	//115 SYNC_V_IIR_CUTOFF,
	{	     0,	     1.0,        0,    0},	//116 WH_CHG_HI, 	-> AC C kWh
	{	     0,	     1.0,        0,    0},	//117 WH_CHG_LO, 	-> AC C mWh
	{	     0,	     1.0,        0,    0},	//118 WH_DCHG_HI,   -> AC C gWh
	{	     0,	     1.0,        0,    0},	//119 WH_DCHG_LO,	-> AC D kWh

	{	     0,	     1.0,        0,    0},	//120 MWH_CHG_AC,	-> AC D mWh
	{	     0,	     1.0,        0,    0},	//121 GWH_CHG_AC,	-> AC D gWh
	{	     0,	     1.0,        0,    0},	//122 MWH_DCHG_AC,	-> DC C kWh
	{	     0,	     1.0,        0,    0},	//123 GWH_DCHG_AC,	-> DC C mWh
	{	     0,	     1.0,        0,    0},	//124 MWH_CHG_DC,	-> DC C gWh
	{	     0,	     1.0,        0,    0},	//125 GWH_CHG_DC,	-> DC D kWh
	{	     0,	     1.0,        0,    0},	//126 MWH_DCHG_DC,	-> DC D mWh
	{	     0,	     1.0,        0,    0},	//127 GWH_DCHG_DC,	-> DC D gWh

#if STABLEEN_HILL_ENABLE == 1
    {    20000,   0.0001,        0,    0},  //128 GN_BYP_V_L1
    {    20000,   0.0001,        0,    0},  //129 GN_BYP_V_L2
    {    20000,   0.0001,        0,    0},  //130 GN_BYP_V_L3
    {    20000,   0.0001,        0,    0},  //131 GN_GEN_V_L1
    {    20000,   0.0001,        0,    0},  //132 GN_GEN_V_L1
    {    20000,   0.0001,        0,    0},  //133 GN_GEN_V_L1
    {    20000,   0.0001,        0,    0},  //134 GN_REV_L1
    {    20000,   0.0001,        0,    0},  //135 GN_REV_L2
    {    20000,   0.0001,        0,    0},  //136 GN_REV_L3
    {        0,   0.0001,        0,    0},  //137 GN_LOAD_I_L1  부하전류R
    {        0,   0.0001,        0,    0},  //138 GN_LOAD_I_L2  부하전류S
    {        0,   0.0001,        0,    0},  //139 GN_LOAD_I_L3  부하전류T
#else
	{	 10000,	  0.0001,        0,    0},	//128 GN_BYP_V_L1
	{	 10000,	  0.0001,        0,    0},	//129 GN_BYP_V_L2
	{	 10000,	  0.0001,        0,    0},	//130 GN_BYP_V_L3
	{	 10000,	  0.0001,        0,    0},	//131 GN_GEN_V_L1
	{	 10000,	  0.0001,        0,    0},	//132 GN_GEN_V_L1
	{	 10000,	  0.0001,        0,    0},	//133 GN_GEN_V_L1
	{	 10000,	  0.0001,        0,    0},	//134 GN_REV_L1
	{	 10000,	  0.0001,        0,    0},	//135 GN_REV_L2
	{	 10000,	  0.0001,        0,    0},	//136 GN_REV_L3
	{	 10000,	  0.0001,        0,    0},	//137 GN_LOAD_I_L1	부하전류R
	{	 10000,	  0.0001,        0,    0},	//138 GN_LOAD_I_L2	부하전류S
	{	 10000,	  0.0001,        0,    0},	//139 GN_LOAD_I_L3	부하전류T
#endif
	{	   300,	     0.1,        0,    0},	//140 BATT_HOLD_SEC,
	{	   300,	     0.1,        0,    0},	//141 BATT_P_DECREASE_SEC,
	{	   250,	     0.1,        0,    0},	//142 BATT_GENERATOR_POWER_P,
	{	     0,	     0.1,        0,    0},	//143 BATT_GENERATOR_POWER_Q,
	{	     0,	     0.1,        0,    0},	//144 BATT_POWER_METER_P,
	{	     0,	     0.1,        0,    0},	//145 BATT_POWER_METER_Q,
	{	     0,	     0.1,        0,    0},	//146 BATT_REV_06,
	{	     0,	     0.1,        0,    0},	//147 BATT_REV_07,
	{	     0,	     0.1,        0,    0},	//148 BATT_REV_08,
	{	     0,	     0.1,        0,    0},	//149 BATT_REV_09,

	{	  -500,	     0.1,        0,    0},	//150 AUTO_CHARGE_P_REF,
	{	   200,	     0.1,        0,    0},	//151 SYNC_PI_MAG_LIMIT /* 20% */
	{	     0,	     0.1,        0,    0},	//152 ,
	{	     0,	     0.1,        0,    0},	//153 PM_OVERLOAD_LV1_KVA,
	{	     0,	     0.1,        0,    0},	//154 PM_OVERLOAD_LV2_KVA,
	{	     0,	       1,        0,    0},	//155 PM_OVERLOAD_TIME,
	{	     0,	       1,        0,    0},	//156 PM_OVERLOAD_CLEAR_TIME,
	{	     0,	     0.1,        0,    0},	//157 SCR_RATED_CURRENT,
	{	     0,	     0.1,        0,    0},	//158 SCR_CURRENT_LIMIT,
	{	   125,	       1,        0,    0},	//159 LOAD_OC_LEVEL,

	{       50,     0.01,        0,    0}, //160 GC_CC_PR_P_GAIN,      GC 전류 P Gain, ( 170323 Varient 0.1 -> 0.01  152~156)
    {      200,     0.01,        0,    0}, //161 GC_CC_PR_I_GAIN_1ST,  GC 전류 I Gain 1st,
    {        1,     0.01,        0,    0},  //162 GC_CC_PR_I_GAIN_5TH,  GC 전류 I Gain 5th,
    {        1,     0.01,        0,    0},  //163 GC_CC_PR_I_GAIN_7TH,  GC 전류 I Gain 7th,
    {        1,     0.01,        0,    0},  //164 GC_CC_PR_I_GAIN_11TH, GC 전류 I Gain 11th,

    {       50,     0.01,        0,    0},  //165 GI_CC_PR_P_GAIN,      GI 전류 P Gain,
    {      200,     0.01,        0,    0},  //166 GI_CC_PR_I_GAIN_1ST,  GI 전류 I Gain 1st,
    {        1,     0.01,        0,    0},  //167 GI_CC_PR_I_GAIN_5TH,  GI 전류 I Gain 5th,
    {        1,     0.01,        0,    0},  //168 GI_CC_PR_I_GAIN_7TH,  GI 전류 I Gain 7th,
    {        1,     0.01,        0,    0},  //169 GI_CC_PR_I_GAIN_11TH, GI 전류 I Gain 11th,

    {    15000,     0.01,        0,    0},  //170 GI_VC_PR_P_GAIN,      GI 전압 P Gain,
    {    15000,     0.01,        0,    0},  //171 GI_VC_PR_I_GAIN_1ST,  GI 전압 I Gain 1st,
    {       10,     0.01,        0,    0},  //172 GI_VC_PR_I_GAIN_5TH,  GI 전압 I Gain 5th,
    {       10,     0.01,        0,    0},  //173 GI_VC_PR_I_GAIN_7TH,  GI 전압 I Gain 7th,
    {       10,     0.01,        0,    0},  //174 GI_VC_PR_I_GAIN_11TH, GI 전압 I Gain 11th,

    {        8,        1,        0,    0},  //175 SEAMLESS_PWM_OFF_CNT, Seamless 동작 시 PWM Off Counter
#if STABLEEN_HILL_ENABLE == 1
    {        0,        1,        0,    0},  //176 EVE_DC_CHARGER_ENABLE, 1: EVE 사용, 0: EVE 사용X
#else
    {        1,        1,        0,    0},  //176 EVE_DC_CHARGER_ENABLE, 1: EVE 사용, 0: EVE 사용X
#endif
    {      900,        1,        0,    0},  //177 EVE_MAXIMUM_CHARGE_VOLTAGE, EVE 충전 목표 전압
	{      580,        1,        0,    0},  //178 EVE_START_CHARGE_VOLTAGE, EVE 충전 시작 전압
	{       20,        1,        0,    0},  //179 HOLD_LOAD_POWER_TIME, GC 시 Load Power Hold 시간
    {      580,        1,        0,    0},  //180 AC_GEN_DC_VOLTAGE_LEVEL, AC Generate 넘어가는 DC 전압
	{        0,        1,        0,    0},  //181 SEAMLESS_PR_STOP
	{        6,        1,        0,    0},  //182 SEAMLESS_FAST_SCR_OFF
	{      500,        1,        0,    0},   //183 TRAN_CNT
//by JCNET
	{      100,     0.01,        0,    0},  // CTRL_SCC_D_P_GAIN
    {      100,     0.01,        0,    0},  // CTRL_SCC_D_I_GAIN
    {      100,     0.01,        0,    0},  // CTRL_SCC_Q_P_GAIN
    {      100,     0.01,        0,    0},  // CTRL_SCC_Q_I_GAIN
    {      100,     0.01,        0,    0},  // CTRL_SCC_N_P_GAIN
    {      100,     0.01,        0,    0},  // CTRL_SCC_N_I_GAIN
    {      200,     0.01,        0,    0},  // CTRL_SCC_D_LIMIT
    {      480,     1.00,        0,    0},  // CTRL_SCC_Q_LIMIT
    {      100,     0.01,        0,    0},  // CTRL_SCC_N_LIMIT
};

Parameter PRM_PCS_BUF[PARAM_NUM];
Parameter *PRM_PCS = PRM_PCS_BUF;
int		PARAM_GROUP_INDEX[PARAM_GROUP_MAX]=
		{
			SYS_MO_ENABLE,				//SYSTEM
			INV_TRANSFORMER,			//INVERTER
			GRID_RATED_VOLT,			//GRID
			TIME_YEAR,					//DATE_TIME
			DGT_DI1,					//DIGITAL INTERFACE
			ANL_AI1_OFFSET,				//ANALOG INTERFACE
			CTRL_VC_P_GAIN,				//CONTROLLER
			TRC_TRACE_MODE,				//TRACE
			OFS_DCLINK_VOLTAGE,				//OFFSET
			GN_DCLINK_VOLTAGE,				//GAIN
			//-13.3.5 for BC ADD_DEBUG_IN_REL,
			BATT_OV_LEVEL, //+13.3.5 for BC
		};


/*
 * idleLoop() 에서 호출 함.
 * 파라미터에 변경 사항이 있으면
 */
void PARAM_UpdateAll()
{
	//-int iTemp;

#if 1
	if ( PRM_PCS[SYS_MO_ENABLE].bChange )
	{
		PRM_PCS[SYS_MO_ENABLE].bChange = FALSE;
	}

	if ( PRM_PCS[SYS_PEBB_MANUAL_OP].bChange )
	{
		if( PRM_PCS[SYS_MO_ENABLE].iValue == 0)
		{
			if( INVERTER.uStatus == SYS_INV_FAULT || INVERTER.uStatus == SYS_INV_STOP || INVERTER.uStatus == SYS_INV_TEST_MODE)
			{
				switch(PRM_PCS[SYS_PEBB_MANUAL_OP].iValue)
				{
				case 0:
					MO_SetState(PEBB_ALL_OFF);
					break;
				case 1:
					MO_SetState(PEBB0_ON);
					break;
				case 2:
					MO_SetState(PEBB1_ON);
					break;
				case 3:
					MO_SetState(PEBB2_ON);
					break;
				case 4:
					MO_SetState(PEBB3_ON);
					break;
				case 5:
					MO_SetState(PEBB4_ON);
					break;
				case 6:
					MO_SetState(PEBB5_ON);
					break;
				case 7:
					MO_SetState(PEBB6_ON);
					break;
				case 8:
					MO_SetState(PEBB7_ON);
					break;
				}

			}
		}

		PRM_PCS[SYS_PEBB_MANUAL_OP].iValue = -1;
		PRM_PCS[SYS_PEBB_MANUAL_OP].bChange = FALSE;
	}

	if ( PRM_PCS[SYS_PEBB_ONOFF_WAIT_TIME].bChange )
	{
		MO_SetOnOffWaitTime_ms(PRM_PCS[SYS_PEBB_ONOFF_WAIT_TIME].iValue);
		PRM_PCS[SYS_PEBB_ONOFF_WAIT_TIME].bChange = FALSE;
	}

	//--------------- PEBB 2  -------------------------//
	if ( PRM_PCS[SYS_SLAVE_PEBB2_ON_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOnLoad(1, PARAM_VAL(SYS_SLAVE_PEBB2_ON_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB2_ON_LOAD_OFFSET].bChange = FALSE;
	}
	if ( PRM_PCS[SYS_SLAVE_PEBB2_OFF_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOffLoad(1, PARAM_VAL(SYS_SLAVE_PEBB2_OFF_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB2_OFF_LOAD_OFFSET].bChange = FALSE;
	}

	//--------------- PEBB 3  -------------------------//
	if ( PRM_PCS[SYS_SLAVE_PEBB3_ON_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOnLoad(2, PARAM_VAL(SYS_SLAVE_PEBB3_ON_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB3_ON_LOAD_OFFSET].bChange = FALSE;
	}
	if ( PRM_PCS[SYS_SLAVE_PEBB3_OFF_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOffLoad(2, PARAM_VAL(SYS_SLAVE_PEBB3_OFF_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB3_OFF_LOAD_OFFSET].bChange = FALSE;
	}

	//--------------- PEBB 4  -------------------------//
	if ( PRM_PCS[SYS_SLAVE_PEBB4_ON_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOnLoad(3, PARAM_VAL(SYS_SLAVE_PEBB4_ON_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB4_ON_LOAD_OFFSET].bChange = FALSE;
	}
	if ( PRM_PCS[SYS_SLAVE_PEBB4_OFF_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOffLoad(3, PARAM_VAL(SYS_SLAVE_PEBB4_OFF_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB4_OFF_LOAD_OFFSET].bChange = FALSE;
	}

	//--------------- PEBB 5  -------------------------//
	if ( PRM_PCS[SYS_SLAVE_PEBB5_ON_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOnLoad(4, PARAM_VAL(SYS_SLAVE_PEBB5_ON_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB5_ON_LOAD_OFFSET].bChange = FALSE;
	}
	if ( PRM_PCS[SYS_SLAVE_PEBB5_OFF_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOffLoad(4, PARAM_VAL(SYS_SLAVE_PEBB5_OFF_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB5_OFF_LOAD_OFFSET].bChange = FALSE;
	}

	//--------------- PEBB 6  -------------------------//
	if ( PRM_PCS[SYS_SLAVE_PEBB6_ON_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOnLoad(5, PARAM_VAL(SYS_SLAVE_PEBB6_ON_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB6_ON_LOAD_OFFSET].bChange = FALSE;
	}
	if ( PRM_PCS[SYS_SLAVE_PEBB6_OFF_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOffLoad(5, PARAM_VAL(SYS_SLAVE_PEBB6_OFF_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB6_OFF_LOAD_OFFSET].bChange = FALSE;
	}

	//--------------- PEBB 7  -------------------------//
	if ( PRM_PCS[SYS_SLAVE_PEBB7_ON_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOnLoad(6, PARAM_VAL(SYS_SLAVE_PEBB7_ON_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB7_ON_LOAD_OFFSET].bChange = FALSE;
	}
	if ( PRM_PCS[SYS_SLAVE_PEBB7_OFF_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOffLoad(6, PARAM_VAL(SYS_SLAVE_PEBB7_OFF_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB7_OFF_LOAD_OFFSET].bChange = FALSE;
	}

	//--------------- PEBB 8  -------------------------//
	if ( PRM_PCS[SYS_SLAVE_PEBB8_ON_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOnLoad(7, PARAM_VAL(SYS_SLAVE_PEBB8_ON_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB8_ON_LOAD_OFFSET].bChange = FALSE;
	}
	if ( PRM_PCS[SYS_SLAVE_PEBB8_OFF_LOAD_OFFSET].bChange )
	{
		MO_SetPebbOffLoad(7, PARAM_VAL(SYS_SLAVE_PEBB8_OFF_LOAD_OFFSET));
		PRM_PCS[SYS_SLAVE_PEBB8_OFF_LOAD_OFFSET].bChange = FALSE;
	}


	if ( PRM_PCS[SYS_DPWM_ONLOAD].bChange )
	{
		PRM_PCS[SYS_DPWM_ONLOAD].bChange = FALSE;
	}

	if ( PRM_PCS[SYS_DPWM_OFFLOAD].bChange )
	{
		PRM_PCS[SYS_DPWM_OFFLOAD].bChange = FALSE;
	}
#if 0
	if ( PRM_PCS[SYS_REV_5].bChange )
	{
		if( PRM_PCS[CTRL_TEST_MODE].iValue != 0 )
		{
			if(PRM_PCS[SYS_REV_5].iValue == 1 )
			{
				GPIO_StaticSwitch(GPIO_STATIC_SW_ON);
			}
			else if( PRM_PCS[SYS_REV_5].iValue == 2)
			{
				GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
			}
		}
		PRM_PCS[SYS_REV_5].bChange = FALSE;
	}

#endif
#if 0	//	160503 bChange 들어 올때만 함수 동작해서 이상동작 하는 것 처럼 보여서 수정함
if( PRM_PCS[CTRL_TEST_MODE].iValue != 0 )
{
	if(PRM_PCS[SYS_REV_5].iValue == 1 )
	{
		GPIO_StaticSwitch(GPIO_STATIC_SW_ON);
	}
	else if( PRM_PCS[SYS_REV_5].iValue == 2)
	{
		GPIO_StaticSwitch(GPIO_STATIC_SW_OFF);
	}
}
#endif
#if 0
	if ( PRM_PCS[SYS_REV_2].bChange )
	{
		PRM_PCS[SYS_REV_2].bChange = FALSE;
	}

	if ( PRM_PCS[SYS_REV_3].bChange )
	{
		PRM_PCS[SYS_REV_3].bChange = FALSE;
	}

	if ( PRM_PCS[SYS_REV_4].bChange )
	{
		PRM_PCS[SYS_REV_4].bChange = FALSE;
	}


#endif


	if( PRM_PCS[SYS_OPTION].bChange )
	{
		PRM_PCS[SYS_OPTION].bChange = FALSE;
	}

	/*============================================================================*/

	if ( PRM_PCS[INV_TRANSFORMER].bChange )
	{
		//-130806 Delete.Model_UpdateParameter();
		EADC_VoltageAdcItemTangentInit();
		SYS_UpdateParameter();
		PRM_PCS[INV_TRANSFORMER].bChange = FALSE;
	}

	if ( PRM_PCS[INV_CAPACITY].bChange )
	{
		//-130806 Delete.Model_UpdateParameter();
		SYS_UpdateParameter();
		SYS_UpdateKpccKiccLi();
		BATT_UpdateParameter();

		EADC_CurrentAdcItemInit();
		EADC_VoltageAdcItemInit();
		EADC_VoltageAdcItemTangentInit();
		//-130806 Delete.MPP_UpdateParameter();
		CC_UpdateIdqRamp();
		CC_UpdateIdqDelayRamp();
		CC_InitLoadInfo(); // 13.12.6

		//-MO_SetPebbVarient();
		//-PARA_UpdateParameter(SYS_SLAVE_PEBB_ON_LOAD_OFFSET,  PRM_PCS[SYS_SLAVE_PEBB_ON_LOAD_OFFSET].iValue);
		//-PARA_UpdateParameter(SYS_SLAVE_PEBB_OFF_LOAD_OFFSET, PRM_PCS[SYS_SLAVE_PEBB_OFF_LOAD_OFFSET].iValue);


		PRM_PCS[INV_CAPACITY].bChange = FALSE;
	}

	if ( PRM_PCS[INV_OC_LEVEL].bChange )
	{
		PRM_PCS[INV_OC_LEVEL].bChange = FALSE;
	}

	if ( PRM_PCS[INV_CURRENT_LMT].bChange )
	{
		INVCTRL.fCurrentLimit = (float)PRM_PCS[INV_CURRENT_LMT].iValue * (float)PRM_PCS[INV_CURRENT_LMT].fIncDec;
		PRM_PCS[INV_CURRENT_LMT].bChange = FALSE;
	}

#if 1

	if ( PRM_PCS[INV_CAB_T_MAX].bChange )
	{
		if( PRM_PCS[INV_CAB_T_MAX].iValue == 10 )
			BUTTON.bPwmOnOff = OFF;
		if( PRM_PCS[INV_CAB_T_MAX].iValue == 20 )
			BUTTON.bPwmOnOff = ON;

		//-PRM_PCS[INV_CAB_T_MAX].iValue = 0;

		PRM_PCS[INV_CAB_T_MAX].bChange = FALSE;
	}

	if ( PRM_PCS[INV_CAB_T_MIN].bChange )
	{
		PRM_PCS[INV_CAB_T_MIN].bChange = FALSE;
	}
#endif
	/*============================================================================*/

	/*
	 * Grid parameter
	 */
	if ( PRM_PCS[GRID_RATED_VOLT].bChange )
	{
		SYS_UpdateParameter();
		CC_UpdateIdqRamp();
		CC_UpdateIdqDelayRamp();
		PRM_PCS[GRID_RATED_VOLT].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_RATED_FREQ].bChange )
	{
		SYS_UpdateParameter();
		CC_UpdateParameter();
		CTRL_FILTER_UpdateAPS();
		RMS_UpdateParameter();
		FASTRMS_UpdateParameter(); /* system memory err function */
		CTRL_BYP_SYNC_UpdateParameter();
		CTRL_INV_SYNC_UpdateParameter();
		CTRL_INV_PQC_UpdateParameter();
		CTRL_INV_PRC_UpdateParameter();
		CTRL_INV_DROOP_UpdateParameter();

		PRM_PCS[GRID_RATED_FREQ].bChange = FALSE;
	}

#if 0
	if ( PRM_PCS[GRID_FDPR_ENB].bChange )
	{
		PRM_PCS[GRID_FDPR_ENB].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_PWR_GRADIENT_ENB].bChange )
	{
		PRM_PCS[GRID_PWR_GRADIENT_ENB].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_GATE_WAY_ENB].bChange )
	{
		PRM_PCS[GRID_GATE_WAY_ENB].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_FRT_OV_ENB].bChange )
	{
		PRM_PCS[GRID_FRT_OV_ENB].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_FDPR_GRADIENT_LEVEL].bChange )
	{
		PRM_PCS[GRID_FDPR_GRADIENT_LEVEL].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_FDPR_DEACTIVATION_FREQ].bChange )
	{
		PRM_PCS[GRID_FDPR_DEACTIVATION_FREQ].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_RECONNECT_COND_ENB].bChange )
	{
		PRM_PCS[GRID_RECONNECT_COND_ENB].bChange = FALSE;
	}
#endif

	/*
	 * Set Protection level
	 */
	if ( PRM_PCS[GRID_OV_LEVEL1].bChange || PRM_PCS[GRID_OV_LEVEL1_TRIP_TIME].bChange )
	{
		SYS_UpdateLevel2ProtectionLevel(SYS_GRID_OV_LEVEL1, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		//-FRT_UpdateParameter();

		PRM_PCS[GRID_OV_LEVEL1_TRIP_TIME].bChange = FALSE;
		PRM_PCS[GRID_OV_LEVEL1].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_UV_LEVEL1].bChange || PRM_PCS[GRID_UV_LEVEL1_TRIP_TIME].bChange )
	{
		SYS_UpdateLevel2ProtectionLevel(SYS_GRID_UV_LEVEL1, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		PRM_PCS[GRID_UV_LEVEL1].bChange = FALSE;
		PRM_PCS[GRID_UV_LEVEL1_TRIP_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_OF_LEVEL1].bChange || PRM_PCS[GRID_OF_LEVEL1_TRIP_TIME].bChange )
	{
		SYS_UpdateLevel2ProtectionLevel(SYS_GRID_OF_LEVEL1, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		PRM_PCS[GRID_OF_LEVEL1].bChange = FALSE;
		PRM_PCS[GRID_OF_LEVEL1_TRIP_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_UF_LEVEL1].bChange || PRM_PCS[GRID_UF_LEVEL1_TRIP_TIME].bChange )
	{

		SYS_UpdateLevel2ProtectionLevel(SYS_GRID_UF_LEVEL1, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		PRM_PCS[GRID_UF_LEVEL1].bChange = FALSE;
		PRM_PCS[GRID_UF_LEVEL1_TRIP_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_OF_LEVEL1].bChange || PRM_PCS[GRID_OF_LEVEL1_TRIP_TIME].bChange )
		{
			SYS_UpdateLevel2ProtectionLevel(SYS_GRID_OF_LEVEL1, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
			PRM_PCS[GRID_OF_LEVEL1].bChange = FALSE;
			PRM_PCS[GRID_OF_LEVEL1_TRIP_TIME].bChange = FALSE;
	}



	/*
	 * Power Gradient
	 */
	if ( PRM_PCS[GRID_PWR_GRADIENT_RAMP].bChange )
	{
		PRM_PCS[GRID_PWR_GRADIENT_RAMP].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_TIME_SHIFT].bChange )
	{
		ODT_HoldTimeChange(INVERTER.odtGI_SlaveDelay, (PRM_PCS[GRID_TIME_SHIFT].iValue * 1000));
		PRM_PCS[GRID_TIME_SHIFT].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].bChange )
	{
		SYS_SetLevel2Protection(PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_OV_LEVEL2].bChange || PRM_PCS[GRID_OV_LEVEL2_TRIP_TIME].bChange )
	{
		SYS_UpdateLevel2ProtectionLevel(SYS_GRID_OV_LEVEL2, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		if (PARAM_RAW_VAL(GRID_LEVEL2_PROTECTION_ENABLE)){
			SYS_UpdateLevel2ProtectionLevel(SYS_INV_OV_LEVEL1, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		}
		PRM_PCS[GRID_OV_LEVEL2].bChange = FALSE;
		PRM_PCS[GRID_OV_LEVEL2_TRIP_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_UV_LEVEL2].bChange || PRM_PCS[GRID_UV_LEVEL2_TRIP_TIME].bChange )
	{
		SYS_UpdateLevel2ProtectionLevel(SYS_GRID_UV_LEVEL2, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		if (PARAM_RAW_VAL(GRID_LEVEL2_PROTECTION_ENABLE)){
			SYS_UpdateLevel2ProtectionLevel(SYS_INV_UV_LEVEL1, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		}
		PRM_PCS[GRID_UV_LEVEL2].bChange = FALSE;
		PRM_PCS[GRID_UV_LEVEL2_TRIP_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_OF_LEVEL2].bChange || PRM_PCS[GRID_OF_LEVEL2_TRIP_TIME].bChange )
	{
		SYS_UpdateLevel2ProtectionLevel(SYS_GRID_OF_LEVEL2, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		if (PARAM_RAW_VAL(GRID_LEVEL2_PROTECTION_ENABLE)){
			SYS_UpdateLevel2ProtectionLevel(SYS_INV_OF_LEVEL1, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		}
		PRM_PCS[GRID_OF_LEVEL2].bChange = FALSE;
		PRM_PCS[GRID_OF_LEVEL2_TRIP_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_UF_LEVEL2].bChange || PRM_PCS[GRID_UF_LEVEL2_TRIP_TIME].bChange )
	{
		SYS_UpdateLevel2ProtectionLevel(SYS_GRID_UF_LEVEL2, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		if (PARAM_RAW_VAL(GRID_LEVEL2_PROTECTION_ENABLE)){
			SYS_UpdateLevel2ProtectionLevel(SYS_INV_UF_LEVEL1, PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1);
		}
		PRM_PCS[GRID_UF_LEVEL2].bChange = FALSE;
		PRM_PCS[GRID_UF_LEVEL2_TRIP_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_FRT_ENB].bChange )
	{
		if (PARAM_RAW_VAL(GRID_FRT_ENB)){
			//-130806 TODO Check.PARA_plugInSpare(PRM_PCS, paraFRT, MODEL_PARA_FRT);
		}else{
			//-130806 TODO Check.PARA_initSparePos(PRM_PCS, paraFRT, MODEL_PARA_FRT);
		}
		PRM_PCS[GRID_FRT_ENB].bChange = FALSE;
	}

	if ( PRM_PCS[TIME_YEAR].bChange )
	{
		PRM_PCS[TIME_YEAR].bChange = FALSE;
	}

	if ( PRM_PCS[TIME_MONTH].bChange )
	{
		PRM_PCS[TIME_MONTH].bChange = FALSE;
	}

	if ( PRM_PCS[TIME_DAY].bChange )
	{
		PRM_PCS[TIME_DAY].bChange = FALSE;
	}

	if ( PRM_PCS[TIME_HOUR].bChange )
	{
		PRM_PCS[TIME_HOUR].bChange = FALSE;
	}

	if ( PRM_PCS[TIME_MINUTE].bChange )
	{
		PRM_PCS[TIME_MINUTE].bChange = FALSE;
	}

	if ( PRM_PCS[TIME_SECOND].bChange )
	{
		PRM_PCS[TIME_SECOND].bChange = FALSE;
	}


#if 1
	if ( PRM_PCS[DGT_DI1].bChange )
	{
		// GI-모드 Master Slave 설정
		PRM_PCS[DGT_DI1].bChange = FALSE;
	}

	if ( PRM_PCS[DGT_DO1].bChange )
	{
		// 시물레이션 모드 SW에서 Grid 주파수 증감.

		if( PRM_PCS[CTRL_TEST_MODE].iValue != 0 )
			CAN_GBI_FanSpeed( PRM_PCS[DGT_DO1].iValue );


#if DBUG_MODE == 0
#else
		SYS_UpdateParameter();
		CTRL_INV_DROOP_UpdateParameter();
#endif

		PRM_PCS[DGT_DO1].bChange = FALSE;
	}

	if ( PRM_PCS[DGT_PWR_CTRL_MODE].bChange )
	{
		if( PRM_PCS[DGT_PWR_CTRL_MODE].iValue == 1 )
			CAN_GBI_ManualOperation(TRUE);
		else
			CAN_GBI_ManualOperation(FALSE);

		PRM_PCS[DGT_PWR_CTRL_MODE].bChange = FALSE;
	}

	if ( PRM_PCS[DGT_CAN_ID].bChange )
	{
		PRM_PCS[DGT_CAN_ID].bChange = FALSE;
	}

	if ( PRM_PCS[DGT_REV_00].bChange )
	{
		PRM_PCS[DGT_REV_00].bChange = FALSE;
	}
#endif

//	if ( PRM_PCS[DGT_MOD_RTU_ID].bChange )
//	{
//		Modbus232_SetId( PRM_PCS[DGT_MOD_RTU_ID].iValue );
//		PRM_PCS[DGT_MOD_RTU_ID].bChange = FALSE;
//	}

	/*
	 * Analog Offset and Gain
	 */
	if ( PRM_PCS[ANL_AI1_OFFSET].bChange )
	{
		PRM_PCS[ANL_AI1_OFFSET].bChange = FALSE;
	}

	if ( PRM_PCS[ANL_AI1_GAIN].bChange )
	{
#if PARA_VERSION < 0x1003
		EADC.pAdcBypEa->fGain=PRM_PCS[ANL_AI1_GAIN].iValue*PRM_PCS[ANL_AI1_GAIN].fIncDec;
#endif
		PRM_PCS[ANL_AI1_GAIN].bChange = FALSE;
	}

	if ( PRM_PCS[ANL_AI2_OFFSET].bChange )
	{
		PRM_PCS[ANL_AI2_OFFSET].bChange = FALSE;
	}

	if ( PRM_PCS[ANL_AI2_GAIN].bChange )
	{
#if PARA_VERSION < 0x1003
		EADC.pAdcBypEb->fGain=PRM_PCS[ANL_AI2_GAIN].iValue*PRM_PCS[ANL_AI1_GAIN].fIncDec;
#endif
		PRM_PCS[ANL_AI2_GAIN].bChange = FALSE;
	}

	if ( PRM_PCS[ANL_AI3_OFFSET].bChange )
	{
		PRM_PCS[ANL_AI3_OFFSET].bChange = FALSE;
	}

	if ( PRM_PCS[ANL_AI3_GAIN].bChange )
	{
#if PARA_VERSION < 0x1003
		EADC.pAdcBypEc->fGain=PRM_PCS[ANL_AI3_GAIN].iValue*PRM_PCS[ANL_AI1_GAIN].fIncDec;
#endif
		PRM_PCS[ANL_AI3_GAIN].bChange = FALSE;
	}

	if ( PRM_PCS[ANL_AI4_OFFSET].bChange )
	{
		PRM_PCS[ANL_AI4_OFFSET].bChange = FALSE;
	}

	if ( PRM_PCS[ANL_AI4_GAIN].bChange )
	{
		PRM_PCS[ANL_AI4_GAIN].bChange = FALSE;
	}

	/*
	 * Control Parameter
	 */
	if ( PRM_PCS[CTRL_VC_P_GAIN].bChange )
	{
		CVC_UpdateGains();
		PRM_PCS[CTRL_VC_P_GAIN].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_VC_I_GAIN].bChange )
	{
		CVC_UpdateGains();
		PRM_PCS[CTRL_VC_I_GAIN].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_VOLT_LPF].bChange )
	{
//		CC_UpdateVoltageFilterCoefficient();
		CC_CalcVoltageFilterShadow();
		PRM_PCS[CTRL_VOLT_LPF].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_CC_P_GAIN].bChange )
	{
		//-TODO CHECK
		// SYS_UpdateKpccKiccLi() 에서 Gain을 계산 하므로 수동 입력 하지 않도록 함.
		//CC_UpdateGains();
		PRM_PCS[CTRL_CC_P_GAIN].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_CC_I_GAIN].bChange )
	{
		//-TODO CHECK
		// SYS_UpdateKpccKiccLi() 에서 Gain을 계산 하므로 수동 입력 하지 않도록 함.
		//-CC_UpdateGains();
		PRM_PCS[CTRL_CC_I_GAIN].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_CC_DI_DT].bChange )
	{
		CVC_UpdateParameter();
		PRM_PCS[CTRL_CC_DI_DT].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_RAMP].bChange )
	{
		CVC_UpdateParameter();
		PRM_PCS[CTRL_RAMP].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_LI].bChange )
	{
		SYS_UpdateLiViaParam((float)PRM_PCS[CTRL_LI].iValue);
		PRM_PCS[CTRL_LI].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_REV_00].bChange )
	{
		PRM_PCS[CTRL_REV_00].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_QV_CTRL_RAMP_TIME].bChange || PRM_PCS[CTRL_FRT_IQ_RAMP_TIME].bChange || PRM_PCS[CTRL_COSPHI_P_PN_RAMP_TIME].bChange)
	{
		CC_UpdateIdqDelayRamp();
		PRM_PCS[CTRL_QV_CTRL_RAMP_TIME].bChange = FALSE;
		PRM_PCS[CTRL_FRT_IQ_RAMP_TIME].bChange = FALSE;
		PRM_PCS[CTRL_COSPHI_P_PN_RAMP_TIME].bChange = FALSE;

	}

	if ( PRM_PCS[CTRL_CC_PERIOD].bChange )
	{
		CC_UpdateParameter();

		/*
		 * Filter coefficient shadow
		 */
		CC_CalcVoltageFilterShadow();
		CC_CalcFilterShadow();
		CC_CalcFsRmsFilterShadow();
		CC_CalcFreqFilterShadow();
		CC_CalcPowerControlFilterShadow();

		SYS_UpdateKpccKiccLi();
		CC_UpdateGains();
		CVC_UpdateParameter();
		CVC_UpdateGains();
		FASTRMS_UpdateParameter();

		/* FIXME : check onece more */
		CTRL_FILTER_UpdateAPS();

		CTRL_INV_DROOP_UpdateRamp();
		CTRL_INV_PQC_UpdateParameterRamp();
		CTRL_INV_PQC_UpdateParameterPIGain();
		CTRL_INV_VUC_UpdateParameterPIGain();
		CTRL_INV_PRC_UpdateParameter();
		CTRL_INV_SYNC_UpdateParameter();
		CTRL_BYP_SYNC_Create();

		PRM_PCS[CTRL_CC_PERIOD].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_PLL_P_GAIN].bChange )
	{
		CC_UpdateGains();
		PRM_PCS[CTRL_PLL_P_GAIN].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_PLL_I_GAIN].bChange )
	{
		CC_UpdateGains();
		PRM_PCS[CTRL_PLL_I_GAIN].bChange = FALSE;
	}


	if ( PRM_PCS[CTRL_AUT_FAULT_RST].bChange )
	{
		PRM_PCS[CTRL_AUT_FAULT_RST].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_PEBB_HS_OVERTEMP].bChange )
	{
		PRM_PCS[CTRL_PEBB_HS_OVERTEMP].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_TEST_MODE].bChange )
	{
		PRM_PCS[CTRL_TEST_MODE].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_INV_IIR1ST_P].bChange )
	{
		CTRL_INV_DROOP_UpdateParameter();
		PRM_PCS[CTRL_INV_IIR1ST_P].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_INV_IIR1ST_Q].bChange )
	{
		CTRL_INV_DROOP_UpdateParameter();
		PRM_PCS[CTRL_INV_IIR1ST_Q].bChange = FALSE;
	}


	if ( PRM_PCS[CTRL_OPTION].bChange )
	{
		CTRL_UpdateOptions();
		PRM_PCS[CTRL_OPTION].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_PWR_LIMIT_RAMP].bChange )
	{
		PRM_PCS[CTRL_PWR_LIMIT_RAMP].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_GRID_FS_RMS_CUTOFF_HZ].bChange || PRM_PCS[CTRL_INV_FS_RMS_CUTOFF_HZ].bChange){
		CC_CalcFsRmsFilterShadow();
		PRM_PCS[CTRL_GRID_FS_RMS_CUTOFF_HZ].bChange = FALSE;
		PRM_PCS[CTRL_INV_FS_RMS_CUTOFF_HZ].bChange = FALSE;
	}

	if ( PRM_PCS[COSPHI_FACTOR_LIMIT].bChange){

		if( PRM_PCS[COSPHI_FACTOR_LIMIT].iValue > 100 )
		{
			PRM_PCS[COSPHI_FACTOR_LIMIT].iValue = 100;
		}
		else if(PRM_PCS[COSPHI_FACTOR_LIMIT].iValue < 80)
		{
			PRM_PCS[COSPHI_FACTOR_LIMIT].iValue = 80;
		}

		PRM_PCS[COSPHI_FACTOR_LIMIT].bChange = FALSE;
	}


	if ( PRM_PCS[CTRL_FREQ_CUTOFF_HZ].bChange ){
		CC_CalcFreqFilterShadow();
		PRM_PCS[CTRL_FREQ_CUTOFF_HZ].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_VOLT_DEVIATION_TIME].bChange ){
		SYS_SetLevel2Protection(PRM_PCS[GRID_LEVEL2_PROTECTION_ENABLE].iValue == 1); /*+ 150930 */
		PRM_PCS[CTRL_VOLT_DEVIATION_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_REACTIVE_POWER].bChange ){
		PRM_PCS[CTRL_REACTIVE_POWER].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_BYP_V_GRID_TEST_MODE].bChange ){
		PRM_PCS[CTRL_BYP_V_GRID_TEST_MODE].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_TEST_PWM_VQE_REF].bChange ){
		PRM_PCS[CTRL_TEST_PWM_VQE_REF].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_REV_04_1].bChange ){
		PRM_PCS[CTRL_REV_04_1].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_CURR_UNBAL_LIMIT].bChange ){
		PRM_PCS[CTRL_CURR_UNBAL_LIMIT].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_REV_05].bChange ){
		PRM_PCS[CTRL_REV_05].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_SYNC_PI_MAG_P].bChange || PRM_PCS[CTRL_SYNC_PI_MAG_I].bChange )
	{
		CTRL_INV_SYNC_UpdateParameter();
		PRM_PCS[CTRL_SYNC_PI_MAG_P].bChange = FALSE;
		PRM_PCS[CTRL_SYNC_PI_MAG_I].bChange = FALSE;
	}

	// Grid_MC_ON
#if IIVC_ENB
	if ( PRM_PCS[CTRL_IIVCD_P_GAIN].bChange )
	{
		CC_UpdateIINVGains();
		PRM_PCS[CTRL_IIVCD_P_GAIN].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_IIVCD_I_GAIN].bChange )
	{
		CC_UpdateIINVGains();
		PRM_PCS[CTRL_IIVCD_I_GAIN].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_IIVCQ_P_GAIN].bChange )
	{
		CC_UpdateIINVGains();
		PRM_PCS[CTRL_IIVCQ_P_GAIN].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_IIVCQ_I_GAIN].bChange )
	{
		CC_UpdateIINVGains();
		PRM_PCS[CTRL_IIVCQ_I_GAIN].bChange = FALSE;
	}
#endif
	if ( PRM_PCS[CTRL_IINV_VOLTAGE_BUILDUP_TIME].bChange ){
		ODT_HoldTimeChange(INVERTER.odtIinvBuildup, PARAM_RAW_VAL(CTRL_IINV_VOLTAGE_BUILDUP_TIME));
#if IIVC_ENB
		CC_UpdateIINVParameter();
#endif
		PRM_PCS[CTRL_IINV_VOLTAGE_BUILDUP_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_IINV_PLL_STABILIZE_TIME].bChange )
	{
		ODT_HoldTimeChange(INVERTER.odtIinvSync, PARAM_RAW_VAL(CTRL_IINV_PLL_STABILIZE_TIME));
		PRM_PCS[CTRL_IINV_PLL_STABILIZE_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_IINV_SYNC_TOLERANCE_THETA].bChange )
	{
		CC_UpdateSyncToleranceTheta();
		CTRL_BYP_SYNC_Create();
		CTRL_GEN_Create();
		PRM_PCS[CTRL_IINV_SYNC_TOLERANCE_THETA].bChange = FALSE;
	}


	if ( PRM_PCS[CTRL_SEAMLESS_CNT].bChange )
	{
		CTRL_INV_SEAMLESS_UpdateParameter_running();
		PRM_PCS[CTRL_SEAMLESS_CNT].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_SEAMLESS_OV_CNT_STATUS].bChange )
	{
		PRM_PCS[CTRL_SEAMLESS_OV_CNT_STATUS].bChange = FALSE;
	}

	if (PRM_PCS[CTRL_ENABLE_ANTI_ISLANDING].bChange)
	{
		PRM_PCS[CTRL_ENABLE_ANTI_ISLANDING].bChange = FALSE;
	}

	if (PRM_PCS[CTRL_REV_REMOTEPLIMIT].bChange)
	{
		PRM_PCS[CTRL_REV_REMOTEPLIMIT].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_COSPHI_POWER_FACTOR_RPC].bChange ||	PRM_PCS[CTRL_COSPHI_REACTIVE_POWER_RPC].bChange )
	{
		#if COSPHI_ENB == 1
		Cosphi.Flag = TRUE;
		#endif

		PRM_PCS[CTRL_COSPHI_POWER_FACTOR_RPC].bChange = FALSE;
		PRM_PCS[CTRL_COSPHI_REACTIVE_POWER_RPC].bChange = FALSE;

	}

	if (PRM_PCS[CTRL_COSPHI_CONTROL_MODE].bChange)
	{
		#if COSPHI_ENB == 1
		if( PARAM_RAW_VAL(CTRL_COSPHI_CONTROL_MODE) == COSPHI_MODE_P_PN_PF)
		{
			COSPHI_P_PN_UpdateParameter();
		}
		#endif
		PRM_PCS[CTRL_COSPHI_CONTROL_MODE].bChange = FALSE;
	}

#if 0
	if ( PRM_PCS[CTRL_COSPHI_STRAY_RATIO].bChange ||
			PRM_PCS[CTRL_QV_CTRL_TARGET_V].bChange ||
			PRM_PCS[CTRL_QV_CTRL_K_FACTOR].bChange ||
			PRM_PCS[CTRL_QV_CTRL_DEADBAND].bChange ){

			PRM_PCS[CTRL_COSPHI_STRAY_RATIO].bChange = FALSE;
			PRM_PCS[CTRL_QV_CTRL_TARGET_V].bChange = FALSE;
			PRM_PCS[CTRL_QV_CTRL_K_FACTOR].bChange = FALSE;
			PRM_PCS[CTRL_QV_CTRL_DEADBAND].bChange = FALSE;

			COSPHI_P_PN_UpdateParameter();
	}

	if ( PRM_PCS[CTRL_APS_LINE_DEADBAND].bChange )
	{
		PRM_PCS[CTRL_APS_LINE_DEADBAND].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_REACTIVE_POWER_LIMIT].bChange )
	{
		PRM_PCS[CTRL_REACTIVE_POWER_LIMIT].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_ANTI_ISLANDING_K_FACTOR].bChange )
	{
		PRM_PCS[CTRL_ANTI_ISLANDING_K_FACTOR].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_PD_ENABLE].bChange )
	{
		PRM_PCS[CTRL_PD_ENABLE].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_PD_ENABLE_TEMP].bChange )
	{
		PRM_PCS[CTRL_PD_ENABLE_TEMP].bChange = FALSE;
	}

	if ( PRM_PCS[CTRL_PD_DISABLE_TEMP].bChange )
	{
		PRM_PCS[CTRL_PD_DISABLE_TEMP].bChange = FALSE;
	}
#endif

	/*
	 * Trace
	 */
	if ( PRM_PCS[TRC_TRACE1].bChange )
	{
		TRC_ChangeTracePoolNode( 0,  PRM_PCS[TRC_TRACE1].iValue );
		PRM_PCS[TRC_TRACE1].bChange = FALSE;
	}

	if ( PRM_PCS[TRC_TRACE2].bChange )
	{
		TRC_ChangeTracePoolNode( 1,  PRM_PCS[TRC_TRACE2].iValue );
		PRM_PCS[TRC_TRACE2].bChange = FALSE;
	}

	if ( PRM_PCS[TRC_TRACE3].bChange )
	{
		TRC_ChangeTracePoolNode( 2,  PRM_PCS[TRC_TRACE3].iValue );
		PRM_PCS[TRC_TRACE3].bChange = FALSE;
	}

	if ( PRM_PCS[TRC_TRACE4].bChange )
	{
		TRC_ChangeTracePoolNode( 3,  PRM_PCS[TRC_TRACE4].iValue );
		PRM_PCS[TRC_TRACE4].bChange = FALSE;
	}

	if ( PRM_PCS[TRC_TRACE5].bChange )
	{
		TRC_ChangeTracePoolNode( 4,  PRM_PCS[TRC_TRACE5].iValue );
		PRM_PCS[TRC_TRACE5].bChange = FALSE;
	}

	if ( PRM_PCS[TRC_TRACE6].bChange )
	{
		TRC_ChangeTracePoolNode( 5,  PRM_PCS[TRC_TRACE6].iValue );
		PRM_PCS[TRC_TRACE6].bChange = FALSE;
	}

	if ( PRM_PCS[TRC_TRACE7].bChange )
	{
		TRC_ChangeTracePoolNode( 6,  PRM_PCS[TRC_TRACE7].iValue );
		PRM_PCS[TRC_TRACE7].bChange = FALSE;
	}

	if ( PRM_PCS[TRC_TRACE8].bChange )
	{
		TRC_ChangeTracePoolNode( 7,  PRM_PCS[TRC_TRACE8].iValue );
		PRM_PCS[TRC_TRACE8].bChange = FALSE;
	}

	if ( PRM_PCS[TRC_TRACE9].bChange )
	{
		TRC_ChangeTracePoolNode( 8,  PRM_PCS[TRC_TRACE9].iValue );
		PRM_PCS[TRC_TRACE9].bChange = FALSE;
	}

	if ( PRM_PCS[TRC_TRACE10].bChange )
	{
		TRC_ChangeTracePoolNode( 9,  PRM_PCS[TRC_TRACE10].iValue );
		PRM_PCS[TRC_TRACE10].bChange = FALSE;
	}

	if ( PRM_PCS[TRC_TRACE11].bChange )
	{
		TRC_ChangeTracePoolNode( 10,  PRM_PCS[TRC_TRACE11].iValue );
		PRM_PCS[TRC_TRACE11].bChange = FALSE;
	}

	if ( PRM_PCS[TRC_TRACE12].bChange )
	{
		TRC_ChangeTracePoolNode( 11,  PRM_PCS[TRC_TRACE12].iValue );
		PRM_PCS[TRC_TRACE12].bChange = FALSE;
	}

    if ( PRM_PCS[TRC_TRACE_MODE].bChange )
	{
		PRM_PCS[TRC_TRACE_MODE].bChange = FALSE;
	}


	if ( PRM_PCS[TRC_TRACE_SAMPLING].bChange )
	{
		TRC_UpdateParameter();
		PRM_PCS[TRC_TRACE_SAMPLING].bChange = FALSE;
	}

	/*
	 * ADC Offset
	 */
	if ( PRM_PCS[OFS_DCLINK_VOLTAGE].bChange )
	{
		EADC.pAdcDCLinkV->fOffset=PRM_PCS[OFS_DCLINK_VOLTAGE].iValue*PRM_PCS[OFS_DCLINK_VOLTAGE].fIncDec;
		PRM_PCS[OFS_DCLINK_VOLTAGE].bChange = FALSE;
	}

	if ( PRM_PCS[OFS_BAT_CURRENT].bChange )
	{
		EADC.pAdcBatI->fOffset=PRM_PCS[OFS_BAT_CURRENT].iValue*PRM_PCS[OFS_BAT_CURRENT].fIncDec;
		PRM_PCS[OFS_BAT_CURRENT].bChange = FALSE;
	}

	if ( PRM_PCS[OFS_BATT_VOLTAGE].bChange )
	{
		EADC.pAdcDCBattV->fOffset=PRM_PCS[OFS_BATT_VOLTAGE].iValue*PRM_PCS[OFS_BATT_VOLTAGE].fIncDec;
		PRM_PCS[OFS_BATT_VOLTAGE].bChange = FALSE;
	}

	if ( PRM_PCS[OFS_INV_I_L1].bChange )
	{
		EADC.pAdcInvIa->fOffset=PRM_PCS[OFS_INV_I_L1].iValue*PRM_PCS[OFS_INV_I_L1].fIncDec;
		PRM_PCS[OFS_INV_I_L1].bChange = FALSE;
	}

	if ( PRM_PCS[OFS_INV_I_L3].bChange )
	{
		EADC.pAdcInvIc->fOffset=PRM_PCS[OFS_INV_I_L3].iValue*PRM_PCS[OFS_INV_I_L3].fIncDec;
		PRM_PCS[OFS_INV_I_L3].bChange = FALSE;
	}

	if ( PRM_PCS[OFS_INV_I_L2].bChange )
	{
		EADC.pAdcInvIb->fOffset=PRM_PCS[OFS_INV_I_L2].iValue*PRM_PCS[OFS_INV_I_L2].fIncDec;
		PRM_PCS[OFS_INV_I_L2].bChange = FALSE;
	}

	if ( PRM_PCS[OFS_STACK_TEMP].bChange )
	{
		PRM_PCS[OFS_STACK_TEMP].bChange = FALSE;
	}

	if ( PRM_PCS[OFS_BATT_CHG_CURRENT].bChange )
	{
		PRM_PCS[OFS_BATT_CHG_CURRENT].bChange = FALSE;
	}

	if ( PRM_PCS[OFS_CAB_TEMP].bChange )
	{
		PRM_PCS[OFS_CAB_TEMP].bChange = FALSE;
	}


	/*
	 * ADC Gain
	 */
	if ( PRM_PCS[GN_DCLINK_VOLTAGE].bChange )
	{
		EADC.pAdcDCLinkV->fGain=PRM_PCS[GN_DCLINK_VOLTAGE].iValue*PRM_PCS[GN_DCLINK_VOLTAGE].fIncDec;
		PRM_PCS[GN_DCLINK_VOLTAGE].bChange = FALSE;
	}

	if ( PRM_PCS[GN_BAT_CURRENT].bChange )
	{
		EADC.pAdcBatI->fGain=PRM_PCS[GN_BAT_CURRENT].iValue*PRM_PCS[GN_BAT_CURRENT].fIncDec;
		PRM_PCS[GN_BAT_CURRENT].bChange = FALSE;
	}

	if ( PRM_PCS[GN_BAT_VOLTAGE].bChange )
	{
		EADC.pAdcDCBattV->fGain=PRM_PCS[GN_BAT_VOLTAGE].iValue*PRM_PCS[GN_BAT_VOLTAGE].fIncDec;
		PRM_PCS[GN_BAT_VOLTAGE].bChange = FALSE;
	}

	if ( PRM_PCS[GN_INV_V_L1].bChange )
	{
		EADC.pAdcInvEa->fGain = PRM_PCS[GN_INV_V_L1].iValue*PRM_PCS[GN_INV_V_L1].fIncDec;
		PRM_PCS[GN_INV_V_L1].bChange = FALSE;
	}

	if ( PRM_PCS[GN_INV_V_L2].bChange )
	{
		EADC.pAdcInvEb->fGain=PRM_PCS[GN_INV_V_L2].iValue*PRM_PCS[GN_INV_V_L2].fIncDec;
		PRM_PCS[GN_INV_V_L2].bChange = FALSE;
	}

	if ( PRM_PCS[GN_INV_V_L3].bChange )
	{
		EADC.pAdcInvEc->fGain=PRM_PCS[GN_INV_V_L3].iValue*PRM_PCS[GN_INV_V_L3].fIncDec;
		PRM_PCS[GN_INV_V_L3].bChange = FALSE;
	}

	if ( PRM_PCS[GN_INV_I_L1].bChange )
	{
		EADC.pAdcInvIa->fGain=PRM_PCS[GN_INV_I_L1].iValue*PRM_PCS[GN_INV_I_L1].fIncDec;
		PRM_PCS[GN_INV_I_L1].bChange = FALSE;
	}

	if ( PRM_PCS[GN_INV_I_L3].bChange )
	{
		EADC.pAdcInvIc->fGain=PRM_PCS[GN_INV_I_L3].iValue*PRM_PCS[GN_INV_I_L3].fIncDec;
		PRM_PCS[GN_INV_I_L3].bChange = FALSE;
	}

	if ( PRM_PCS[GN_GRID_V_L1].bChange )
	{
		EADC.pAdcOutEa->fGain=PRM_PCS[GN_GRID_V_L1].iValue*PRM_PCS[GN_GRID_V_L1].fIncDec;
		PRM_PCS[GN_GRID_V_L1].bChange = FALSE;
	}

	if ( PRM_PCS[GN_GRID_V_L2].bChange )
	{
		EADC.pAdcOutEb->fGain=PRM_PCS[GN_GRID_V_L2].iValue*PRM_PCS[GN_GRID_V_L2].fIncDec;
		PRM_PCS[GN_GRID_V_L2].bChange = FALSE;
	}

	if ( PRM_PCS[GN_GRID_V_L3].bChange )
	{
		EADC.pAdcOutEc->fGain=PRM_PCS[GN_GRID_V_L3].iValue*PRM_PCS[GN_GRID_V_L3].fIncDec;
		PRM_PCS[GN_GRID_V_L3].bChange = FALSE;
	}

	// ISO --> I L2
	if ( PRM_PCS[GN_INV_I_L2].bChange )
	{
		EADC.pAdcInvIb->fGain=PRM_PCS[GN_INV_I_L2].iValue*PRM_PCS[GN_INV_I_L2].fIncDec;
		PRM_PCS[GN_INV_I_L2].bChange = FALSE;
	}

	// Grid I a
	if ( PRM_PCS[GN_GRID_I_L1].bChange )
	{
		//-EADC.pPebbTemp[0]->fGain=PRM_PCS[GN_STACK_TEMP].iValue*PRM_PCS[GN_STACK_TEMP].fIncDec;
		EADC.pAdcTrIa->fGain=PRM_PCS[GN_GRID_I_L1].iValue*PRM_PCS[GN_GRID_I_L1].fIncDec;
		PRM_PCS[GN_GRID_I_L1].bChange = FALSE;
	}

	// Grid I b
	if ( PRM_PCS[GN_GRID_I_L2].bChange )
	{
		//-EADC.pCabTemp->fGain=PRM_PCS[GN_CAB_TEMP].iValue*PRM_PCS[GN_CAB_TEMP].fIncDec;
		EADC.pAdcTrIb->fGain=PRM_PCS[GN_GRID_I_L2].iValue*PRM_PCS[GN_GRID_I_L2].fIncDec;
		PRM_PCS[GN_GRID_I_L2].bChange = FALSE;
	}

	// Grid I c
	if ( PRM_PCS[GN_GRID_I_L3].bChange )
	{
		EADC.pAdcTrIc->fGain=PRM_PCS[GN_GRID_I_L3].iValue*PRM_PCS[GN_GRID_I_L3].fIncDec;
		PRM_PCS[GN_GRID_I_L3].bChange = FALSE;
	}

	if ( PRM_PCS[BATT_OC_LEVEL].bChange)
	{
		BATT_UpdateParameter();
		PRM_PCS[BATT_OC_LEVEL].bChange = FALSE;
	}

	if ( PRM_PCS[BATT_DC_SIDE_V_ABNORMAL_LEVEL].bChange )
	{
		PRM_PCS[BATT_DC_SIDE_V_ABNORMAL_LEVEL].bChange = FALSE;
	}

	if (PRM_PCS[BATT_OV_LEVEL].bChange)
	{
		PRM_PCS[BATT_OV_LEVEL].bChange = FALSE;
	}

	if ( PRM_PCS[BATT_UV_LEVEL].bChange )
	{
		//-130806 Delete.MPP_UpdateParameter();
		BATT_UpdateParameter();

		PRM_PCS[BATT_UV_LEVEL].bChange = FALSE;
	}

	if ( PRM_PCS[BATT_V_RANGE_MIN].bChange)
	{
		BATT_UpdateParameter();
		PRM_PCS[BATT_V_RANGE_MIN].bChange = FALSE;
	}
	if ( PRM_PCS[BATT_IVC_VDC_RAMP].bChange)
	{
		/*
		 * {BATT_VDC_RAMP} means A/sec
		 * FSM will be called every 5msec
		 */
		//-130806 TODO Check.RAMP_SetDelta(&BC_fsm.currentLimitRamp, PARAM_VAL(BATT_VDC_RAMP)*0.005);

		//13.3.24
		CVC_UpdateParameter();
		PRM_PCS[BATT_IVC_VDC_RAMP].bChange = FALSE;
	}


	// 13.3.24
	if (PRM_PCS[BATT_ICC_P_GAIN].bChange || PRM_PCS[BATT_ICC_I_GAIN].bChange)
	{
		CVC_UpdateGains();
		PRM_PCS[BATT_ICC_P_GAIN].bChange = PRM_PCS[BATT_ICC_I_GAIN].bChange = FALSE;
	}

	// 130624 @ PCC PI Gain
	if (PRM_PCS[BATT_PCC_P_GAIN].bChange || PRM_PCS[BATT_PCC_I_GAIN].bChange)
	{
		CVC_UpdateGains();
		PRM_PCS[BATT_PCC_P_GAIN].bChange = PRM_PCS[BATT_PCC_I_GAIN].bChange = FALSE;
	}

	// 130626 @ PCC Cut off Hz
	if (PRM_PCS[BATT_PCC_PQ_CUTOFF_HZ].bChange)
	{
		CC_CalcPowerControlFilterShadow();
		PRM_PCS[BATT_PCC_PQ_CUTOFF_HZ].bChange = FALSE;
	}

	if ( PRM_PCS[RESYNC_W_PI_MAX].bChange)
	{
		PRM_PCS[RESYNC_W_PI_MAX].bChange = FALSE;
	}

	if ( PRM_PCS[GEN_CONNECT_CHK_STX_POWER].bChange)
	{
		SYS_SetGeneratorConnectPredict();
		PRM_PCS[GEN_CONNECT_CHK_STX_POWER].bChange = FALSE;
	}

	if ( PRM_PCS[GEN_CONNECT_I_DIDT].bChange)
	{
		SYS_SetGeneratorConnectPredict();
		PRM_PCS[GEN_CONNECT_I_DIDT].bChange = FALSE;
	}

	if (PRM_PCS[BATT_ORDER_SOURCE].bChange)
	{
		CC_BCStateUpdate();
		PRM_PCS[BATT_ORDER_SOURCE].bChange = FALSE;
	}

	if (PRM_PCS[BATT_REF_VAL_MODE].bChange)
	{
		PRM_PCS[BATT_REF_VAL_MODE].bChange = FALSE;
	}


	if (PRM_PCS[BATT_LOCAL_CONTROL_MODE].bChange)
	{
		CC_BCStateUpdate();
		PRM_PCS[BATT_LOCAL_CONTROL_MODE].bChange = FALSE;
	}

	if (PRM_PCS[BATT_LOCAL_POWER_FLOW].bChange)
	{
		CC_BCStateUpdate();
		PRM_PCS[BATT_LOCAL_POWER_FLOW].bChange = FALSE;
	}




	if (PRM_PCS[BATT_LOCAL_CONTROL_OPTION].bChange)
	{
		PRM_PCS[BATT_LOCAL_CONTROL_OPTION].bChange = FALSE;
	}

	if (PRM_PCS[BATT_LOCAL_IVC_V_REF_CHG].bChange)
	{
		PRM_PCS[BATT_LOCAL_IVC_V_REF_CHG].bChange = FALSE;
	}

	if (PRM_PCS[BATT_LOCAL_IVC_V_REF_DCHG].bChange)
	{
		PRM_PCS[BATT_LOCAL_IVC_V_REF_DCHG].bChange = FALSE;
	}

	if (PRM_PCS[BATT_LOCAL_ICC_I_REF_CHG].bChange)
	{
		PRM_PCS[BATT_LOCAL_ICC_I_REF_CHG].bChange = FALSE;
	}

	if (PRM_PCS[BATT_LOCAL_ICC_I_REF_DCHG].bChange)
	{
		PRM_PCS[BATT_LOCAL_ICC_I_REF_DCHG].bChange = FALSE;
	}

	if (PRM_PCS[BATT_LOCAL_PCC_P_REF_CHG].bChange)
	{
		PRM_PCS[BATT_LOCAL_PCC_P_REF_CHG].bChange = FALSE;
	}

	if (PRM_PCS[BATT_LOCAL_PCC_P_REF_DCHG].bChange)
	{
		PRM_PCS[BATT_LOCAL_PCC_P_REF_DCHG].bChange = FALSE;
	}

	if (PRM_PCS[BATT_REMOTE_CONTROL_MODE].bChange)
	{
		CC_BCStateUpdate();
		PRM_PCS[BATT_REMOTE_CONTROL_MODE].bChange = FALSE;
	}

	if (PRM_PCS[BATT_REMOTE_CONTROL_OPTION].bChange)
	{
		PRM_PCS[BATT_REMOTE_CONTROL_OPTION].bChange = FALSE;
	}

	if (PRM_PCS[BATT_REMOTE_PCC_P_REF].bChange)
	{
		PRM_PCS[BATT_REMOTE_PCC_P_REF].bChange = FALSE;
	}

	if (PRM_PCS[BATT_REMOTE_PCC_Q_REF].bChange)
	{
		PRM_PCS[BATT_REMOTE_PCC_Q_REF].bChange = FALSE;
	}

	if (PRM_PCS[BATT_IVC_PCC_CURRENT_LMT_CHG].bChange)
	{
		PRM_PCS[BATT_IVC_PCC_CURRENT_LMT_CHG].bChange = FALSE;
	}

	if (PRM_PCS[BATT_IVC_PCC_CURRENT_LMT_DCHG].bChange)
	{
		PRM_PCS[BATT_IVC_PCC_CURRENT_LMT_DCHG].bChange = FALSE;
	}


	if (PRM_PCS[BATT_PCC_PWM_DISABLE_SEC].bChange)
	{
		PRM_PCS[BATT_PCC_PWM_DISABLE_SEC].bChange = FALSE;
	}

	if (PRM_PCS[BATT_PEBB_FAULT_ID].bChange)
	{
		PRM_PCS[BATT_PEBB_FAULT_ID].bChange = FALSE;
	}

	if (PRM_PCS[BATT_REBB_FAULT_RST].bChange)
	{
		PRM_PCS[BATT_REBB_FAULT_RST].bChange = FALSE;
	}

	if (PRM_PCS[BATT_PEBB_0_TEMP_S].bChange)
	{
		PRM_PCS[BATT_PEBB_0_TEMP_S].bChange = FALSE;
	}

	if (PRM_PCS[BATT_PEBB_1_TEMP_S].bChange)
	{
		PRM_PCS[BATT_PEBB_1_TEMP_S].bChange = FALSE;
	}

	if (PRM_PCS[BATT_PEBB_0_HEATSYNC].bChange)
	{
		PRM_PCS[BATT_PEBB_0_HEATSYNC].bChange = FALSE;
	}

	if (PRM_PCS[BATT_PEBB_1_HEATSYNC].bChange)
	{
		PRM_PCS[BATT_PEBB_1_HEATSYNC].bChange = FALSE;
	}

	if (PRM_PCS[VCON_P_GAIN].bChange || PRM_PCS[ICON_I_GAIN].bChange ||
		PRM_PCS[BATT_17].bChange || PRM_PCS[CC_CUT_OFF_FREQ].bChange )
	{
		CTRL_INV_PRC_UpdateParameter();
		PRM_PCS[VCON_P_GAIN].bChange = FALSE;
		PRM_PCS[ICON_I_GAIN].bChange = FALSE;
		PRM_PCS[BATT_17].bChange = FALSE;
		PRM_PCS[CC_CUT_OFF_FREQ].bChange = FALSE;
	}


	if (PRM_PCS[BATT_CUTOFF_VOLTAGE_DCHG].bChange ||
			PRM_PCS[BATT_VDCREF_CHG].bChange ||
			PRM_PCS[BATT_VDCREF_DCHG].bChange ||
			PRM_PCS[BATT_CYCLING_CNT].bChange ||
			PRM_PCS[BATT_INIT_STATE].bChange ||
			PRM_PCS[BATT_CHG_TIME].bChange ||
			PRM_PCS[BATT_DCHG_TIME].bChange ||
			PRM_PCS[BATT_START_IDLE_TIME].bChange ||
			PRM_PCS[BATT_END_COND_HOLD_TIME].bChange ||
			PRM_PCS[BATT_CHG_DCHG_MODE].bChange ||
			PRM_PCS[BATT_CONST_PWR_CHG].bChange ||
			PRM_PCS[CONTROLLER_SEL].bChange ||
			PRM_PCS[GC_IS_MODE].bChange ||
			PRM_PCS[DROOP_ENB].bChange ||
			PRM_PCS[PQC_Q_I_GAIN].bChange
	)
		{
			CTRL_INV_PRC_UpdateParameter();
			CTRL_INV_SYNC_UpdateParameter();
			CTRL_INV_PQC_UpdateParameter();
			CTRL_INV_DROOP_UpdateParameter();

			PRM_PCS[BATT_CUTOFF_VOLTAGE_DCHG].bChange = FALSE;
			PRM_PCS[BATT_VDCREF_CHG].bChange = FALSE;
			PRM_PCS[BATT_VDCREF_DCHG].bChange = FALSE;
			PRM_PCS[BATT_CYCLING_CNT].bChange = FALSE;
			PRM_PCS[BATT_INIT_STATE].bChange = FALSE;
			PRM_PCS[BATT_CHG_TIME].bChange = FALSE;
			PRM_PCS[BATT_DCHG_TIME].bChange = FALSE;
			PRM_PCS[BATT_START_IDLE_TIME].bChange = FALSE;
			PRM_PCS[BATT_END_COND_HOLD_TIME].bChange = FALSE;
			PRM_PCS[BATT_CHG_DCHG_MODE].bChange = FALSE;
			PRM_PCS[BATT_CONST_PWR_CHG].bChange = FALSE;
			PRM_PCS[CONTROLLER_SEL].bChange = FALSE;
			PRM_PCS[GC_IS_MODE].bChange = FALSE;
			PRM_PCS[DROOP_ENB].bChange = FALSE;
			PRM_PCS[PQC_Q_I_GAIN].bChange = FALSE;
		}

	if (PRM_PCS[BATT_CONST_PWR_DCHG].bChange || PRM_PCS[BATT_DCC_P_GAIN].bChange)
	{

		CTRL_INV_PQC_UpdateParameterRamp();

		PRM_PCS[BATT_CONST_PWR_DCHG].bChange = FALSE;
		PRM_PCS[BATT_DCC_P_GAIN].bChange = FALSE;
	}

	if (PRM_PCS[V_DROOP_RAMP].bChange)
	{
		CTRL_INV_DROOP_UpdateRamp();
		PRM_PCS[V_DROOP_RAMP].bChange = FALSE;
	}

	if (PRM_PCS[VI_ENB].bChange || PRM_PCS[VI_R_P].bChange
			|| PRM_PCS[VI_R_PI].bChange || PRM_PCS[VI_L_P].bChange
			|| PRM_PCS[VI_L_PI].bChange || PRM_PCS[VI_T_MSEC].bChange)
	{

		CTRL_INV_VI_UpdateParameter();

		PRM_PCS[VI_ENB].bChange = FALSE;
		PRM_PCS[VI_R_P].bChange = FALSE;
		PRM_PCS[VI_R_PI].bChange = FALSE;
		PRM_PCS[VI_L_P].bChange = FALSE;
		PRM_PCS[VI_L_PI].bChange = FALSE;
		PRM_PCS[VI_T_MSEC].bChange = FALSE;
	}

	if (PRM_PCS[VUC_ENB].bChange || PRM_PCS[VUC_PI_KP].bChange
			|| PRM_PCS[VUC_PI_KI].bChange || PRM_PCS[VUC_PEAK_LPF].bChange)
	{
		CTRL_INV_VUC_UpdateParameter();

		PRM_PCS[VUC_ENB].bChange = FALSE;
		PRM_PCS[VUC_PI_KP].bChange = FALSE;
		PRM_PCS[VUC_PI_KI].bChange = FALSE;
		PRM_PCS[VUC_PEAK_LPF].bChange = FALSE;
	}

	if (PRM_PCS[VC_PR_CUTOFF_IS].bChange )
	{
		CTRL_INV_PRC_UpdateParameter();

		PRM_PCS[VC_PR_CUTOFF_IS].bChange = FALSE;
	}

	if (PRM_PCS[BYP_MODE].bChange )
	{
		CTRL_BYP_SYNC_UpdateParameter();
		SYS_ESLV_UpdateParameter();

		PRM_PCS[BYP_MODE].bChange = FALSE;
	}

	if (PRM_PCS[SEAMLESS_ENB].bChange )
	{
		CTRL_INV_SEAMLESS_UpdateParameter();
		PRM_PCS[SEAMLESS_ENB].bChange = FALSE;
	}

	if (PRM_PCS[IS_CCP].bChange )
	{
		CTRL_FILTER_Create();
		CTRL_FILTER_UpdateAPS();
		CC_CalcVoltageFilterShadow();
		CTRL_BYP_SYNC_Create();
		CTRL_INV_DROOP_Create();
		CTRL_INV_PRC_UpdateParameter();
		CTRL_INV_SYNC_Create();
		CTRL_INV_VUC_Create();

		PRM_PCS[IS_CCP].bChange = FALSE;
	}

	if ( PRM_PCS[IS_OV_LEVEL1].bChange  )
	{
		PRM_PCS[IS_OV_LEVEL1].bChange = FALSE;
		SYS_UpdateLevel2ProtectionLevel(SYS_INV_OV_LEVEL1, FALSE);
	}

	if ( PRM_PCS[IS_UV_LEVEL1].bChange  )
	{
		PRM_PCS[IS_UV_LEVEL1].bChange = FALSE;
		SYS_UpdateLevel2ProtectionLevel(SYS_INV_UV_LEVEL1, FALSE);
	}

	if ( PRM_PCS[IS_OF_LEVEL1].bChange  )
	{
		PRM_PCS[IS_OF_LEVEL1].bChange = FALSE;
		SYS_UpdateLevel2ProtectionLevel(SYS_INV_OF_LEVEL1, FALSE);
	}

	if ( PRM_PCS[IS_UF_LEVEL1].bChange  )
	{
		PRM_PCS[IS_UF_LEVEL1].bChange = FALSE;
		SYS_UpdateLevel2ProtectionLevel(SYS_INV_UF_LEVEL1, FALSE);
	}

	if ( PRM_PCS[GRID_OV_LEVEL_INSTANT].bChange || PRM_PCS[GRID_UV_LEVEL_INSTANT].bChange )
	{
		CTRL_INV_SEAMLESS_UpdateParameter();

		PRM_PCS[GRID_OV_LEVEL_INSTANT].bChange = FALSE;
		PRM_PCS[GRID_UV_LEVEL_INSTANT].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_OF_LEVEL2_SEAMLESS].bChange || PRM_PCS[GRID_UF_LEVEL2_SEAMLESS].bChange )
	{
		SYS_SetLevel2SeamlessProtection();

		PRM_PCS[GRID_OF_LEVEL2_SEAMLESS].bChange = FALSE;
		PRM_PCS[GRID_UF_LEVEL2_SEAMLESS].bChange = FALSE;
	}

	if ( PRM_PCS[GRID_OV_LEVEL2_INSTANT].bChange || PRM_PCS[GRID_UV_LEVEL2_INSTANT].bChange )
	{

		if(PRM_PCS[GRID_UV_LEVEL2_INSTANT].iValue > 95)
			PRM_PCS[GRID_UV_LEVEL2_INSTANT].iValue = 95;

		CTRL_INV_SEAMLESS_UpdateParameter();

		PRM_PCS[GRID_OV_LEVEL2_INSTANT].bChange = FALSE;
		PRM_PCS[GRID_UV_LEVEL2_INSTANT].bChange = FALSE;
	}

	if ( PRM_PCS[DROOP_FREQ].bChange || PRM_PCS[DROOP_VOLT].bChange || PRM_PCS[DROOP_TR_Z].bChange )
	{
		CTRL_INV_DROOP_UpdateParameter();

		PRM_PCS[DROOP_FREQ].bChange = FALSE;
		PRM_PCS[DROOP_VOLT].bChange = FALSE;
		PRM_PCS[DROOP_TR_Z].bChange = FALSE;
	}

	if ( PRM_PCS[SYNC_W_PI_KP].bChange || PRM_PCS[SYNC_W_PI_KI].bChange || PRM_PCS[SYNC_W_PI_MAX].bChange
			|| PRM_PCS[SYNC_W_IIR_CUTOFF].bChange || PRM_PCS[SYNC_V_IIR_CUTOFF].bChange )
	{
		CTRL_INV_SYNC_UpdateParameter();
		CTRL_BYP_SYNC_UpdateParameter();
#if PARA_VERSION >= 0x1003
		CTRL_GEN_UpdateParameter();
#endif

		PRM_PCS[SYNC_W_PI_KP].bChange = FALSE;
		PRM_PCS[SYNC_W_PI_KI].bChange = FALSE;
		PRM_PCS[SYNC_W_PI_MAX].bChange = FALSE;
		PRM_PCS[SYNC_W_IIR_CUTOFF].bChange = FALSE;
		PRM_PCS[SYNC_V_IIR_CUTOFF].bChange = FALSE;
	}

	// 151104
	if ( PRM_PCS[SYNC_W_PI_KP_DIVISION].bChange )
	{
		CTRL_INV_SYNC_UpdateParameter();
		CTRL_BYP_SYNC_UpdateParameter();
#if PARA_VERSION >= 0x1003
		CTRL_GEN_UpdateParameter();
#endif

		PRM_PCS[SYNC_W_PI_KP_DIVISION].bChange = FALSE;
	}

	// 1603
	if ( PRM_PCS[GI_V_REF].bChange )
	{
		// Write Limit
		if( PRM_PCS[GI_V_REF].iValue < ACP.INV.RATE.Vll * 0.89 )
			PRM_PCS[GI_V_REF].iValue = ACP.INV.RATE.Vll * 0.89;

		if( PRM_PCS[GI_V_REF].iValue > ACP.INV.RATE.Vll * 1.11 )
			PRM_PCS[GI_V_REF].iValue = ACP.INV.RATE.Vll * 1.11;

		CTRL_INV_DROOP_GI_V_Update();
		PRM_PCS[GI_V_REF].bChange = FALSE;
	}

	if ( PRM_PCS[BESS71].bChange )
	{
		PRM_PCS[BESS71].bChange = FALSE;
	}
	if ( PRM_PCS[BESS72].bChange )
	{
		PRM_PCS[BESS72].bChange = FALSE;
	}
	if ( PRM_PCS[BESS73].bChange )
	{
		CTRL_FILTER_Create();///XXX temp
		PRM_PCS[BESS73].bChange = FALSE;
	}

#if PARA_VERSION >= 0x1003
	if ( PRM_PCS[GN_BYP_V_L1].bChange )
	{
		EADC.pAdcBypEa->fGain=PRM_PCS[GN_BYP_V_L1].iValue*PRM_PCS[GN_BYP_V_L1].fIncDec;
		PRM_PCS[GN_BYP_V_L1].bChange = FALSE;
	}
	if ( PRM_PCS[GN_BYP_V_L2].bChange )
	{
		EADC.pAdcBypEb->fGain=PRM_PCS[GN_BYP_V_L2].iValue*PRM_PCS[GN_BYP_V_L2].fIncDec;
		PRM_PCS[GN_BYP_V_L2].bChange = FALSE;
	}
	if ( PRM_PCS[GN_BYP_V_L3].bChange )
	{
		EADC.pAdcBypEc->fGain=PRM_PCS[GN_BYP_V_L3].iValue*PRM_PCS[GN_BYP_V_L3].fIncDec;
		PRM_PCS[GN_BYP_V_L3].bChange = FALSE;
	}
	if ( PRM_PCS[GN_GEN_V_L1].bChange )
	{
		EADC.pAdcGenEa->fGain=PRM_PCS[GN_GEN_V_L1].iValue*PRM_PCS[GN_GEN_V_L1].fIncDec;
		PRM_PCS[GN_GEN_V_L1].bChange = FALSE;
	}
	if ( PRM_PCS[GN_GEN_V_L2].bChange )
	{
		EADC.pAdcGenEb->fGain=PRM_PCS[GN_GEN_V_L2].iValue*PRM_PCS[GN_GEN_V_L2].fIncDec;
		PRM_PCS[GN_GEN_V_L2].bChange = FALSE;
	}
	if ( PRM_PCS[GN_GEN_V_L3].bChange )
	{
		EADC.pAdcGenEc->fGain=PRM_PCS[GN_GEN_V_L3].iValue*PRM_PCS[GN_GEN_V_L3].fIncDec;
		PRM_PCS[GN_GEN_V_L3].bChange = FALSE;
	}
	if ( PRM_PCS[GN_REV_L1].bChange )
	{
		PRM_PCS[GN_REV_L1].bChange = FALSE;
	}
	if ( PRM_PCS[GN_REV_L2].bChange )
	{
		PRM_PCS[GN_REV_L2].bChange = FALSE;
	}
	if ( PRM_PCS[GN_REV_L3].bChange )
	{
		PRM_PCS[GN_REV_L3].bChange = FALSE;
	}
	if ( PRM_PCS[GN_LOAD_I_L1].bChange )
	{
		EADC.pAdcCapIa->fGain = PRM_PCS[GN_LOAD_I_L1].iValue * PRM_PCS[GN_LOAD_I_L1].fIncDec;
		PRM_PCS[GN_LOAD_I_L1].bChange = FALSE;
	}
	if ( PRM_PCS[GN_LOAD_I_L2].bChange )
	{
		EADC.pAdcCapIb->fGain = PRM_PCS[GN_LOAD_I_L2].iValue * PRM_PCS[GN_LOAD_I_L2].fIncDec;
		PRM_PCS[GN_LOAD_I_L2].bChange = FALSE;
	}
	if ( PRM_PCS[GN_LOAD_I_L3].bChange )
	{
		EADC.pAdcCapIc->fGain = PRM_PCS[GN_LOAD_I_L3].iValue * PRM_PCS[GN_LOAD_I_L3].fIncDec;
		PRM_PCS[GN_LOAD_I_L3].bChange = FALSE;
	}
	if ( PRM_PCS[BATT_HOLD_SEC].bChange  )
	{
		CTRL_GEN_UpdateParameter();
		PRM_PCS[BATT_HOLD_SEC].bChange = FALSE;
	}
	if ( PRM_PCS[BATT_P_DECREASE_SEC].bChange  )
	{
		CTRL_GEN_UpdateParameter();
		PRM_PCS[BATT_P_DECREASE_SEC].bChange = FALSE;
	}
	if ( PRM_PCS[BATT_GENERATOR_POWER_P].bChange  )
	{
		CTRL_GEN_UpdateParameter();
		PRM_PCS[BATT_GENERATOR_POWER_P].bChange = FALSE;
	}
	if ( PRM_PCS[BATT_GENERATOR_POWER_Q].bChange  )
	{
		CTRL_GEN_UpdateParameter();
		PRM_PCS[BATT_GENERATOR_POWER_Q].bChange = FALSE;
	}
	if ( PRM_PCS[BATT_POWER_METER_P].bChange  )
	{
		CTRL_GEN_UpdateParameter();
		PRM_PCS[BATT_POWER_METER_P].bChange = FALSE;
	}
	if ( PRM_PCS[BATT_POWER_METER_Q].bChange  )
	{
		CTRL_GEN_UpdateParameter();
		PRM_PCS[BATT_POWER_METER_Q].bChange = FALSE;
	}

	if ( PRM_PCS[BATT_REV_06].bChange  )
	{
		PRM_PCS[BATT_REV_06].bChange = FALSE;
	}
	if ( PRM_PCS[BATT_REV_07].bChange  )
	{
		if( PRM_PCS[CTRL_TEST_MODE].iValue != 0 )
		{

		}
		PRM_PCS[BATT_REV_07].bChange = FALSE;
	}
	if ( PRM_PCS[BATT_REV_08].bChange  )
	{
		PRM_PCS[BATT_REV_08].bChange = FALSE;
	}
	if ( PRM_PCS[BATT_REV_09].bChange  )
	{
		PRM_PCS[BATT_REV_09].bChange = FALSE;
	}
	if ( PRM_PCS[SYNC_PI_MAG_LIMIT].bChange  )
	{
		CTRL_INV_SYNC_UpdateParameter();
		PRM_PCS[SYNC_PI_MAG_LIMIT].bChange = FALSE;
	}

	if ( PRM_PCS[SYNC_PI_MAG_FILTER].bChange  )
	{
		//CTRL_INV_SYNC_UpdateParameter();
		PRM_PCS[SYNC_PI_MAG_LIMIT].bChange = FALSE;
	}

	if ( PRM_PCS[PM_OVERLOAD_LV1_KVA].bChange  )
	{
		PRM_PCS[PM_OVERLOAD_LV1_KVA].bChange = FALSE;
	}

	if ( PRM_PCS[PM_OVERLOAD_LV2_KVA].bChange  )
	{
		PRM_PCS[PM_OVERLOAD_LV2_KVA].bChange = FALSE;
	}

	if ( PRM_PCS[PM_OVERLOAD_TIME].bChange  )
	{
		ODT_HoldTimeChange(INVERTER.odtPMOverLoad, PARAM_RAW_VAL(PM_OVERLOAD_TIME));
		PRM_PCS[PM_OVERLOAD_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[PM_OVERLOAD_CLEAR_TIME].bChange  )
	{
		ODT_HoldTimeChange(INVERTER.odtPMOverLoadClear, PARAM_RAW_VAL(PM_OVERLOAD_CLEAR_TIME));
		PRM_PCS[PM_OVERLOAD_CLEAR_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[SCR_RATED_CURRENT].bChange  )
	{
		PRM_PCS[SCR_RATED_CURRENT].bChange = FALSE;
	}

	if ( PRM_PCS[SCR_CURRENT_LIMIT].bChange  )
	{
		PRM_PCS[SCR_CURRENT_LIMIT].bChange = FALSE;
	}

	if ( PRM_PCS[LOAD_OC_LEVEL].bChange  )
	{
		PRM_PCS[LOAD_OC_LEVEL].bChange = FALSE;
	}

#if 0
	if ( PRM_PCS[GEN_OV_LEVEL].bChange || PRM_PCS[GEN_UV_LEVEL].bChange || PRM_PCS[GEN_OF_LEVEL].bChange || PRM_PCS[GEN_UF_LEVEL].bChange )
	{
		SYS_SetLevelGeneratorProtection();
		PRM_PCS[GEN_OV_LEVEL].bChange = FALSE;
		PRM_PCS[GEN_UV_LEVEL].bChange = FALSE;
		PRM_PCS[GEN_OF_LEVEL].bChange = FALSE;
		PRM_PCS[GEN_UF_LEVEL].bChange = FALSE;
	}
#endif

	/* GI, GC 전류제어기 Gain 분리 */
	/* GC 전류제어기 Gain */
    if (PRM_PCS[GC_CC_PR_P_GAIN].bChange || PRM_PCS[GC_CC_PR_I_GAIN_1ST].bChange
            || PRM_PCS[GC_CC_PR_I_GAIN_5TH].bChange || PRM_PCS[GC_CC_PR_I_GAIN_7TH].bChange
            || PRM_PCS[GC_CC_PR_I_GAIN_11TH].bChange)
    {
    	CTRL_INV_PRC_ApplyGC_GI_PI_Gain();

        PRM_PCS[GC_CC_PR_P_GAIN].bChange = FALSE;
        PRM_PCS[GC_CC_PR_I_GAIN_1ST].bChange = FALSE;
        PRM_PCS[GC_CC_PR_I_GAIN_5TH].bChange = FALSE;
        PRM_PCS[GC_CC_PR_I_GAIN_7TH].bChange = FALSE;
        PRM_PCS[GC_CC_PR_I_GAIN_11TH].bChange = FALSE;
    }

    /* GI 전류제어기 Gain */
    if (PRM_PCS[GI_CC_PR_P_GAIN].bChange || PRM_PCS[GI_CC_PR_I_GAIN_1ST].bChange
            || PRM_PCS[GI_CC_PR_I_GAIN_5TH].bChange || PRM_PCS[GI_CC_PR_I_GAIN_7TH].bChange
            || PRM_PCS[GI_CC_PR_I_GAIN_11TH].bChange)
    {
    	CTRL_INV_PRC_ApplyGC_GI_PI_Gain();

        PRM_PCS[GI_CC_PR_P_GAIN].bChange = FALSE;
        PRM_PCS[GI_CC_PR_I_GAIN_1ST].bChange = FALSE;
        PRM_PCS[GI_CC_PR_I_GAIN_5TH].bChange = FALSE;
        PRM_PCS[GI_CC_PR_I_GAIN_7TH].bChange = FALSE;
        PRM_PCS[GI_CC_PR_I_GAIN_11TH].bChange = FALSE;
    }

    /* GI 전압제어기 Gain */
    if (PRM_PCS[GI_VC_PR_P_GAIN].bChange || PRM_PCS[GI_VC_PR_I_GAIN_1ST].bChange
            || PRM_PCS[GI_VC_PR_I_GAIN_5TH].bChange || PRM_PCS[GI_VC_PR_I_GAIN_7TH].bChange
            || PRM_PCS[GI_VC_PR_I_GAIN_11TH].bChange)

    {
    	CTRL_INV_PRC_ApplyGC_GI_PI_Gain();

        PRM_PCS[GI_VC_PR_P_GAIN].bChange = FALSE;
        PRM_PCS[GI_VC_PR_I_GAIN_1ST].bChange = FALSE;
        PRM_PCS[GI_VC_PR_I_GAIN_5TH].bChange = FALSE;
        PRM_PCS[GI_VC_PR_I_GAIN_7TH].bChange = FALSE;
        PRM_PCS[GI_VC_PR_I_GAIN_11TH].bChange = FALSE;
    }

    /* EVE DC Charger 관련 파라미터 */
    if(PRM_PCS[EVE_MAXIMUM_CHARGE_VOLTAGE].bChange || PRM_PCS[EVE_START_CHARGE_VOLTAGE].bChange)
    {
    	EVE_UpdateParameter();

        PRM_PCS[EVE_MAXIMUM_CHARGE_VOLTAGE].bChange = FALSE;
        PRM_PCS[EVE_START_CHARGE_VOLTAGE].bChange = FALSE;
    }

	if ( PRM_PCS[HOLD_LOAD_POWER_TIME].bChange  )
	{
		ODT_HoldTimeChange(INVERTER.odtHoldLoadP, PARAM_RAW_VAL(HOLD_LOAD_POWER_TIME));
		PRM_PCS[HOLD_LOAD_POWER_TIME].bChange = FALSE;
	}

	if ( PRM_PCS[SEAMLESS_PR_STOP].bChange  )
	{
		PRM_PCS[SEAMLESS_PR_STOP].bChange = FALSE;
	}

	if ( PRM_PCS[SEAMLESS_FAST_SCR_OFF].bChange  )
	{
		PRM_PCS[SEAMLESS_FAST_SCR_OFF].bChange = FALSE;
	}

	if ( PRM_PCS[TRAN_CNT].bChange  )
	{
		PRM_PCS[TRAN_CNT].bChange = FALSE;
	}
//by JCNET
    if (PRM_PCS[CTRL_SCC_D_P_GAIN].bChange ||  PRM_PCS[CTRL_SCC_D_I_GAIN].bChange ||
        PRM_PCS[CTRL_SCC_Q_P_GAIN].bChange ||  PRM_PCS[CTRL_SCC_Q_I_GAIN].bChange ||
        PRM_PCS[CTRL_SCC_N_P_GAIN].bChange ||  PRM_PCS[CTRL_SCC_N_I_GAIN].bChange)
    {   extern void SCC_UpdateGains(void);
        PRM_PCS[CTRL_SCC_D_P_GAIN].bChange = FALSE;
        PRM_PCS[CTRL_SCC_D_I_GAIN].bChange = FALSE;
        PRM_PCS[CTRL_SCC_Q_P_GAIN].bChange = FALSE;
        PRM_PCS[CTRL_SCC_Q_I_GAIN].bChange = FALSE;
        PRM_PCS[CTRL_SCC_N_P_GAIN].bChange = FALSE;
        PRM_PCS[CTRL_SCC_N_I_GAIN].bChange = FALSE;
        SCC_UpdateGains();
    }
#endif
#endif
}


