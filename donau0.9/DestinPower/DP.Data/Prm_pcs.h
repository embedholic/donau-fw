/*
 * Prm_pcs.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef PRM_PCS_H_
#define PRM_PCS_H_

#include "LGS_Common.h"
#include "Parameter.h"

#define PARAM_VAL(PARAM)	\
(PRM_PCS[PARAM].iValue * PRM_PCS[PARAM].fIncDec)

#define PARAM_VAL_PRM(PRM)	\
((PRM).iValue * (PRM).fIncDec)

#define PARAM_RAW_VAL(PARAM)	\
(PRM_PCS[PARAM].iValue)

#define	PARAM_COL_VAL(PARAM)	\
(PRM_PCS[PARAM].fIncDec)

#define	PARAM_GROUP_MAX	11

enum { PARAM_CSI_DQ = 0, PARAM_VSI_PR, PARAM_CSI_PR };
typedef enum { PARAM_OPERATION_MODE_GC = 0, PARAM_OPERATION_MODE_IS, PARAM_OPERATION_MODE_AUTO_IS } CMODE;

typedef enum
{
	// �ý���
	SYS_MO_ENABLE,					// 1 ��� ���� Enable / Disable
	SYS_PEBB_MANUAL_OP,				// 2
	SYS_PEBB_ONOFF_WAIT_TIME,		// 3 - Pebb On/Off Wait Time

	// PEBB ���� �Ķ���ʹ� ������� ���� �ؾ� ��.
	SYS_SLAVE_PEBB2_ON_LOAD_OFFSET,		// 6 - Pebb ON ���� ���� �ɼ�.
	SYS_SLAVE_PEBB2_OFF_LOAD_OFFSET,	// 7 - Pebb OFF ���� ���� �ɼ�.
	SYS_SLAVE_PEBB3_ON_LOAD_OFFSET,		// 8 - Pebb ON ���� ���� �ɼ�.
	SYS_SLAVE_PEBB3_OFF_LOAD_OFFSET,	// 9 - Pebb OFF ���� ���� �ɼ�.
	SYS_SLAVE_PEBB4_ON_LOAD_OFFSET,		// 10 - Pebb ON ���� ���� �ɼ�.
	SYS_SLAVE_PEBB4_OFF_LOAD_OFFSET,	// 11 - Pebb OFF ���� ���� �ɼ�.
	SYS_SLAVE_PEBB5_ON_LOAD_OFFSET,		// 12 - Pebb ON ���� ���� �ɼ�.
	SYS_SLAVE_PEBB5_OFF_LOAD_OFFSET,	// 13 - Pebb OFF ���� ���� �ɼ�.
	SYS_SLAVE_PEBB6_ON_LOAD_OFFSET,		// 14 - Pebb ON ���� ���� �ɼ�.
	SYS_SLAVE_PEBB6_OFF_LOAD_OFFSET,	// 15 - Pebb OFF ���� ���� �ɼ�.
	SYS_SLAVE_PEBB7_ON_LOAD_OFFSET,		// 16 - Pebb ON ���� ���� �ɼ�.
	SYS_SLAVE_PEBB7_OFF_LOAD_OFFSET,	// 17 - Pebb OFF ���� ���� �ɼ�.
	SYS_SLAVE_PEBB8_ON_LOAD_OFFSET,		// 18 - Pebb ON ���� ���� �ɼ�.
	SYS_SLAVE_PEBB8_OFF_LOAD_OFFSET,	// 19 - Pebb OFF ���� ���� �ɼ�.

	SYS_DPWM_ONLOAD,	// 20 - 120 DPWM ON LEVEL
	SYS_DPWM_OFFLOAD,	// 21 - 120 DPWM OFF LEVEL
	SYS_DPWM_ONTEMP,	// 22 - DPWM ON TEMP
	SYS_DPWM_OFFTEMP,	// 23 - DPWM OFF TEMP || MC virtual mode 0: virtual
	SYS_REV_4,	// 24 Flexible Li
	SYS_REV_5,	// 25 Static Switch Control
	SYS_OPTION,	// 26 LSB 0: GRD ��� ����

	INV_TRANSFORMER,		// 1
	INV_CAPACITY,			// 2
	INV_OC_LEVEL,			// 3
	INV_CURRENT_LMT,		// 4
	INV_CAB_T_MAX,			// 5 PEBB TEMP
	INV_CAB_T_MIN,			// 6

	GRID_RATED_VOLT,		// 1
	GRID_RATED_FREQ,		// 2
	GRID_OV_LEVEL1,			// 3
	GRID_UV_LEVEL1,			// 4
	GRID_OF_LEVEL1,			// 5
	GRID_UF_LEVEL1,			// 6
	GRID_FDPR_ENB,			// 7
	GRID_PWR_GRADIENT_ENB,	// 8
	GRID_PWR_GRADIENT_RAMP,	// 9
	GRID_TIME_SHIFT,		// 10
	GRID_GATE_WAY_ENB,		// 11 - MC4 LESS
	GRID_LEVEL2_PROTECTION_ENABLE, // 12
	GRID_OV_LEVEL2,			// 13
	GRID_UV_LEVEL2,			// 14
	GRID_UF_LEVEL2,			// 15
	GRID_OV_LEVEL1_TRIP_TIME, //16
	GRID_OV_LEVEL2_TRIP_TIME, //17
	GRID_UV_LEVEL1_TRIP_TIME, // 18
	GRID_UV_LEVEL2_TRIP_TIME, // 19
	GRID_UF_LEVEL1_TRIP_TIME, // 20
	GRID_UF_LEVEL2_TRIP_TIME, // 21
	GRID_OF_LEVEL1_TRIP_TIME,  // 22
	GRID_FRT_ENB,		  	// 23
	GRID_FRT_OV_ENB,		// 24
	GRID_FDPR_GRADIENT_LEVEL,	// 25
	GRID_FDPR_DEACTIVATION_FREQ,// 26
	GRID_OF_LEVEL2,				// 27
	GRID_OF_LEVEL2_TRIP_TIME,	// 28
	GRID_RECONNECT_COND_ENB,	// 29

	TIME_YEAR,				// 1
	TIME_MONTH,				// 2
	TIME_DAY,				// 3
	TIME_HOUR,				// 4
	TIME_MINUTE,			// 5
	TIME_SECOND,			// 6

	DGT_DI1,				// 1 - GI �϶� 0:Slave 1:Master
	DGT_DO1,				// 2 - Test ����� �� GBI Fan speed control, FR: PCS ID(�����ϱ�� ��.)
	DGT_PWR_CTRL_MODE,		// 3 - HILL Op of Normal Mode
	DGT_CAN_ID,				// 4
	DGT_REV_00,				// 5 - �����Թ��� �ð�.
	//DGT_MOD_RTU_ID,			// 6 - Modbus RTU ID

	ANL_AI1_OFFSET,			// 1
	ANL_AI1_GAIN,			// 2
	ANL_AI2_OFFSET,			// 3
	ANL_AI2_GAIN,			// 4
	ANL_AI3_OFFSET,			// 5
	ANL_AI3_GAIN,			// 6
	ANL_AI4_OFFSET,			// 7
	ANL_AI4_GAIN,			// 8

	CTRL_VC_P_GAIN,				// 1
	CTRL_VC_I_GAIN,				// 2
	CTRL_VOLT_LPF,				// 3
	CTRL_CC_P_GAIN,				// 4
	CTRL_CC_I_GAIN,				// 5
	CTRL_CC_DI_DT,				// 6
	CTRL_RAMP,					// 7
	CTRL_LI,					// 8 - Read Only - SYS_UpdateKpccKiccLi()
	CTRL_REV_00,				// 9 - ��� ����.
	CTRL_CC_PERIOD,				// 10
	CTRL_PLL_P_GAIN,			// 11
	CTRL_PLL_I_GAIN,			// 12
	CTRL_AUT_FAULT_RST,			// 13
	CTRL_PEBB_HS_OVERTEMP,		// 14 - HeatSink OT(PEBB �µ� ����)
	CTRL_POWER_COMPEN,			// 15
	CTRL_TEST_MODE,				// 16
	CTRL_OPTION,				// 17
	CTRL_VOLT_DEVIATION_TIME,	// 18
	CTRL_REACTIVE_POWER,			// 19 --> -5% �ؾ� PF ��. 0�̸� PF����� 95%���� ��
	CTRL_BYP_V_GRID_TEST_MODE,		// 20 - 141201 Bypass �������� Grid ���� ���� �׽�Ʈ.
	CTRL_INV_IIR1ST_P,				// 21 - iir1st_p
	CTRL_INV_IIR1ST_Q,				// 22 - iir1st_q
	CTRL_REV_REMOTEPLIMIT,			// 23 - ��� ����
	CTRL_TEST_PWM_VQE_REF,			// 24 -
	CTRL_REV_04_1,					// 25 - ��� ����
	CTRL_CURR_UNBAL_LIMIT,			// 26
	CTRL_REV_05,					// 27 - ��� ����
	CTRL_PWR_LIMIT_RAMP,			// 28
	CTRL_SYNC_PI_MAG_P,				// 29 - SYNC MAG P Gain
	CTRL_SYNC_PI_MAG_I,				// 30 - SYNC MAG I Gain
	// AC GENERATE | SYNC
	CTRL_IIVCD_P_GAIN,				//31
	CTRL_IIVCD_I_GAIN,				//32
	CTRL_IIVCQ_P_GAIN,				//33
	CTRL_IIVCQ_I_GAIN,				//34
	///////////////////////////////////////////
	// Anti-Islanding
	CTRL_ENABLE_ANTI_ISLANDING,		//35
	CTRL_APS_LINE_DEADBAND,			//36
	CTRL_REACTIVE_POWER_LIMIT,		//37
	CTRL_ANTI_ISLANDING_K_FACTOR,	//38
	///////////////////////////////////
	CTRL_PR_V_COMPEN,				//39 - P+R ���� ����: 0=>Feedback, 1=>Ref. (�߰� �ɼ�)
	CTRL_SEAMLESS_CNT,				//40 - SEAMLESS ���: UV OV üũ ī��Ʈ: �����ؼ� UV �Ǵ� OV�� �� ī��Ʈ �̻� üũ�Ǹ� SEAMLESS ��ȯ �õ���
	CTRL_SEAMLESS_UV_CNT_STATUS,	//41 - SEAMLESS ���: ���ӵ� UV �߻� ���� Ƚ���� �ִ밪
	CTRL_SEAMLESS_OV_CNT_STATUS,	//42 - SEAMLESS ���: ���ӵ� OV �߻� ���� Ƚ���� �ִ밪
	CTRL_REV_12,					//43 - ��� ����
	///////////////////////////////////
	CTRL_GRID_FS_RMS_CUTOFF_HZ,			// 44
	COSPHI_FACTOR_LIMIT,				// 45 - 140904 �߰�.
	CTRL_REV_14,						// 46 - ��� ����
    CTRL_REV_15,					 	// 47 - ��� ����
    //----------------------- COSPHI ��� ����
	CTRL_COSPHI_CONTROL_MODE,			// 48
	CTRL_COSPHI_POWER_FACTOR_INTERNAL,	// 49
	CTRL_COSPHI_REACTIVE_POWER_INTERNAL,// 50
	CTRL_COSPHI_POWER_FACTOR_ACTUAL,	// 51
	CTRL_COSPHI_POWER_FACTOR_RPC,		// 52
	CTRL_COSPHI_REACTIVE_POWER_RPC,		// 53
	CTRL_COSPHI_STRAY_RATIO,			// 54
	//---------------------------------------
	CTRL_POS_SEQ_PLL_ENB,				// 55
	CTRL_FRT_K_FACTOR,					// 56
	CTRL_FRT_IQ_RAMP_TIME,				// 57
	CTRL_FRT_IQ_RAMP_USE_TIME,			// 58
	CTRL_INV_FS_RMS_CUTOFF_HZ,			// 59
	CTRL_FREQ_CUTOFF_HZ,				// 60

	CTRL_QV_CTRL_TARGET_V,				// 61
	CTRL_QV_CTRL_K_FACTOR,				// 62
	CTRL_QV_CTRL_DEADBAND,				// 63
	CTRL_QV_CTRL_RAMP_TIME,				// 64

	CTRL_FRT_ASYNC_LEVEL,				// 65
	CTRL_COSPHI_P_PN_RAMP_TIME,			// 66
	//----------------------- COSPHI ��� ����
	PARAM_TEMP_MIN,						// 67
	PARAM_TEMP_MAX,						// 68
	PARAM_FAN_START_LOAD,				// 69
	PARAM_P_DE_LV1_TEMP,				// 70
	PARAM_P_DE_LV2_TEMP,				// 71
	PARAM_P_DE_CLEAR_TEMP,				// 72
	PARAM_P_DE_LV1,						// 73
	PARAM_P_DE_LV2,						// 74
	PARAM_FAN_STOP_HOLD_TIME,			// 75
	CTRL_COSPHI_P_5,					// 76
	CTRL_COSPHI_PF_6,					// 77 pd debug
	CTRL_COSPHI_P_6,					// 78 fan debug
	CTRL_COSPHI_PF_7,					// 79
	CTRL_COSPHI_P_7,					// 80
	CTRL_COSPHI_PF_8,					// 81
	CTRL_COSPHI_P_8,					// 82
	CTRL_COSPHI_PF_9,					// 83
	CTRL_COSPHI_P_9,					// 84
	CTRL_COSPHI_PF_10,					// 85
	CTRL_COSPHI_P_10,					// 86
	//---------------------------------------
	CTRL_IINV_VOLTAGE_BUILDUP_TIME,		// 87
	CTRL_IINV_PLL_STABILIZE_TIME,		// 88
	CTRL_IINV_PLL_P_GAIN,				// 89
	CTRL_IINV_PLL_I_GAIN,				// 90
	CTRL_IINV_SYNC_TOLERANCE_THETA,		// 91
	CTRL_FRT_IQ_HOLD_TIME,				// 92
	CTRL_REV_0,							// 93

	TRC_TRACE_MODE,		// 1
	TRC_TRACE_SAMPLING,	// 2
	TRC_TRACE1,			// 3
	TRC_TRACE2,			// 4
	TRC_TRACE3,			// 5
	TRC_TRACE4,			// 6
	TRC_TRACE5,			// 7
	TRC_TRACE6,			// 8
	TRC_TRACE7,			// 9
	TRC_TRACE8,			// 10
	TRC_TRACE9,			// 11
	TRC_TRACE10,			// 12
	TRC_TRACE11,			// 13
	TRC_TRACE12,			// 14
	TRC_TRACE13,			// 15
	TRC_TRACE14,			// 16
	TRC_TRACE15,			// 17
	TRC_TRACE16,			// 18
	TRC_TRACE17,			// 19
	TRC_TRACE18,			// 20
	TRC_TRACE19,			// 21
	TRC_TRACE20,			// 22
	TRC_TRACE21,			// 23
	TRC_TRACE22,			// 24
	TRC_TRACE23,			// 25
	TRC_TRACE24,			// 26
	TRC_TRACE25,			// 27
	TRC_TRACE26,			// 28
	TRC_TRACE27,			// 29
	TRC_TRACE28,			// 30
	TRC_TRACE29,			// 31
	TRC_TRACE30,			// 32
	TRC_TRACE31,			// 33
	TRC_TRACE32,			// 34
	TRC_TRACE33,			// 35
	TRC_TRACE34,			// 36
	TRC_TRACE35,			// 37
	TRC_TRACE36,			// 38
	TRC_TRACE37,			// 39
	TRC_TRACE38,			// 40
	TRC_TRACE39,			// 41
	TRC_TRACE40,			// 42
	TRC_TRACE41,			// 43
	TRC_TRACE42,			// 44
	TRC_TRACE43,			// 45
	TRC_TRACE44,			// 46
	TRC_TRACE45,			// 47
	TRC_TRACE46,			// 48
	TRC_TRACE47,			// 49
	TRC_TRACE48,			// 50

	OFS_DCLINK_VOLTAGE,		// 1
	OFS_BAT_CURRENT,		// 2
	OFS_BATT_VOLTAGE,		// 3
	OFS_INV_I_L1,			// 4
	OFS_INV_I_L3,			// 5
	OFS_INV_I_L2,			// 6 ISO-> I L2
	OFS_STACK_TEMP,			// 7
	OFS_CAB_TEMP,			// 8
	OFS_BATT_CHG_CURRENT,	// 9

	GN_DCLINK_VOLTAGE,		// 1
	GN_BAT_CURRENT,			// 2
	GN_BAT_VOLTAGE,			// 3
	GN_INV_V_L1,			// 4
	GN_INV_V_L2,			// 5
	GN_INV_V_L3,			// 6
	GN_INV_I_L1,			// 7
	GN_INV_I_L3,			// 8
	GN_GRID_V_L1,			// 9
	GN_GRID_V_L2,			// 10
	GN_GRID_V_L3,			// 11
	GN_INV_I_L2,			// 12 ISO->I L2
	GN_GRID_I_L1,			// 13 grid current gain a
	GN_GRID_I_L2,			// 14 grid current gain b
	GN_GRID_I_L3,			// 15 grid current gain c

	BATT_OV_LEVEL,				   	//1
	BATT_UV_LEVEL,				   	//2 ( �ִ� ���͸� ���� �� ��� & �ý��� ON ���� )
	BATT_OC_LEVEL,				   	//3 - ���͸� OVER CURRENT. ��/���� ������ ���ؼ��� �ʿ� ����.
	BATT_DC_SIDE_V_ABNORMAL_LEVEL, 	//4 - DC �� ���� ������ ���� ����.( Dclink�� ���͸��� ���� ���� �� �Ķ���� ���� ũ�� Fault )

	BATT_V_RANGE_MAX,			   	//5
	BATT_V_RANGE_MIN,			   	//6

	BATT_ORDER_SOURCE,			   	//7 PCS ���� ��� �ҽ�. 0:Default 1: Local 2: Remote (0: Local)
	BATT_REF_VAL_MODE,			   	//8 Operating V min/max +-5V ���������� �Ŀ� ����(%)

	BATT_LOCAL_CONTROL_MODE,	   	//9 - 0: Default 1: CP, 2: CC-CV, 3: CC 4: CV (Default: CV)
	BATT_LOCAL_POWER_FLOW,			//10 - LOCAL ��忡���� ��/���� �帧. 0: Standby, 	1:Charge, 2:Discharge , ������ �ʿ� ����. �ý��� �� ���۽� ������ Idle
	BATT_LOCAL_CONTROL_OPTION,		//11 - ���� �ɼ�. Bit 1: Auto-Restart�ÿ� ���� ��/���� State �ڵ� ����
	BATT_LOCAL_IVC_V_REF_CHG,		//12 - CV, CC-CV V REFERENCE
	BATT_LOCAL_IVC_V_REF_DCHG,		//13 - CV ���� REFERENCE
	BATT_LOCAL_ICC_I_REF_CHG,		//14 - CC I REFERENCE +: ���� -: ����
	BATT_LOCAL_ICC_I_REF_DCHG,		//15 - RESERVED. CC�� ������ ����. (B:- CC I REFERENCE)
	BATT_LOCAL_PCC_P_REF_CHG,		//16 - PC P REFERENCE +: ���� -: ����
	BATT_LOCAL_PCC_P_REF_DCHG,		//17 - PC P REFERENCE +: ���� -: ����

	BATT_REMOTE_CONTROL_MODE,		//18 0: Default(CP), 1: CP
	BATT_REMOTE_POWER_FLOW,			//19 Reserved
	BATT_REMOTE_CONTROL_OPTION,		//20 Reserved
	BATT_REMOTE_PCC_P_REF,			//21 0: Idle ���: ���� ����: ����
	BATT_REMOTE_PCC_Q_REF,			//22 P�� 0�� �ƴ� �� ���: ���� ���� : ���� ����

	/* Local, Remote ���� */
	BATT_IVC_VDC_RAMP,				//23 CC-CV DC RAMP
	BATT_IVC_PCC_CURRENT_LMT_CHG,	//24 CC-CV, PCC ���� ���� Limit
	BATT_IVC_PCC_CURRENT_LMT_DCHG,	//25 CC-CV, PCC ���� ���� Limit


	BATT_ICC_P_GAIN,				//26 ICC P Gain
	BATT_ICC_I_GAIN,				//27 ICC I Gain

	BATT_PCC_P_GAIN,		    	//28 PCC P gain 0.05 -> 0.005
	BATT_PCC_I_GAIN,		    	//29 PCC I gain
	BATT_PCC_PQ_CUTOFF_HZ,			//30 �Ŀ� ��Ʈ�� Cut Off Hz
	BATT_PCC_PWM_DISABLE_SEC,		//31 �Ŀ������ Idle time(BATT_PCC_PWM_DISABLE_SEC) sec ���� ref�� 0�� �����Ǹ� CVC_DISABLE

	BATT_PEBB_FAULT_MODE,			//32
	RESYNC_W_PI_MAX,			//33

	BATT_PEBB_FAULT_ID,				//34 GBI, PEBB ���峭 ID
	BATT_REBB_FAULT_RST,			//35 GBI, PEBB ���峭 RST����
	BATT_PEBB_0_TEMP_S,	   			//36 GBI, PEBB 0 tempS
	BATT_PEBB_1_TEMP_S,		   		//37 GBI, PEBB 1 tempS
	BATT_PEBB_0_HEATSYNC,	   		//38 GBI, PEBB 0 HeatSync
	BATT_PEBB_1_HEATSYNC,			//39 GBI, PEBB 1 HeatSync

	GEN_CONNECT_CHK_STX_POWER,		//40 MC6 ignore start power ( MC ON delay: 4ms.., MC Status delay: 20ms )
	GEN_CONNECT_I_DIDT,				//41 MC6 ON Predict Current di/dt level
	BATT_3,							//42
	BATT_4,							//43
	BATT_5,							//44 GBI, PEBB 2 HeatSync
	BATT_6,							//45 GBI, PEBB 3 HeatSync

	ACTIVE_DAMPING,				//46 - active damping - X
	VCON_P_GAIN,				//47 - �������� p-gain - X
	ICON_I_GAIN, 				//48 - �������� p-gain - X
	BATT_CUTOFF_VOLTAGE_DCHG,	//49 - �������� 5�� - X
	BATT_VDCREF_CHG,			//50 - �������� 7�� - X
	BATT_VDCREF_DCHG,			//51 - �������� 11�� - X
	BATT_CYCLING_CNT,			//52 - �������� 5�� - X
	BATT_INIT_STATE,			//53 - �������� 7�� - X
	BATT_CHG_TIME,				//54 - �������� 11�� - X
	BATT_DCHG_TIME,				//55 - �������� 1�� i-gain (750kW�̻� Openloop���� ������ ���Ƽ� ���ݿ� �� ��ħ..)
	BATT_START_IDLE_TIME,		//56 - �������� 1�� i-gain - X
	BATT_17,					//57 - vc PResonant Cutoff
	BATT_DCHG_PWR,				//58 - Power Ref
	BATT_END_COND_HOLD_TIME,	//59 - ���뿬�� P i-gain
	BATT_VDC_RAMP,				//60 - ���뿬�� N-gain
	BATT_VF_CHECK_TIME,			//61 - Q ref
	BATT_CURRENT_REF_OFFSET,	//62 - IDLE ���¿����� Iqe ������, ���뿬�� M-gain
	BATT_CHG_DCHG_MODE,			//63 - ��/���������� ��� 0: constant power, 1: constant current, ���뿬�� Droop N-gain
	BATT_CONST_PWR_CHG,			//64 -> �Ŀ� ��Ʈ�� �Ŀ� ���۷��� +: ���� -: ���� , ���뿬�� Droop M-gain
	BATT_CONST_PWR_DCHG,		//65 -> �Ŀ� ��Ʈ�� Cut Off Hz, V ref ramp - X
	BATT_DCC_P_GAIN,			//66 - Omega ref ramp
	V_DROOP_RAMP,				//67 - V Droop ramp
	CC_CUT_OFF_FREQ,			//68 - CC_CUT_OFF_FREQ
	SYNC_W_PI_KP_DIVISION,		//69 - 151104 SYNC_W_PI_KP_DIVISION
	GI_V_REF,			    	//70 GI ��忡���� AC�� ���� ���� ���۷���
	BESS71,					    //71 NOT USED
	BESS72,					    //72 NOT USED
	BESS73,		    			//73 NOT USED
	CONTROLLER_SEL,				//74 -> CSI_dq(0), VSI_p+r(1), CSI_p+r(2) *AutoGC-GI ��忡�� GI�� ����Ǵ� ���� 2���� 1�� ���� ��(�ҽ�����)
	GC_IS_MODE,				    //75 -> GC(0), IS(1), GC+IS(2)
	DROOP_ENB,				    //76 -> DROOP DISABLE(0), ENABLE(1)
	TEST_MODE2,			        //77 -> ���� ���¿��� ��� Ȱ��ȭ (PCS ���� �ÿ� �̿� ����)
	PQC_Q_I_GAIN,				//78 -> ���뿬�� Q i-gain
	VDS_FF_GAIN,				//79 -> feedfoward gain
	VQS_FF_GAIN,	  		 	//80 -> feedfoward gain
	
	VI_ENB,						//81
	VI_R_P,		  		 		//82
	VI_R_PI,		  		 	//83
	VI_L_P,		  		 		//84
	VI_L_PI,		  		 	//85
	VI_T_MSEC,		  		 	//86
	VI_LEVEL,		  		 	//87

	VUC_ENB,					//88
	VUC_PI_KP,					//89
	VUC_PI_KI,					//90
	VUC_PEAK_LPF,				//91
	VC_PR_CUTOFF_IS,			//92 vc PResonant Cutoff - X

	BYP_MODE,					//93
	/* 0:NONE
	 * 1:BYP_ENB(SSW,MG)						2:ES-LV Ÿ�� GC->OFF->GI->AUTO GC Ȱ��ȭ <���ó ����>
	 * 10:Auto-GC->GI->GC����(Recloser,GT)		3:GI���->GC����(Recloser-TestMode)
	 * 20:Auto-GC-GI(SSW,MC6,MG,STABLEEN)
	 * 30:Auto-GC->GI->GC(GenV Sensing,GT Model)
	 */
	SEAMLESS_ENB,				//94
	IS_CCP,						//95

	IS_OV_LEVEL1,			    //96
	IS_UV_LEVEL1,			    //97
	IS_OF_LEVEL1,			    //98
	IS_UF_LEVEL1,			    //99
	GRID_OV_LEVEL_INSTANT,		//100
	GRID_UV_LEVEL_INSTANT,		//101
	BYP_OV_LEVEL1,			    //102 NOT USED
	BYP_UV_LEVEL1,			    //103 NOT USED
	GRID_OF_LEVEL2_SEAMLESS,	//104 BYP_OF_LEVEL1->GRID_OF_LEVEL2_SEAMLESS
	GRID_UF_LEVEL2_SEAMLESS,	//105 BYP_UF_LEVEL1->GRID_UF_LEVEL2_SEAMLESS
	GRID_OV_LEVEL2_INSTANT,	    //106 BYP_OV_LEVEL_INSTANT->GRID_OV_LEVEL2_INSTANT
	GRID_UV_LEVEL2_INSTANT,		//107 BYP_UV_LEVEL_INSTANT->GRID_UV_LEVEL2_INSTANT

	DROOP_FREQ,		//108
	DROOP_VOLT,		//109
	DROOP_TR_Z,		//110

	SYNC_W_PI_KP, 		//111
	SYNC_W_PI_KI, 		//112
	SYNC_W_PI_MAX, 		//113
	SYNC_W_IIR_CUTOFF, 	//114
	SYNC_V_IIR_CUTOFF, 	//115

	WH_CHG_HI, 		// 116 ( INV P 30 MWh )
	WH_CHG_LO,  	// 117 ( INV P 30 MWh )
	WH_DCHG_HI, 	// 118 ( INV P 30 MWh )
	WH_DCHG_LO,		// 119 ( INV P 30 MWh )

	/* 141211 June for FR */
#if PARA_VERSION >= 0x1002
	MWH_CHG_AC,		// 120
	GWH_CHG_AC,     // 121
	MWH_DCHG_AC,    // 122
	GWH_DCHG_AC,    // 123
	MWH_CHG_DC,     // 124
	GWH_CHG_DC,     // 125
	MWH_DCHG_DC,    // 126
	GWH_DCHG_DC,    // 127
#endif

#if PARA_VERSION >= 0x1003
	GN_BYP_V_L1,			// 128
	GN_BYP_V_L2,			// 129
	GN_BYP_V_L3,			// 130
	GN_GEN_V_L1,			// 131
	GN_GEN_V_L2,			// 132
	GN_GEN_V_L3,			// 133
	GN_REV_L1,				// 134
	GN_REV_L2,				// 135
	GN_REV_L3,				// 136
	GN_LOAD_I_L1,			// 137	��������R
	GN_LOAD_I_L2,			// 138	��������S
	GN_LOAD_I_L3,			// 139	��������T

	BATT_HOLD_SEC,				// 140 -> Holding Time
	BATT_P_DECREASE_SEC,		// 141 -> Power Decrease time
	BATT_GENERATOR_POWER_P,		// 142 -> Generator Power
	BATT_GENERATOR_POWER_Q,		// 143
	BATT_POWER_METER_P,			// 144 -> Power Meter ( + ) -> Load Power P
	BATT_POWER_METER_Q,			// 145
	BATT_REV_06,					// 146 Debug : MC6 Status
	BATT_REV_07,					// 147 Debug : MC6 Control
	BATT_REV_08,					// 148 GEN DEBUG
	BATT_REV_09,					// 149 Power Decrease, Auto Charge Mode Enable,Disable

	AUTO_CHARGE_P_REF,				// 150 - �ڵ� ���� �Ŀ� ���۷���
	SYNC_PI_MAG_LIMIT,				// 151 - PI MAG Limit
	SYNC_PI_MAG_FILTER,				// 152 - ��� X
	PM_OVERLOAD_LV1_KVA,			// 153 -> PM_OVERLOAD_LV1 �����
	PM_OVERLOAD_LV2_KVA,			// 154 -> PM_OVERLOAD_LV2 �߰���
	PM_OVERLOAD_TIME,				// 155 -> PM_OVERLOAD_LV1 ���� �ð�, LV2�� Delay���� �ٷ� ����
	PM_OVERLOAD_CLEAR_TIME,				// 156 -> PM_OVERLOAD_LV1 Clear �ð�
	SCR_RATED_CURRENT,				// 157 -> SCR ���� ����
	SCR_CURRENT_LIMIT,				// 158 -> SCR ���� ���� ���� ������ x%
	LOAD_OC_LEVEL,					// 159

    GC_CC_PR_P_GAIN,                // 160 - GC ���� P Gain,
    GC_CC_PR_I_GAIN_1ST,            // 161 - GC ���� I Gain 1st,
    GC_CC_PR_I_GAIN_5TH,            // 162 - GC ���� I Gain 5th,
    GC_CC_PR_I_GAIN_7TH,            // 163 - GC ���� I Gain 7th,
    GC_CC_PR_I_GAIN_11TH,           // 164 - GC ���� I Gain 11th,

    GI_CC_PR_P_GAIN,        // 165 GI ���� P Gain
    GI_CC_PR_I_GAIN_1ST,    // 166 GI ���� I Gain 1st
    GI_CC_PR_I_GAIN_5TH,    // 167 GI ���� I Gain 5th
    GI_CC_PR_I_GAIN_7TH,    // 168 GI ���� I Gain 7th
    GI_CC_PR_I_GAIN_11TH,   // 169 GI ���� I Gain 11th

    GI_VC_PR_P_GAIN,        // 170 GI ���� P Gain
    GI_VC_PR_I_GAIN_1ST,    // 171 GI ���� I Gain 1st
    GI_VC_PR_I_GAIN_5TH,    // 172 GI ���� I Gain 5th
    GI_VC_PR_I_GAIN_7TH,    // 173 GI ���� I Gain 7th
    GI_VC_PR_I_GAIN_11TH,   // 174 GI ���� I Gain 11th

	SEAMLESS_PWM_OFF_CNT,	// 175 Seamless ���� �� PWM Off Counter
	EVE_DC_CHARGER_ENABLE,	// 176 1: EVE ���, 0: EVE ���X
	EVE_MAXIMUM_CHARGE_VOLTAGE,	// 177 EVE ���� ��ǥ ����
	EVE_START_CHARGE_VOLTAGE,// 178 EVE ���� ���� ����
	HOLD_LOAD_POWER_TIME,	 // 179 GC �� Load Power Hold �ð�
	AC_GEN_DC_VOLTAGE_LEVEL, // 180 AC Generate ������ DC ����
	SEAMLESS_PR_STOP,		// 181
	SEAMLESS_FAST_SCR_OFF,	// 182
	TRAN_CNT,				// 183

    //by JCNET PARAM_NUM
#endif
//by JCNET add
	CTRL_SCC_D_P_GAIN,
	CTRL_SCC_D_I_GAIN,
    CTRL_SCC_Q_P_GAIN,
    CTRL_SCC_Q_I_GAIN,
    CTRL_SCC_N_P_GAIN,
    CTRL_SCC_N_I_GAIN,
    CTRL_SCC_D_LIMIT,
    CTRL_SCC_Q_LIMIT,
    CTRL_SCC_N_LIMIT,
    PARAM_NUM
} PRM_PCS_LIST;


extern int		PARAM_GROUP_INDEX[PARAM_GROUP_MAX];
extern Parameter_Table_Item PRM_PCS_TABLE[PARAM_NUM];

extern Parameter *PRM_PCS;

void PARAM_UpdateAll();

#endif /* PRM_PCS_H_ */
