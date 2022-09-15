/*
 * LGS_Common.h
 *
 *  Created on: 2012. 11. 1.
 *      Author: destinPower
 *
 */

/*
 * ���� ����
 * #include "DSP28x_Project.h"	�� Include �� ��쿡 LGS_Common.h ���� ���� ������ ������ ������ �߻��Ѵ�.
 * SYS/BIOS - LIB Ÿ���� ����� ������ Custom��. ���� ���� �� �� ū �ڵ�� ǲ ����Ʈ �߻�.
 */

#ifndef DP_COMMON_DEF_H_
#define DP_COMMON_DEF_H_

#include "MathConst.h"
#include "Types.h"

#define TI_28346
#ifdef TI_28346
#include <xdc/std.h>
#endif

/* private �Լ� ���� define */
#define private static
#define public 			/* public method */
#define out 			/* parameter direction is out */


/* MCU BOARD VERSION_ȸ�ε� ���� ======================================================== */
/* --------------- LIST -------------------*/
#define MCU_PCB_REV_A_D 0x0  /* Init Version */
#define MCU_PCB_REV_B_G 0x1  /* Rev.F ���� ���� SRAM&DATA FLASH GPIO 27 <-> 40 ���� */
						     /* SRAM�� CS�� MCBSP ����� MFSXB(CS 40->27)�� �����Ͽ� ����ϰ�, DATA FLASH�� GPIO27�� 40���� �����Ͽ� CS���� ���. */
  	  	  	  	             /* ADC MUX �߰� �� */
#define MCU_PCB_REV_I_I 0x2
#define MCU_PCB_REV_J 	0x3  /* SRAM�� MR20H40 MRAM���� ��ü ��. */

/* --------------- SET -------------------*/
#define MCU_PCB_REV_VER MCU_PCB_REV_J
/* ===================================================================================== */

/* for zerocrossing */
//#define ZERO_COROSSING_METHOD 0	// Constant W based estimation
#define ZERO_COROSSING_METHOD 1 // PLL Theta based estimation
//=======================================================================================
/* Source Code Infomation
 *
 * 1: MCU1 - DP_LGS_DSP_SW_SVN 	( GT Only )
 * 2: MCU1 - DP_LGS_DSP_SW_SVN_EX  ( GT + GI (BYPMODE, BlackStart, ���� ���� �и� )
 */
#define SOURCE_CODE_BASE 1

/* Compile Build Date - modbus info*/
#define BUILD_DATE_YYYY 2021
#define BUILD_DATE_MMDD 1221

/* compile for debug =================================================================== */
/* --------------- LIST -------------------*/
#define NORMAL 0
#define DEBUG_DEVELOP 1		// for Driver test.
#define DEBUG_SIMULSATION 2 // for Simulation, HILL Test

/* --------------- SET -------------------*/
#define DEBUG_MODE NORMAL

#if DEBUG_MODE
#warning "DEBUG MODE!"
/* --------------- SET -------------------*/
#define DBUG_MODE 2 // 1:dio�� ������ ����, adc�� ����
					// 2:dio .adc ��� ����
					// 3:dio�� ���� - MC4 ON/OFF Input/Output�� ������ ��� ��.

#if DBUG_MODE == 3
	/* �������� �δ� ���� HILL �׽�Ʈ�� ���� bypass ������ capacity ������ ������� ���� - ������ �ž� ���� �� ��� */
	#define HILL_CAPI_TO_BYPV 0
	/* HILL- CapI�� GenV�� ������� ���� - BYP 30 PCS1,2 */
	#define HILL_CAPI_TO_GENV 0
	/* HILL- CAN��� �Է����� ACB3 Status���� - BYP 30 PCS1 ON*/
	#define HILL_ACB3_STATUS_TO_CAN 0
	/* HILL- CAN��� �Է����� ACB3 Status���� - BYP 30 PCS2 ON*/
	#define HILL_ACB3_STATUS_FROM_CAN 0

	/* HILL- BypV�� GenV�� ������� ���� */
	#define HILL_BYPV_TO_GENV 0
	/* CB2 ��ȣ�� Bypass Signal�� �������� ���� - ES-LV Ÿ�� HILL ���� �� ��� */
	#define HILL_CB2_TO_BYPASS 0
	/* MC6 ���� ���� ��� */
	#define MC6_SIMUL_ON 0
#endif

#define USE_EGBI 0
#define IIVC_ENB 0

#endif
/* ===================================================================================== */

#define STABLEEN_HILL_ENABLE 0
#if STABLEEN_HILL_ENABLE == 1
#warning "CB HIL MODE!!!!!!!!!!!!"
#endif

/* Model =============================================================================== */
/* --------------- LIST -------------------*/
#define STBLN_300K	300
#define STBLN_500K 	500 /* TL Ÿ�� */
#define STBLN_1250K 1200

/* --------------- SET -------------------*/
//by JCNET #define OP_MODE STBLN_300K
#define DONAU   999
#define OP_MODE DONAU
/* ===================================================================================== */


#define MAJOR_MODEL 0x100 /* SAVEEN-MG */

#define STABLEEN
/*
 *  �� �� �ɼ� ����
 */
#define SP_NORMAL 0
#define SP_MODEL SP_NORMAL

#define RE_SYNC_DEBUG		/* 16.06.29 Yang Weak-Grid���� Re-sync�� �Ϸ���� �ʾ������� Run���� �Ѿ�� ���� �ذ�  Start Sync�� ������ ��� ���� */
//#define BYP_EVT_OPERATION	/* 16.06.30 Yang MG �� STABLEEN�� STS & ���� CB ����	*/

//JCNET#warning "400kW PEBB"
#define PEBB_TYPE_400KW 1
#define FAN_AUTO_CTRL 1

//#warning "125kW PEBB"
//#define PEBB_TYPE_400KW 0

#if SP_MODEL == SP_NORMAL
//JCNET#warning "Normal TYPE"
#else
#warning "UNKNOWN TYPE"
#endif


/* ====== ���� ���� ================================================================ */
/* PARAMETER VERSION HISTORY
 * MSB 4bit�� Model�� �Ľ��Ѵ�. ������ ��� �Ķ���� ���̺� �������� ���� ����. NvSRAM Parameter ���� �˻�� ����ϹǷ�, hex ��)
 * 0x1001 : 2013~2014.12.11��
 * 0x1002 : 2014.12.11 FR ���������� ��� �߰��� ���� �Ķ���� ���� �߰�(ESS TAB)
 * 0x1003 : 2015.08.28 MG-LV ���� ���� �Ķ���� �߰�.
 * 					   GAIN �߰� �� ����. (GenV�߰�, BypassV Analog->Gain tab���� ����)
 * 					   AI 1���� ���� �߰��� �Ķ���Ͱ� ��� ��.
*/

#define PARA_VERSION 0x1004 // by JCNET for version matching 0x1003
#define PARA_VERSION_MODBUS 1004 // by JCNET 1003


/*
 * MCU VERSION HISTORY
 * 1.00.01 [16.04.11] :
 * 						- 160411 MG ��(Bypass Mode20) �ҽ� �и� ����.
 * 						- 		 ModbusAddr0(Manufacture)�� MAJOR_MODEL Ÿ�� ����
 * 						- 		 Generator OV, UV, OF, UF ���� �߰�(�Ķ����, ���� ���)
 * 						-		 MG �𵨿��� �ƴ� Bypass ��带 ���� ���� ����( ��������� ��� ). ���Ŀ� ������ �����ؾ� ��.
 * 						- 160523 CB2 TEST MODE
 * 						- 180409 HILS ���� �� DI fault ����
 * 		   [18.06.25] : - ���� ������ ���� �� ���� �߰�
 * 		   				- Fault �߻� �� ADO8 Ȱ��ȭ �߰�
 * 		   				- Sag �߻� �� DDO4 Ȱ��ȭ �߰�
 * 		   [18.07.31] : - Sag ���� VRMS�� �߰�
 * 		   				- ���� ����(DC UV) �� �ڵ� ��⵿
 * 		   [18.08.16] : - CTRL_REV_12 -> CTRL_SEAMLESS_CNT
 * 		   				- Stop -> SCR ON ���ǿ� !Grid UV Warning �߰�
 * 		   [18.08.31] : - Run -> Islanding ������� ����
 * 1.00.02 [18.12.18] :
 * 						- ������� �ڵ� ��⵿
 * 		   [19.01.16] :
 * 		   				- system stop -> scr on USE_MBP_PANNEL == 1 �κ� ����
 * 		   [19.03.05] :
 * 		   				- DC UV �ڵ� clear���� DC ���� ���� ����
 * 		   [19.03.06] :
 * 		   				- SagEventHistory ����
 * 		   				- MAX_V_RMS �ʱ�ȭ
 * 		   				- Sag �߻� �� �������� �߰�
 * 		   [19.03.08] :
 * 		   				- CTRL_INV_SEAMLESS_Reset��(SYS_INV_STATE_TRAN_STOP���� ���) ctrl_seamless_uv_ov_cnt_min�� �ʱ�ȭ ����(�Ķ���� ������ ����Ǿ� �ϴµ�, �ʱⰪ���� �����)
 * 		   				- MCU SW Ver. 1.00.02 -> 1.00.03 Update
 * 		   [19.03.13] :
 * 		   				- OC �߻� �� PWM ���� OFF�ϰ�, SSW ON(Y2 ������ / Y1 ����)
 * 		   				- Sag Time offset �ʱⰪ 256 -> 200 ����
 * 		   [19.04.03] :
 * 1.00.04              - system.c SYS_INV_STATE_TRAN_FAULT�� SYS_SetInverterCommand(OFF); ����
 *         [19.04.04] :
 *                      - �߰��� �߻� �� seamless.c���� SCR ���� ���ϵ��� ����
 *         [19.04.08] :
 *                      - seamless.c�� ���� ������ ���� �Ķ���� �⺻�� rms 110 -> 115 / ��ũ 115 -> 130 / Inv OV 120 -> 140
 * 1.00.05
 * 		   [19.05.13] : - SagEvent Duration �ð� Input ���б������� �����ǵ��� ����(Wesco ��û), -200 Offset ����, Trace Buf 8000->400
 * 		                - Islanding bpy ����ȭ üũ �ð� 200ms->100ms. islanding 100ms + resync 50ms
 * 		   [19.08.08] : - seamless.c pk���� pcc -> byp ����
 * 		                - ADC offset ����
 * 		   [19.08.13] : - System���� fast rms ������ ���� ���к��� �������� ���� ��� Islanding -> resync ���� �ð���ŭ Sag Cnt���� ����
 * 		   [19.12.10] : - DDI2�� DC door warning X -> CB2 ����
 * 		                - CB5,6,7 -> EX MCCB 1,2,3 ��Ī ����
 * 		                - AC SMPS -> SMPS ��Ī ����
 * 		   [19.12.11] : - Trace �Ķ���� �⺻�� ����
 * 		                - EVE_MAXIMUM_CHARGE_VOLTAGE �⺻�� 850 -> 900 ����
 * 		                - GRID_UV_LEVEL2_INSTANT �⺻�� 85 -> 82 ����
 * 		                - LOAD_OC_LEVEL �⺻�� 110 -> 125 ����
 * 		   [19.12.12] : - GI_V_REF �⺻�� 330
 * 		                - Grid OV LV2 �ڵ� clear�Ǵ� ���� ����
 * 		   [19.12.26] : - CTRL_GEN_Proceed �������κп� black out �� SCR off �ϴ� �κ� ����(fault ��Ȳ���� system ���°� fault�� �����ϱ� ������ SCR on, off �ݺ��Ǵ� ���� ����)
 * 		                - Byp ���� pk�� ���� ACP.BYP.v_pk -> EXCTRL.bypEqe�� ����
 * 		   [20.01.02] : - TR 320_480 ��� �߰�
 * 		                - HILS ����� �� EVE_DC_CHARGER_ENABLE �Ķ���� �⺻�� 0
 * 		   [20.01.10] : - ���ݺ��Ͽ��� ���� �� ���� �ȵ� -> �������� �ɸ��� 50kW�� ����(25kW)�� �����ǰ� ����
 * 		                - CC_Mode_CP_P �Ŀ� 10%�� ���̴ºκ� ����
 * 		   [20.01.11] : - HILS ��忡�� SYS_DPWM_OFFTEMP �Ķ���� �⺻�� 1
 * 		                - ���� ���ļ� ���� �� CTRL_INV_SYNC_UpdateParameter, CTRL_INV_PQC_UpdateParameter, CTRL_INV_PRC_UpdateParameter, CTRL_INV_DROOP_UpdateParameter �Լ� �����ϵ��� ����
 * 		   [20.01.13] : - Sag RMS ���� �⺻�� 90% -> 85%
 * 		   [20.01.30] : - 480Vgrid ��� ���̰��� ����
 *		   [20.02.05] : - DS1�� �����ϰ� CB3, CB4�� OFF�϶� ��ȣ�� �ֱ������� ����. STOP���¿��� CB4�� ���� ������ SSW�� ����.
 *		   [20.03.09] : - Sag RMS ���� �⺻��(GRID_UV_LEVEL_INSTANT) 90% -> 85% ����(1/30�� ������ �� �˾Ҵµ�, ���� �ȵǾ� ����)
 *		                - system.c run ���¿��� CTRL.GRT.bEnable �ٽ� false�� ����� �κ� ��Ȱ��ȭ -> Ȱ��ȭ(resync ���¿� ���� �� �ٽ� run���·� �����ϸ� ���ļ� ���� ���� �߻����� �ʴ� ���� ����)
 *	       [20.10.14] : - GFD Fault ����
 * 1.00.06
 *		   [21.05.31] : - ���� ���� ����Ʈ ����(���̸����� �� -> ��)
 *		   [21.06.30] : - ���ؾ� New SCR ������ ���� SCR ���� �ҽ��ڵ� Merge & ����. ( EPWM.c )
 * 1.00.07
 *         [22.01.04] : - AC GEN���� MC1A����� FLTH_BATT_CB1A ������ FLTH_MC1A �� ����.
 */

#define VER_MAXLEN	(32)
#define VER_VERSION	"1.00.06"   	// Major Miner Build
#define VER_VERSION_MODBUS 10006	// 65535���� ����

//=======================================================================================


/* DI POLLING PERIOD 0: 5ms 1: CC Period -�� ��񿡼� ��� �ſ� ����. */
#define FAST_DI_POLLING 0

/* Trace 1 Sec.. for FR - SPI FALSH */
#define EXTRACE_USE_SPI_FLASH 0

/* DC�� ���� GFD ON/OFF�� {SYS_OPTION}  �Ķ���͸� �̿� ��. */
/* Start - Stop Command ��� ���� {SYS_OPTION} �Ķ���ͷ� �ű�. �⺻ OFF - POSCO:ON */


#define DISABLE_AUTO_FLT_RST 0

#define USE_CAN_B_POSCO 1

/* ���� ���� ���� ���� POSCO������ Idle��Ȳ���� ������ �߻��Ѵٰ� �Ͽ� �ӽ÷� ���� ��. (����� 0�� �����ص� ���忡�� �߻� ���ϰ� ����) */
/* CE ���� ���� �� 0�� �ؾ� ��. */
#define DISABLE_CURRENT_SENSOR_FLT 1

/* PCC ���� Idle�� Gate OFF. */
#define USE_IDLE_GATE_OFF 1

#if USE_IDLE_GATE_OFF == 1
//JCNET#warning "AUTO PWM-OFF MODE"
#endif


/* Double Control */
#define DOUBLE_CONTROL 1 // 1: Enable

/* S�� ADC ����� �� ������ ������� �� ������ */
#define ADC_DECTECT_S 1

/* T�� ADC ����� �� ������ ������� �� ������ */
#define ADC_DECTECT_T 1

/* �ι��� ������ Phase Voltage���� Line To Line Voltage */
#define INV_V_USE_LTL 0

/* Static Switch( PMI�� EPWM9B - GPIO63 )��� ���� */
#define STATIC_SWITCH_GPIO 1

/* Static Switch( PWM4~6 )��� ���� */
#define STATIC_SWITCH_GPIO_PWM 0

/* Static Switch( PWM6~9 ) 0: ����, 1: ���� PWM SCR, 2: Ȳ���� ���� ���� ����  */
#define STATIC_SWITCH_GPIO_PWM6_9 2

#if STATIC_SWITCH_GPIO_PWM6_9 == 1
#warning "###### ���� SCR ���"
#elif STATIC_SWITCH_GPIO_PWM6_9 == 2
//JCNET#warning "###### Ȳ���� ���� ���� ����"
#else
#error "���õ� SCR ��� ����"
#endif

/* ���� State Machine */
#define Fixed_State_Machine

/* 1: EVE DC Charger ���, 0: ���X */
#if STABLEEN_HILL_ENABLE == 1
#define EVE_DC_CHARGER 0
#warning "HIL MODE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
#else
#define EVE_DC_CHARGER 1
#endif


/* 1: CB5, CB6, CB7 ���, 0: ���X */
#define USE_MBP_PANNEL 0

/* STBLN Power Decrease */
#define STBLN_POWER_DECREASE 0

/*
 * ON ���·� �ý��� ���� Ȥ�� ��Ÿ ������ �ý��� Shutdown ��
 * ��Ʈ �Ǿ��� ���, ���� ���°� ON�̶�� �ڵ����� �� �⵿ ��ų ������ ����
 * 0: ��� ����.
 * 1: �ڵ� RESTART
 */
#define PCS_AUTO_RESTART_ENB 1

#define COSPHI_ENB 0

//by JCNET
#if OP_MODE == DONAU
#define TR_RATIO_XY(x,y) (x/y)
#define PEBB_COUNT_MAX 1
#define INV_RATED_VOLTAGE 480.0
#define GRID_RATED_VOLTAGE 110
#define TR_RATIO TR_RATIO_XY(INV_RATED_VOLTAGE,GRID_RATED_VOLTAGE)
#define TR_RATIO_INVERSE TR_RATIO_XY(GRID_RATED_VOLTAGE,INV_RATED_VOLTAGE)
#define FILTER_INDUCTANCE_LI    90//300//150

#endif
//end

#if OP_MODE == STBLN_300K
#define PEBB_COUNT_MAX 1
#if 1
/* 208 �Ǵ� 220 */
#define TR_RATIO TR_RATIO_320_208
#define TR_RATIO_INVERSE TR_RATIO_320_208_INVERSE
#define INV_RATED_VOLTAGE 320.0
#define GRID_RATED_VOLTAGE (208)
//JCNET#warning "TR 320:208"

//#define TR_RATIO TR_RATIO_320_220
//#define TR_RATIO_INVERSE TR_RATIO_320_220_INVERSE
//#define INV_RATED_VOLTAGE 320.0
//#define GRID_RATED_VOLTAGE (220)
//#warning "TR 320:220"
#else
#define TR_RATIO TR_RATIO_320_480
#define TR_RATIO_INVERSE TR_RATIO_320_480_INVERSE
#define INV_RATED_VOLTAGE 320.0
#define GRID_RATED_VOLTAGE (480)
#endif

#define FILTER_INDUCTANCE_LI	100//300//150
#define COSPHI_QCF COSPHI_Qcf_250kW
#define COSPHI_EXRP 0
#endif	// #if OP_MODE == STBLN_300K

#if USE_1000KW_FGDC_SWITCH == 1
#warning "DC SWITCH:FGDC"
#else
//JCNET#warning "DC SWITCH:CB"
#endif

#if DEBUG_MODE != 0
#define PWM_TEST 0
#define DEBUG_PWM_SIGNAL 0
#endif


/* DELAY_US�Լ��� 1�� �� */
#define DELAY_US_1SEC 625000

/* I2C Mode */
//#define USE_I2C_SIMUL
#define USE_I2C

/* SCI ���� - 5:38400 6:57600 7:115200 (���� Modbus �ӵ��� 115200���� ���� ������, number of coil ���� �� ó�� ��û�� ���� �߸��� ���� ����) */
#define SCI_FIFO 0
#define SCI_BAUD 7

/* McBSP as SPI Master option. 0: McBSP ��� ���. 1: GPIO ���� ��� (NvSRAM) */
#define McBSP_SPI_GPIO_MODE 0

/* 2 or 3 LEVEL IGBT config */
#define IGBT_LEVEL 2

#if MCU_PCB_REV_VER >= MCU_PCB_REV_B_G
#define MCBSP_B_CS_USE_MFSX 0 /* 0: GPIO ��� 1: MFSXB */
#endif

/* GI, GC ��������� Gain �и� */
#define CURRENT_CONTROLLER_GAIN_DIV 1 //  170220 Yang �迵�� å�� ��û ���� ����� Gain �и�
#ifdef CURRENT_CONTROLLER_GAIN_DIV
//JCNET#warning "GI,GC_���������_Gain_�и�"
#endif

#if DEBUG_MODE == 1
#include <xdc/runtime/System.h>
#endif

extern volatile Bool bSystemStarted;
extern Bool bUpdateFlag;

#if DEBUG_MODE == 1
#define error()\
				{\
	    		asm("     ESTOP0");\
				}
#else
#define error()
#endif //-for (;;);\

#define ZERO 0

#ifndef TRUE
#define FALSE ((Bool)0)
#define TRUE  ((Bool)1)
#endif

#ifndef CLOSED
#define OPEN    ((Bool)0)
#define CLOSED  ((Bool)1)
#endif

#ifndef ON
#define OFF ((Bool)0)
#define ON  ((Bool)1)
#endif

#define BLACKOUT (0)

#define elif else if

//���ó�� ���� üũ �Ϸ�.
typedef union _Ltype {
	float f;
	Int32 i;
	UInt32 u;
} Ltype;

typedef union _TimePack {
//-	Uns		UnsVal;
	Uint32 UnsVal;

	struct {
		Uns LSB :16;
		Uns MSB :16;
	} WordVal;
	struct {
		Uns By0 :8; // LSB
		Uns By1 :8;
		Uns By2 :8;
		Uns By3 :8; // MSB
	} ByteVal;
	struct {
		Uns Nib0 :4; // LSB
		Uns Nib1 :4;
		Uns Nib2 :4;
		Uns Nib3 :4;
		Uns Nib4 :4;
		Uns Nib5 :4;
		Uns Nib6 :4;
		Uns Nib7 :4; // MSB
	} NibVal;
	struct {
		Uns Second :6;
		Uns Minute :6;
		Uns Hour :5;
		Uns Day :5;
		Uns Month :4;
		Uns Year :6;
	} BitVal;
} TimePack;

typedef union {
	Uns UnsVal;
	struct {
		Uns OneDigit :4;
		Uns TenDigit :3;
		Uns _rev :9;
	} ss;
	struct {
		Uns OneDigit :4;
		Uns TenDigit :3;
		Uns _rev :9;
	} min;
	struct {
		Uns OneDigit :4;
		Uns TenDigit :1;
		Uns _rev :10;
	} hh;
	struct {
		Uns OneDigit :4;
		Uns TenDigit :2;
		Uns _rev :10;
	} dd;
	struct {
		Uns OneDigit :4;
		Uns TenDigit :1;
		Uns _rev :11;
	} mm;
	struct {
		Uns OneDigit :4;
		Uns TenDigit :4;
		Uns _rev :8;
	} yy;
} Rtc;

typedef struct {
	Rtc Second;
	Rtc Minute;
	Rtc Hour;
	Rtc Day;
	Rtc Month;
	Rtc Year;
} Ds1337;

typedef struct {
	Int16 Year;
	Int16 Month;
	Int16 Date;
	Int16 Hour;
	Int16 Minute;
	Int16 Second;
	//-Bool bUpdating;
} RtcTime;

float sqrt7(float x);

#endif /* DP_COMMON_DEF_H_ */
