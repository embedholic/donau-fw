/*
 * LGS_Common.h
 *
 *  Created on: 2012. 11. 1.
 *      Author: destinPower
 *
 */

/*
 * 주의 사항
 * #include "DSP28x_Project.h"	를 Include 할 경우에 LGS_Common.h 보다 위에 있으면 컴파일 오류가 발생한다.
 * SYS/BIOS - LIB 타입은 디버깅 가능한 Custom임. 느린 성능 및 더 큰 코드와 풋 프린트 발생.
 */

#ifndef DP_COMMON_DEF_H_
#define DP_COMMON_DEF_H_

#include "MathConst.h"
#include "Types.h"

#define TI_28346
#ifdef TI_28346
#include <xdc/std.h>
#endif

/* private 함수 선언 define */
#define private static
#define public 			/* public method */
#define out 			/* parameter direction is out */


/* MCU BOARD VERSION_회로도 버젼 ======================================================== */
/* --------------- LIST -------------------*/
#define MCU_PCB_REV_A_D 0x0  /* Init Version */
#define MCU_PCB_REV_B_G 0x1  /* Rev.F 이후 부터 SRAM&DATA FLASH GPIO 27 <-> 40 변경 */
						     /* SRAM의 CS는 MCBSP 모듈의 MFSXB(CS 40->27)로 수정하여 운용하고, DATA FLASH는 GPIO27을 40으로 수정하여 CS모드로 사용. */
  	  	  	  	             /* ADC MUX 추가 됨 */
#define MCU_PCB_REV_I_I 0x2
#define MCU_PCB_REV_J 	0x3  /* SRAM이 MR20H40 MRAM으로 교체 됨. */

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
 * 2: MCU1 - DP_LGS_DSP_SW_SVN_EX  ( GT + GI (BYPMODE, BlackStart, 제어 게인 분리 )
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
#define DBUG_MODE 2 // 1:dio는 실제로 동작, adc는 가상
					// 2:dio .adc 모두 가상
					// 3:dio만 가상 - MC4 ON/OFF Input/Output만 실제로 사용 함.

#if DBUG_MODE == 3
	/* 포스코의 두대 연동 HILL 테스트를 위해 bypass 전압을 capacity 전류로 사용할지 여부 - 포스코 신안 시험 시 사용 */
	#define HILL_CAPI_TO_BYPV 0
	/* HILL- CapI을 GenV로 사용할지 여부 - BYP 30 PCS1,2 */
	#define HILL_CAPI_TO_GENV 0
	/* HILL- CAN통신 입력으로 ACB3 Status수신 - BYP 30 PCS1 ON*/
	#define HILL_ACB3_STATUS_TO_CAN 0
	/* HILL- CAN통신 입력으로 ACB3 Status수신 - BYP 30 PCS2 ON*/
	#define HILL_ACB3_STATUS_FROM_CAN 0

	/* HILL- BypV을 GenV로 사용할지 여부 */
	#define HILL_BYPV_TO_GENV 0
	/* CB2 신호를 Bypass Signal로 내보낼지 여부 - ES-LV 타입 HILL 시험 시 사용 */
	#define HILL_CB2_TO_BYPASS 0
	/* MC6 가상 동작 모드 */
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
#define STBLN_500K 	500 /* TL 타입 */
#define STBLN_1250K 1200

/* --------------- SET -------------------*/
//by JCNET #define OP_MODE STBLN_300K
#define DONAU   999
#define OP_MODE DONAU
/* ===================================================================================== */


#define MAJOR_MODEL 0x100 /* SAVEEN-MG */

#define STABLEEN
/*
 *  모델 별 옵션 설정
 */
#define SP_NORMAL 0
#define SP_MODEL SP_NORMAL

#define RE_SYNC_DEBUG		/* 16.06.29 Yang Weak-Grid에서 Re-sync가 완료되지 않았음에도 Run으로 넘어가는 문제 해결  Start Sync와 동일한 방법 적용 */
//#define BYP_EVT_OPERATION	/* 16.06.30 Yang MG 및 STABLEEN의 STS & 병렬 CB 동작	*/

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


/* ====== 버젼 정보 ================================================================ */
/* PARAMETER VERSION HISTORY
 * MSB 4bit로 Model을 파싱한다. 기존의 모든 파라미터 테이블 버전보다 높게 설정. NvSRAM Parameter 버젼 검사시 사용하므로, hex 값)
 * 0x1001 : 2013~2014.12.11일
 * 0x1002 : 2014.12.11 FR 누적발전량 기능 추가로 인한 파라미터 개수 추가(ESS TAB)
 * 0x1003 : 2015.08.28 MG-LV 모델을 위한 파라미터 추가.
 * 					   GAIN 추가 및 변경. (GenV추가, BypassV Analog->Gain tab으로 변경)
 * 					   AI 1에서 새로 추가한 파라미터가 사용 됨.
*/

#define PARA_VERSION 0x1004 // by JCNET for version matching 0x1003
#define PARA_VERSION_MODBUS 1004 // by JCNET 1003


/*
 * MCU VERSION HISTORY
 * 1.00.01 [16.04.11] :
 * 						- 160411 MG 모델(Bypass Mode20) 소스 분리 시작.
 * 						- 		 ModbusAddr0(Manufacture)에 MAJOR_MODEL 타입 적용
 * 						- 		 Generator OV, UV, OF, UF 고장 추가(파라미터, 고장 목록)
 * 						-		 MG 모델용이 아닌 Bypass 모드를 삭제 하지 않음( 참고용으로 사용 ). 추후에 적절히 제거해야 함.
 * 						- 160523 CB2 TEST MODE
 * 						- 180409 HILS 시험 시 DI fault 제거
 * 		   [18.06.25] : - 부하 과전류 검출 및 고장 추가
 * 		   				- Fault 발생 시 ADO8 활성화 추가
 * 		   				- Sag 발생 시 DDO4 활성화 추가
 * 		   [18.07.31] : - Sag 검출 VRMS도 추가
 * 		   				- 계통 고장(DC UV) 시 자동 재기동
 * 		   [18.08.16] : - CTRL_REV_12 -> CTRL_SEAMLESS_CNT
 * 		   				- Stop -> SCR ON 조건에 !Grid UV Warning 추가
 * 		   [18.08.31] : - Run -> Islanding 제어게인 변경
 * 1.00.02 [18.12.18] :
 * 						- 계통고장 자동 재기동
 * 		   [19.01.16] :
 * 		   				- system stop -> scr on USE_MBP_PANNEL == 1 부분 수정
 * 		   [19.03.05] :
 * 		   				- DC UV 자동 clear에서 DC 전압 조건 없앰
 * 		   [19.03.06] :
 * 		   				- SagEventHistory 수정
 * 		   				- MAX_V_RMS 초기화
 * 		   				- Sag 발생 시 보상전압 추가
 * 		   [19.03.08] :
 * 		   				- CTRL_INV_SEAMLESS_Reset시(SYS_INV_STATE_TRAN_STOP에서 사용) ctrl_seamless_uv_ov_cnt_min값 초기화 제거(파라미터 값으로 적용되야 하는데, 초기값으로 적용됨)
 * 		   				- MCU SW Ver. 1.00.02 -> 1.00.03 Update
 * 		   [19.03.13] :
 * 		   				- OC 발생 시 PWM 먼저 OFF하고, SSW ON(Y2 미적용 / Y1 적용)
 * 		   				- Sag Time offset 초기값 256 -> 200 수정
 * 		   [19.04.03] :
 * 1.00.04              - system.c SYS_INV_STATE_TRAN_FAULT에 SYS_SetInverterCommand(OFF); 제거
 *         [19.04.04] :
 *                      - 중고장 발생 시 seamless.c에서 SCR 제어 안하도록 수정
 *         [19.04.08] :
 *                      - seamless.c에 계통 과전압 레벨 파라미터 기본값 rms 110 -> 115 / 피크 115 -> 130 / Inv OV 120 -> 140
 * 1.00.05
 * 		   [19.05.13] : - SagEvent Duration 시간 Input 전압기준으로 측정되도록 수정(Wesco 요청), -200 Offset 제거, Trace Buf 8000->400
 * 		                - Islanding bpy 안정화 체크 시간 200ms->100ms. islanding 100ms + resync 50ms
 * 		   [19.08.08] : - seamless.c pk검출 pcc -> byp 수정
 * 		                - ADC offset 제거
 * 		   [19.08.13] : - System에서 fast rms 전압이 기준 전압보다 내려가지 않을 경우 Islanding -> resync 가는 시간만큼 Sag Cnt에서 빼줌
 * 		   [19.12.10] : - DDI2번 DC door warning X -> CB2 상태
 * 		                - CB5,6,7 -> EX MCCB 1,2,3 명칭 변경
 * 		                - AC SMPS -> SMPS 명칭 변경
 * 		   [19.12.11] : - Trace 파라미터 기본값 변경
 * 		                - EVE_MAXIMUM_CHARGE_VOLTAGE 기본값 850 -> 900 변경
 * 		                - GRID_UV_LEVEL2_INSTANT 기본값 85 -> 82 변경
 * 		                - LOAD_OC_LEVEL 기본값 110 -> 125 변경
 * 		   [19.12.12] : - GI_V_REF 기본값 330
 * 		                - Grid OV LV2 자동 clear되는 버그 수정
 * 		   [19.12.26] : - CTRL_GEN_Proceed 마지막부분에 black out 시 SCR off 하는 부분 삭제(fault 상황에서 system 상태가 fault로 전이하기 전까지 SCR on, off 반복되는 버그 수정)
 * 		                - Byp 전압 pk값 검출 ACP.BYP.v_pk -> EXCTRL.bypEqe로 수정
 * 		   [20.01.02] : - TR 320_480 장비 추가
 * 		                - HILS 모드일 때 EVE_DC_CHARGER_ENABLE 파라미터 기본값 0
 * 		   [20.01.10] : - 정격부하에서 복전 시 충전 안됨 -> 부하조건 걸리면 50kW의 절반(25kW)로 충전되게 수정
 * 		                - CC_Mode_CP_P 파워 10%로 줄이는부분 삭제
 * 		   [20.01.11] : - HILS 모드에서 SYS_DPWM_OFFTEMP 파라미터 기본값 1
 * 		                - 계통 주파수 변경 시 CTRL_INV_SYNC_UpdateParameter, CTRL_INV_PQC_UpdateParameter, CTRL_INV_PRC_UpdateParameter, CTRL_INV_DROOP_UpdateParameter 함수 실행하도록 수정
 * 		   [20.01.13] : - Sag RMS 검출 기본값 90% -> 85%
 * 		   [20.01.30] : - 480Vgrid 장비 센싱게인 수정
 *		   [20.02.05] : - DS1과 동일하게 CB3, CB4도 OFF일때 신호를 주기적으로 생성. STOP상태에서 CB4가 붙지 않으면 SSW를 붙임.
 *		   [20.03.09] : - Sag RMS 검출 기본값(GRID_UV_LEVEL_INSTANT) 90% -> 85% 변경(1/30에 변경한 줄 알았는데, 변경 안되어 있음)
 *		                - system.c run 상태에서 CTRL.GRT.bEnable 다시 false로 만드는 부분 비활성화 -> 활성화(resync 상태에 빠진 후 다시 run상태로 전이하면 주파수 관련 고장 발생하지 않는 버그 수정)
 *	       [20.10.14] : - GFD Fault 삭제
 * 1.00.06
 *		   [21.05.31] : - 역상 검출 포인트 변경(사이리스터 후 -> 전)
 *		   [21.06.30] : - 시준씨 New SCR 버전과 기존 SCR 버전 소스코드 Merge & 정리. ( EPWM.c )
 * 1.00.07
 *         [22.01.04] : - AC GEN에서 MC1A고장시 FLTH_BATT_CB1A 고장을 FLTH_MC1A 로 변경.
 */

#define VER_MAXLEN	(32)
#define VER_VERSION	"1.00.06"   	// Major Miner Build
#define VER_VERSION_MODBUS 10006	// 65535까지 가능

//=======================================================================================


/* DI POLLING PERIOD 0: 5ms 1: CC Period -실 장비에서 통신 매우 끊김. */
#define FAST_DI_POLLING 0

/* Trace 1 Sec.. for FR - SPI FALSH */
#define EXTRACE_USE_SPI_FLASH 0

/* DC측 벤더 GFD ON/OFF는 {SYS_OPTION}  파라미터를 이용 함. */
/* Start - Stop Command 사용 여부 {SYS_OPTION} 파라미터로 옮김. 기본 OFF - POSCO:ON */


#define DISABLE_AUTO_FLT_RST 0

#define USE_CAN_B_POSCO 1

/* 전류 센서 고장 제거 POSCO측에서 Idle상황에서 고장이 발생한다고 하여 임시로 제거 함. (현재는 0로 설정해도 현장에서 발생 안하고 있음) */
/* CE 인증 시험 시 0로 해야 함. */
#define DISABLE_CURRENT_SENSOR_FLT 1

/* PCC 에서 Idle시 Gate OFF. */
#define USE_IDLE_GATE_OFF 1

#if USE_IDLE_GATE_OFF == 1
//JCNET#warning "AUTO PWM-OFF MODE"
#endif


/* Double Control */
#define DOUBLE_CONTROL 1 // 1: Enable

/* S상 ADC 검출로 할 것인지 계산으로 할 것인지 */
#define ADC_DECTECT_S 1

/* T상 ADC 검출로 할 것인지 계산으로 할 것인지 */
#define ADC_DECTECT_T 1

/* 인버터 전압을 Phase Voltage에서 Line To Line Voltage */
#define INV_V_USE_LTL 0

/* Static Switch( PMI의 EPWM9B - GPIO63 )사용 여부 */
#define STATIC_SWITCH_GPIO 1

/* Static Switch( PWM4~6 )사용 여부 */
#define STATIC_SWITCH_GPIO_PWM 0

/* Static Switch( PWM6~9 ) 0: 없음, 1: 예전 PWM SCR, 2: 황동옥 수석 개발 보드  */
#define STATIC_SWITCH_GPIO_PWM6_9 2

#if STATIC_SWITCH_GPIO_PWM6_9 == 1
#warning "###### 기존 SCR 모드"
#elif STATIC_SWITCH_GPIO_PWM6_9 == 2
//JCNET#warning "###### 황동옥 수석 개발 보드"
#else
#error "선택된 SCR 모드 없음"
#endif

/* 개정 State Machine */
#define Fixed_State_Machine

/* 1: EVE DC Charger 사용, 0: 사용X */
#if STABLEEN_HILL_ENABLE == 1
#define EVE_DC_CHARGER 0
#warning "HIL MODE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
#else
#define EVE_DC_CHARGER 1
#endif


/* 1: CB5, CB6, CB7 사용, 0: 사용X */
#define USE_MBP_PANNEL 0

/* STBLN Power Decrease */
#define STBLN_POWER_DECREASE 0

/*
 * ON 상태로 시스템 정전 혹은 기타 이유로 시스템 Shutdown 후
 * 부트 되었을 경우, 이전 상태가 ON이라면 자동으로 재 기동 시킬 것인지 설정
 * 0: 사용 안함.
 * 1: 자동 RESTART
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
/* 208 또는 220 */
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


/* DELAY_US함수의 1초 값 */
#define DELAY_US_1SEC 625000

/* I2C Mode */
//#define USE_I2C_SIMUL
#define USE_I2C

/* SCI 설정 - 5:38400 6:57600 7:115200 (현재 Modbus 속도를 115200으로 하지 않으면, number of coil 변경 후 처음 요청된 값은 잘못된 값을 수신) */
#define SCI_FIFO 0
#define SCI_BAUD 7

/* McBSP as SPI Master option. 0: McBSP 모듈 사용. 1: GPIO 모드로 사용 (NvSRAM) */
#define McBSP_SPI_GPIO_MODE 0

/* 2 or 3 LEVEL IGBT config */
#define IGBT_LEVEL 2

#if MCU_PCB_REV_VER >= MCU_PCB_REV_B_G
#define MCBSP_B_CS_USE_MFSX 0 /* 0: GPIO 모드 1: MFSXB */
#endif

/* GI, GC 전류제어기 Gain 분리 */
#define CURRENT_CONTROLLER_GAIN_DIV 1 //  170220 Yang 배영상 책임 요청 전류 제어기 Gain 분리
#ifdef CURRENT_CONTROLLER_GAIN_DIV
//JCNET#warning "GI,GC_전류제어기_Gain_분리"
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

//사용처에 대한 체크 완료.
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
