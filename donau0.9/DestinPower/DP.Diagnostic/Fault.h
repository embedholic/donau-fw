/*
 * Fault.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef FAULT_H_
#define FAULT_H_

#include "LGS_Common.h"
#include "CAN_GBI.h"

#define FLT_ON	1
#define FLT_OFF	0

/**************************************************************************/
/* Definition for FLT ID(Identification)                                  */
/**************************************************************************/

/* Heavy Fault */
#define FLTH_START_FAILURE			1
#define FLTH_BATT_OV				2
#define FLTH_BATT_OC				3
#define FLTH_BATT_UV				4
#define FLTH_BATT_MC1               5 // by JCNET CB1				5
#define FLTH_BATT_MC1A              6 // by JCNET CB1A				6
#define FLTH_BATT_POLARITY			7

#define FLTH_DC_SIDE_ABNORMAL		9
#define FLTH_INV_OV					10
#define FLTH_INV_UV					11
#define FLTH_INV_OF					12
#define FLTH_INV_UF					13
#define FLTH_INV_OC					14
#define FLTH_INV_PHASE				15
#define FLTH_INV_PAHSE_JUMP			16
#define FLTH_INV_CURR_UNBAL			17
#define FLTH_BATT_I					18
#define FLTH_PEBB_OC				19
#define FLTH_PEBB_OVERTEMP			20
#define FLTH_PEBB_FAN_FAULT			21
#define FLTH_PEBB_FUSE_FAULT		22
#define FLTH_PEBB_IGBT				23

// MC1A, CB5~7
#define FLTH_MC1A					24
#define FLTH_CB5		 			25
#define FLTH_CB6					26
#define FLTH_CB7					27
#define FLTH_EVE_FAULT				28
#define FLTH_SSW_MC2                29 // by JCNET CB4				29 // SSW side CB
#define FLTH_GRID_OV_LEVEL1			30
#define FLTH_GRID_UV_LEVEL1			31
#define FLTH_GRID_OF_LEVEL1			32
#define FLTH_GRID_UF_LEVEL1			33
#define FLTH_GRID_CB3				34
#define FLTH_GRID_OV_LEVEL2			35
#define FLTH_GRID_UV_LEVEL2			36
#define FLTH_GRID_UF_LEVEL2			37
#define FLTH_GRID_OF_LEVEL2			38
//MCB7~9
#define FLTH_MCB7					39
#define FLTH_MC8					40
#define FLTH_MC9					41
#define FLTH_SSW_OT					42
#define FLTH_TR_TEMP				43
#define FLTH_RE_TEMP				44

#define FLTH_SSW_FUSE				45
#define FLTH_ASYNC					46
#define FLTH_CAN					47

#define FLTH_EPO					49 // emergency Stop
#define FLTH_FIRE					50 // 지락
#define FLTH_GFD					51 // 지락 (DC측 지락)
#define FLTH_PARA_VERSION	 		52

/* FR */
#define FLTH_DOOR_OPEN				53 // PCS 판넬 도어 개방
#define FLTH_DC_CHARGE_FAILED   	54 // DC Link 초기 충전 실패

#define FLTH_AC_GFD			    	56 // AC 지락( PCS가 정지 중에는 51번 고장 발생시킴-DC지락으로 봄 )
#define FLTH_GRID_ZERO_VOLTAGE  	57 // Grid Zero Voltage
#define FLTH_AC_FUSE		    	58 // 사용 안됨
#define FLTH_LPMS_ESTOP		    	59 // LPMS E-STOP ( 통신 명령에 의한 )

#define FLTH_LOAD_OC		    	60 // LOAD OC

#define FLTH_PM_OVERLOAD			61
#define FLTH_ETC					62

#define FLT_HEAVY_MAX				63

/* --------------- Light Fault -------------------------*/
#define FLTL_REV81				81
#define FLTL_REV82				82
#define FLTL_OVER_MDL			83
#define FLTL_PEBB_1_STOP		84
#define FLTL_PEBB_2_STOP		85
#define FLTL_PEBB_3_STOP		86
#define FLTL_PEBB_4_STOP		87
#define FLTL_PEBB_5_STOP		88
#define FLTL_PEBB_6_STOP		89
#define FLTL_PEBB_7_STOP		90
#define FLTL_PEBB_8_STOP		91

#define FLTL_DPWM_ON			92
#define FLTL_GFD				93
#define FLTL_SPD1				94
#define FLTL_SPD4				95 // 141201 SPD
#define FLTL_BF_PROTECT			96 // 151217 - GS네오텍
#define FLTL_BMS_COMM_ERR		97
#define FLTL_WAR_REV1			98 // EVE CAN Fail
#define FLTL_WAR_REV2			99
#define	FLTL_CB4				100
#define	FLTL_CB5				101
#define	FLTL_CB6				102
#define	FLTL_CB7				103

#define FLTL_GBI_PARAM_ERR 		110
#define FLTL_OC_WARN_IS 		111
#define FLTL_GRID_OV			112
#define FLTL_GRID_UV			113
#define FLTL_GRID_OF			114
#define FLTL_GRID_UF			115

// FOR POSCO DEBUG
#define FLTL_RECLOSER_ON 		116

#define FLTL_GRID_OF2_SEAMLESS	117
#define FLTL_GRID_UF2_SEAMLESS	118

#define FLTL_TEST_RUN		    120

#define FLTL_INVALID_PARAM		125

#define FLTL_SMPS   			134
#define FLTL_SMPS_DC   			135

#define FLTL_SSW_OC				136
#define FLTL_PM_OVERLOAD		137
#define FLTL_EVE_WARNING		138

/* 고장 140 번 사용 금지. HMI가 Communication Error번호로 사용*/
#define FLTL_DOOR_AC_OPEN		142
#define FLTL_DOOR_DC_OPEN		143
#define REV144					144
// 145번 부터는 CMT에서 파싱하지 못함. (이유: 80 + 64Bit)

#define FLT_LIGHT_MAX			145

#define EVT_ON				160
#define EVT_OFF				161
#define EVT_FLT_RESET		162
#define EVT_PARA_INIT		163
#define EVT_HISTORY_INIT	164
#define EVT_DATA_INIT		165
#define EVT_TESTRUN_ON		166
#define EVT_TESTRUN_OFF		167
#define EVT_AUTO_RESTART	168
#define EVT_BOOT_OK			169

#if 0 //by JCNET
#define EVT_DS1_ON					170 //
#define EVT_DS1_OFF					171 //
#define EVT_MC1A_ON					172 //
#define EVT_MC1A_OFF				173 //
#define EVT_CB3_ON					174 //
#define EVT_CB3_OFF					175 //
#define EVT_EX_MCCB1_ON				176 //
#define EVT_EX_MCCB1_OFF			177 //
#define EVT_EX_MCCB2_ON				178 //
#define EVT_EX_MCCB2_OFF			179 //
#define EVT_EX_MCCB3_ON				180 //
#define EVT_EX_MCCB3_OFF			181 //
#define EVT_MCB7_ON					182 //
#define EVT_MCB7_OFF				183 //
#define EVT_CB4_ON					184
#define EVT_CB4_OFF					185
#define EVT_MCB8_ON					186
#define EVT_MCB8_OFF				187
#define EVT_MCB9_ON					188
#define EVT_MCB9_OFF				189
#define EVT_CB2_ON                  190
#define EVT_CB2_OFF                 191

#define EVT_SSW_ON					215 //
#define EVT_SSW_OFF					216
#else
#define EVT_MC1A_ON                 170
#define EVT_MC1A_OFF                171
#define EVT_MC1_ON                  172
#define EVT_MC1_OFF                 173
#define EVT_MC2_ON                  173
#define EVT_MC2_OFF                 174
#define EVT_CB1_ON                  175
#define EVT_CB1_OFF                 176
#define EVT_CB2_ON                  177
#define EVT_CB2_OFF                 178
#define EVT_CB3_ON                  179
#define EVT_CB3_OFF                 180
#define EVT_SSW_ON                  215 //
#define EVT_SSW_OFF                 216
#endif
#define EVT_WD_TIMEOUT				220

#define EVT_CHG_WH_OVER_CLEAR		230
#define EVT_DCHG_WH_OVER_CLEAR		231

#define EVT_PCC_NOT_BLACKOUT		240
#define EVT_PCC_NOT_OK				241
#define EVT_SW_AND_CB_NOT_ON		242
#define EVT_SW_OR_CB_NOT_OFF		243
#define EVT_PCC_LEVEL2				244
#define EVT_PWM_OFF_RESYNC			245
#define EVT_PCC_LEVEL1				246
#define EVT_GI_A					247
#define EVT_GI_B					248

#define	EVT_EVE_STAT_STOP			250
#define	EVT_EVE_STAT_ON				251
#define	EVT_EVE_STAT_CHARGING		252
#define	EVT_EVE_STAT_FAULT			253
//#define	EVT_EVE_WARNING				254
#define	EVT_EVE_CMD_STOP			255
#define	EVT_EVE_CMD_ON				256
#define EVT_EVE_CMD_FAULT_RESET		257

#define EVT_STAT_INV_SCR_ON			298
#define EVT_STAT_INV_EVE_DC_CHARGE	299

#define EVT_STAT_INV_FAULT			300
#define EVT_STAT_INV_STOP			301
#define EVT_STAT_INV_DC_CHARGE		302
#define EVT_STAT_INV_AC_GENERATE	303
#define EVT_STAT_INV_START_SYNC		304
#define EVT_STAT_INV_RUN			305
#define EVT_STAT_INV_ISLANDING		306
#define EVT_STAT_INV_RESYNC			307
#define EVT_STAT_INV_TEST			308
#define EVT_STAT_INV_BYP_EVT_OPERATION 309

#define EVT_STAT_BATTCB_RESET		310
#define EVT_STAT_BATTCB_OFF			311
#define EVT_STAT_BATTCB_ON			312
#define EVT_STAT_BATTCB_TRIP		313

#define EVT_STAT_CB4_RESET			314
#define EVT_STAT_CB4_OFF			315
#define EVT_STAT_CB4_ON				316
#define EVT_STAT_CB4_TRIP			317

#define EVT_STAT_CB3_RESET			318
#define EVT_STAT_CB3_OFF			319
#define EVT_STAT_CB3_ON				320
#define EVT_STAT_CB3_TRIP			321

#define EVT_STATE_AUTORESET_NORMAL		324
#define EVT_STATE_AUTORESET_DELAY		325
#define EVT_STATE_AUTORESET				326
#define EVT_STATE_AUTORESET_CLR_CMD		327

#define EVT_NODE_FAULT					330
#define EVT_NODE_OFF					331
#define EVT_NODE_STANDBY				332
#define EVT_NODE_RUN					333
#define EVT_NODE_MPP_INIT				334

#define EVT_MASTER_SLAVE_ON				340
#define EVT_MASTER_SLAVE_OFF			341
#define EVT_MASTER_SLAVE_NONE			342
#define EVT_MASTER_SLAVE_ON_FAIL		343
#define	EVT_MASTER_SLAVE_OFF_FAIL		344

#define EVT_MASTER_SLAVE_ON_SUCCESS		345
#define	EVT_MASTER_SLAVE_OFF_SUCCESS	346

#define EVT_MASTER_CMD_ON				347
#define EVT_MASTER_CMD_OFF				348

#define EVT_MASTER_FAULT_NONE			349
#define EVT_MASTER_FAULT_LOOK			350
#define EVT_MASTER_FAULT_WAIT			351
#define EVT_MASTER_FAULT_CHG			352

// CTRL_BYP_EVT_OPERATION
#define BYP_OVERLOAD					381
#define BYP_OVERLOAD_CLEAR				382
#define EVT_DEBUG_Charging_GI3			383
#define PM_OVERLOAD						384
#define SCR_FAIL						385

#define FLT_STAT_MAXNUM			15

typedef enum {
 FLT_STATE_AUTORESET_NORMAL,
 FLT_STATE_AUTORESET_DELAY,
 FLT_STATE_AUTORESET,
 FLT_STATE_AUTORESET_CLR_CMD
}AUTO_RST_STATE;

/**********************************************************************/
/* Structure Definition for Fault2 Status                             */
/**********************************************************************/
typedef	union
{
	GBI_COMM_RX_MSG_M0 data;
}FLT_PEBB_ST;

typedef struct
{
	Uns uFlag;      // 고장 일괄 플래그
	Uns uHeavyFlag; // 중고장 플래그
	Uns uLightFlag; // 경고장 플래그
	AUTO_RST_STATE uState;
	Uns uClearDelay;
	Uns uInitDelay;
	Uns uAutoResetDelayCnt;
	Uns uAutoResetCnt;
	Uns uAutoResetIntervalCnt;
	Bool bAutoResetInit;
	Bool bAutoResetCntClearTimerEnb;
	Uns uAutoResetCntClearTimer;
	Bool bGridFailureEvent;
	char cNthStatus[FLT_STAT_MAXNUM];
} Flt;

/**************************************************************************/
/* FLT Methods                                                            */
/**************************************************************************/
void FLT_Create( void );
void FLT_Initialize( void );

void FLT_CheckHW(void);
void FLT_Handling( void );
void FLT_Raise( int FltNum );
Uns FLT_GetStatus( void );
Uns FLT_GetHeavyStatus( void );
Uns FLT_GetLightStatus( void );
Uns FLT_GetEachStatus( int FltNum );

void FLT_Clear( int FltNum );
String FLT_GetStatusString(void);
Bool FLT_ConVoltageFailure( void );
Bool FLT_BattVoltageFailure( void );
Bool FLT_GridVoltageFailure( void );

void FLT_CabinetTemp(void);
void FLT_IncreseClearDelayCnt(void);
void FLT_ResetClearDelayCnt(void);
void FLT_IncreseInitDelayCnt(void);
void FLT_UpdateAutoResetState(void);
void FLT_UpdateAutoResetInit(void);
void FLT_SetGridFailureEvent( Bool bSetVal );
Bool  FLT_GetGridFailureEvent( void );
Bool FLT_HeavyFaultExceptGridFaults( void );

#define FLT_BIT_NUM 4

extern Flt FLT;
extern Uint32 uFault[FLT_BIT_NUM];
extern FLT_PEBB_ST FLT_PEBB[PEBB_COUNT_MAX];

extern Uns FLT_HEAVY_TABLE[80];

#endif /* FAULT_H_ */
