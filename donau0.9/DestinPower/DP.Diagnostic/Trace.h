/*
 * Trace.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef TRACE_H_
#define TRACE_H_

#include "LGS_Common.h"
#include "SYSTEM.h"

#define TRC_INT	0x100
#define TRC_UNS	0x110
#define TRC_FLOAT	0x200
#define TRC_UNS32 0x300

typedef enum
{
	TRC_00_DCLINK_VDC,
	TRC_01_DCLINK_VDC_FLT,
	TRC_02_BATT_I,
	TRC_03_BATT_I_FLT,
	TRC_04_BATT_VDC,
	TRC_05_BATT_VDC_FLT,
	TRC_06_,
	TRC_07_,
	TRC_08_,
	TRC_09_PWR_CONTROL_EQS,
	TRC_10_PWR_CONTROL_EDS,
	TRC_11_TR_I_IQS,
	TRC_12_TR_I_IDS,
	TRC_13_,
	TRC_14_,
	TRC_15_,
	TRC_16_,
	TRC_17_IVC_REF,
	TRC_18_ICC_REF,
	TRC_19_,
	TRC_20_CON_EA,
	TRC_21_CON_EB,
	TRC_22_CON_EC,
	TRC_23_CON_IA,
	TRC_24_CON_IB,
	TRC_25_CON_IC,
	TRC_26_CON_EA_OFFSET,
	TRC_27_CON_EB_OFFSET,
	TRC_28_CON_EC_OFFSET,
	TRC_29_CON_EDE_FLT,
	TRC_30_CON_EQE_FLT,
	TRC_31_CON_FREQ,
	TRC_32_CON_THETA,
	TRC_33_CON_POSITIVE_RMS_VA,
	TRC_34_CON_POSITIVE_RMS_VB,
	TRC_35_CON_POSITIVE_RMS_VC,
	TRC_36_CON_POSITIVE_INSTANT_VA,
	TRC_37_CON_POSITIVE_INSTANT_VB,
	TRC_38_CON_POSITIVE_INSTANT_VC,
	TRC_39_,
	TRC_40_GRID_EA,
	TRC_41_GRID_EB,
	TRC_42_GRID_EC,
	TRC_43_GRID_EDE,
	TRC_44_GRID_EQE,
	TRC_45_GRID_EDE_FLT,
	TRC_46_GRID_EQE_FLT,
	TRC_47_GRID_THETA,
	TRC_48_GRID_PA,
	TRC_49_GRID_PB,
	TRC_50_GRID_PC,
	TRC_51_GRID_EA_OFFSET,
	TRC_52_GRID_EB_OFFSET,
	TRC_53_GRID_EC_OFFSET,
	TRC_54_GRID_FREQ,
	TRC_55_GRID_FAST_RMS_A,
	TRC_56_GRID_FAST_RMS_B,
	TRC_57_GRID_FAST_RMS_C,
	TRC_58_GRID_Vq_PU,
	TRC_59_,
	TRC_60_CON_IDE_REF,
	TRC_61_CON_IDE,
	TRC_62_CON_IQE_REF,
	TRC_63_CON_IQE,
	TRC_64_CON_IA_REF,
	TRC_65_CON_IB_REF,
	TRC_66_CON_IC_REF,
	TRC_67_CON_VDEREF,
	TRC_68_CON_VQEREF,
	TRC_69_CON_EA_REF,
	TRC_70_CON_EB_REF,
	TRC_71_CON_EC_REF,
	TRC_72_CON_PWM_A,
	TRC_73_CON_PWM_B,
	TRC_74_CON_PWM_C,
	TRC_75_THETA_DIFF,
	TRC_76_IIVCQ_REF,
	TRC_77_IINV_VDE,
	TRC_78_IINV_VQE,
	TRC_79_,
	TRC_80_CON_VC_REF,
	TRC_81_SYS_INV_STAT,
	TRC_82_RPC_LIMIT,
	TRC_83_RPC_GRADIENT,
	TRC_84_RPC_FDPR_RATIO,
	TRC_85_GRID_RMS_FILTER_VA,
	TRC_86_GRID_RMS_FILTER_VB,
	TRC_87_GRID_RMS_FILTER_VC,
	TRC_88_SYS_PERIOD_TIME,
	TRC_89_CC_PERIOD_TIME2,
	TRC_90_CC_EXECUTION_TIME,
	TRC_91_CC_PERIOD,
	TRC_92_MPPV,
	TRC_93_FLT_IQE,
	TRC_94_MPP_VARIABLE_MIN,
	TRC_95_MVP_PERIOD_TIME,
	TRC_96_PD_CON_ERR,
	TRC_97_PD_CON_OUT,
	TRC_98_PD_CON_RAMP_OUT,
	TRC_99_EVENT_QUEUE,
	TRC_100_,
	TRC_101_,
	TRC_102_,
	TRC_103_,
	TRC_104_,
	TRC_105_,
	TRC_106_,
	TRC_107_,
	TRC_108_,
	TRC_109_,
	TRC_110_,
	TRC_111_,
	TRC_112_,
	TRC_113_,
	TRC_114_,
	TRC_115_,
	TRC_116_,
	TRC_117_,
	TRC_118_,
	TRC_119_,
	TRC_120_,
	TRC_121_,
	TRC_122_,
	TRC_123_,
	TRC_124_,
	TRC_125_,
	TRC_126_,
	TRC_127_,
	TRC_128_,
	TRC_129_,
	TRC_130_,
	TRC_131_,
	TRC_132_,
	TRC_133_,
	TRC_134_,
	TRC_135_,
	TRC_136_,
	TRC_137_,
	TRC_138_,
	TRC_139_,
	TRC_140_,
	TRC_141_,
	TRC_142_,
	TRC_143_,
	TRC_144_,
	TRC_145_,
	TRC_146_,
	TRC_147_,
	TRC_148_,
	TRC_149_,
	TRC_150_,
	TRC_151_,
	TRC_152_,
	TRC_153_,
	TRC_154_,
	TRC_155_,
	TRC_156_,
	TRC_157_,
	TRC_158_,
	TRC_159_,
	TRC_160_,
	TRC_161_,
	TRC_162_,
	TRC_163_,
	TRC_164_,
	TRC_165_,
	TRC_166_,
	TRC_167_,
	TRC_168_,
	TRC_169_,
	TRC_170_,
	TRC_171_,
	TRC_172_,
	TRC_173_,
	TRC_174_,
	TRC_175_,
	TRC_176_,
	TRC_177_,
	TRC_178_,
	TRC_179_,
	TRC_180_,
	TRC_181_,
	TRC_182_,
	TRC_183_,
	TRC_184_,
	TRC_185_,
	TRC_186_,
	TRC_187_,
	TRC_188_,
	TRC_189_,
	TRC_190_,
	TRC_191_,
	TRC_192_,
	TRC_193_,
	TRC_194_,
	TRC_195_,
	TRC_196_,
	TRC_197_,
	TRC_198_,
	TRC_199_,
	TRC_NUM
} InfoID;

enum
{
	TRC_FAULT_ONLY_STOP,
	TRC_INV_CTRL_ENB_STOP,
	TRC_INIT_INV_START, // CVC_INIT_VOLTAGE_BUILD 후 첫 pwm
	TRC_BYP_OFF_STOP,
	TRC_VI_ENB_STOP,
	TRC_PWM_OFF_STOP,
	TRC_RE_SYNC_TO_GC,
	TRC_BYP_EVT_HISTORY,
	// TMP
	TRC_FRT_ENTRY_STOP = 10

};


/*
 * 190513 june commant 웨스코 김현수 부장과 4월 출장에서 8000 -> 400 raw 데이터 표현 안하기로 함.
 * vPeak로 현상만. Trace Sample 15로 하면 약 960ms 가능. input peak, load peak만 trace 걸면 될 것으로 보임.
 * */
#define TRACE_MEMORY_SIZE 400
#define TRACE_NUM 6


typedef struct _InfoNode{
	int		iItemDataType;
	void	*pItemAddr;
	int		bInUsed;
} InfoNode;

#define TRACE_LEVEL			1
typedef struct _TracePoolNode{
	int		nIndex;
	float 	TRACE_BUF[TRACE_LEVEL][TRACE_MEMORY_SIZE];
} TracePoolNode;

typedef struct _Trace{
	int		iMemorySize;
	int		iSamplingTime;
	int		iSamplingTimeCounter;
	float	fStopTimeRatio;
	unsigned int	iStop;
	int		iStopTime;
	int		iStopTimeCounter;
	int		iTail[TRACE_LEVEL];
	int		iCurrentLevel;
	int		iNextLevel;
} Trace;

void TRC_Create( void );
//Parameter*	TRC_GetTraceBufAdr();
Bool TRC_AddNode( InfoID ID, int InfoItemDataType, void *pItemAddr );
Bool TRC_CreateTracePool( void );
void TRC_UpdateTraceChannel( void );
void TRC_StopTrace( void );
void TRC_StartTrace( void );
Bool TRC_TraceReset( void );
void TRC_UpdateParameter( void );

Bool TRC_GetTraceString( int TraceChID, int Offset, String sHex );
void TRC_ChangeTracePoolNode( int ChNum, int DstID );

/* Unused */
Bool TRC_GetItemFloatValue( InfoID ID, float *Val );


extern TracePoolNode TPOOL[TRACE_NUM];
extern Trace TRACE;
extern InfoNode INFO[TRC_NUM];

#endif /* TRACE_H_ */
