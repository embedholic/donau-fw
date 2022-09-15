#include <xdc/std.h>
#include <string.h>
#include "Modbus232.h"
#include "DSP2834x_Device.h"
#include "DSP2834x_Examples.h"

#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include "SCI.h"
#include "Circ.h"
#include "CC.h"
#include "Fault.h"
#include "System.h"
#include "Trace.h"
#include "Event.h"
#include "Trace.h"
#include "MCCB.h"
#include "PRM_PCS.h"
#include "MCB.h"
#include "RtcTime.h"
#include "Modbus232_func.h"
#include "CTRL_MODE.h"

unsigned int lastPos = 0;
unsigned int lastPosTrcBuf = 0;
unsigned int iTraceAutoRstCnt = 0;

#define NUMOFREG_MAX 110

extern CIRC_Obj		circSCI_RXBUF[SCI_COUNT];
extern CIRC_Obj		circSCI_TXBUF[SCI_COUNT];

static volatile int			m_bModbus232StopFlag = 0;
int bPcsCommandLocked = 1;
int iInputTemp = 0;

#define TOTAL_REGS_SIZE		(43999)

enum{
	GRP_PCS_A,
	GRP_TRACE,
	GRP_REAL_TIME_DATA,
	GRP_EVENT_DATA,
	GRP_PARAMETER,
	GRP_PCS_CTRL,
	GRP_ETC,
	GRP_TRACE_EX,
	GRP_MAX
};

unsigned int g_uiRegGrpTable[GRP_MAX][2] =
{
	//Start Address, End Address ( No Modbus Offset:40001 )

	{0000, 199}, //GRP_PCS_A,
	{1000, 1399}, //TRACE
	{1500, 1699}, //GRP_REAL_TIME_DATA
	{1900, 1902}, //GRP_EVENT_DATA
	{2000, 2999}, //GRP_PARAMETER
	{3000, 3499}, //GRP_PCS_CTRL,
	{3500, 3999}, //GRP_ETC,
	{10000, 30000}, //GRP_TRACE_EX,
};	
#define CTRL_BUF_MAX 20
#define ETC_BUF_MAX 5
unsigned int* g_pU16RegTrace;				//TRACE
Uint16		  strEventBuffer[3]; 			//EVENT DATA. 0: His ID  1,2: TimeStamp(LSB First)
unsigned int  g_u16RegPCSCtrl[CTRL_BUF_MAX];			//Inverter Control
unsigned int  g_u16RegETC[ETC_BUF_MAX];				//ETC
            
int iTraceReqNum, iTraceReqLevel;

#define BUFFER_SIZE 256

// frame[] is used to receive and transmit packages.
// The maximum serial ring buffer size is 128
static unsigned char frame[BUFFER_SIZE];
static unsigned int holdingRegsSize; // size of the register array
static unsigned char broadcastFlag;
static unsigned char slaveID;
static unsigned char function;
static unsigned int errorCount;
static unsigned int T1_5; // inter character time out
static unsigned int T3_5; // frame delay

// function definitions
static void exceptionResponse(unsigned char exception);
static unsigned int calculateCRC(unsigned char bufferSize);
static void sendPacket(unsigned char bufferSize);
static int GetParamChannelIdx(int ModbusId);

void Modbus232_SetId(unsigned int id)
{
	if( id >= 255 || id == 0 )
	{
		id = 1;
		//-PRM_PCS[DGT_MOD_RTU_ID].iValue = 1;
	}

	slaveID = id;
}
int findRegGroup(unsigned int addr)
{
	int group;
	int foundgroup = -1;
	unsigned int grp_start_addr, grp_end_addr;

	for(group=0; group<GRP_MAX ; group++)
	{
		grp_start_addr = g_uiRegGrpTable[group][0];
		grp_end_addr = g_uiRegGrpTable[group][1];

		if( (grp_start_addr <= addr) && (addr <= grp_end_addr) )
		{
			foundgroup = group;
			break;
		}
	}

	return foundgroup;
}

/*
 * READ ===============================================================================
 */

int trace_req_pos = 0;
int trace_req_pos_noTail = 0;

int readRegData(int group, int group_offset, unsigned int *rReg)
{
	Uint16 data = 0;
	int temp = 0;
	int reqPos = 0;

	switch(group)
	{
	//FIXME 메모리 보호, Range Limit
	case GRP_PCS_A			:
		//-130827data = g_u16RegPCSA[group_offset];
		data = MT_getPcs_A(group_offset);
		break;
	case GRP_TRACE			:
			if( group_offset >= 0 && group_offset <= 400)
			{
				// TRACE 버퍼에서의 데이터 시작 지점 search
				temp = (TRACE.iTail[iTraceReqLevel] + 1);
				if ( temp >= TRACE.iMemorySize )
					temp = temp - TRACE.iMemorySize;

				// 데이터 요청 지점 확인.
				reqPos = temp + (group_offset / 2);
				if ( reqPos >= TRACE.iMemorySize )
					reqPos = reqPos - TRACE.iMemorySize;

				reqPos *= 2;// uint16 * 이므로, float address의 2배가 되었다. 따라서 reqPos *2를 수행.

				if( (group_offset % 2) == 0 ) // IEEE754-float MSB 순서로 보내기 위해.
					data = g_pU16RegTrace[reqPos+1];
				else
					data = g_pU16RegTrace[reqPos];
			}
			else
				return -1;
		break;
	case GRP_REAL_TIME_DATA		:
		data = MT_getRealTimeData(group_offset); //(int)(*(INFO[group_offset].pItemAddr) * 10 + 0.5);
		break;
	case GRP_EVENT_DATA 	:
		data = strEventBuffer[group_offset];
		break;
	case GRP_PARAMETER:
		temp = GetParamChannelIdx(group_offset);
		if( temp == -1)
			return -1;

		if( temp < PARAM_NUM)
		{
			data = PRM_PCS[temp].iValue;
		}

		break;
	case GRP_PCS_CTRL		:
		{
			switch( group_offset )
			{
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
				data = g_u16RegPCSCtrl[group_offset];
				break;
			case 6:  data = (unsigned int)PRM_PCS[GC_IS_MODE].iValue; break;
			case 7:  data = (unsigned int)PRM_PCS[BATT_PEBB_FAULT_MODE].iValue; break;
			case 15: data = (unsigned int)PRM_PCS[BATT_OV_LEVEL].iValue; break;
			case 16: data = (unsigned int)PRM_PCS[BATT_UV_LEVEL].iValue; break;
			case 17: data = (unsigned int)PRM_PCS[BATT_OC_LEVEL].iValue; break;
			case 18: data = (unsigned int)PRM_PCS[INV_OC_LEVEL].iValue; break;
			case 19: data = (unsigned int)PRM_PCS[GRID_OV_LEVEL1].iValue; break;
			case 20: data = (unsigned int)PRM_PCS[GRID_UV_LEVEL1].iValue; break;
			case 41: data = (unsigned int)PRM_PCS[BATT_V_RANGE_MAX].iValue; break;
			case 42: data = (unsigned int)PRM_PCS[BATT_V_RANGE_MIN].iValue; break;
			case 43: data = (unsigned int)PRM_PCS[BATT_ORDER_SOURCE].iValue; break;
			case 44: data = (unsigned int)PRM_PCS[BATT_REF_VAL_MODE].iValue; break;
			case 45: data = (unsigned int)PRM_PCS[BATT_LOCAL_CONTROL_MODE].iValue; break;
			case 46: data = (unsigned int)PRM_PCS[BATT_LOCAL_POWER_FLOW].iValue; break;
			case 47: data = (unsigned int)PRM_PCS[BATT_LOCAL_CONTROL_OPTION].iValue; break;
			case 48: data = (unsigned int)PRM_PCS[BATT_LOCAL_IVC_V_REF_CHG].iValue; break;
			case 49: data = (unsigned int)PRM_PCS[BATT_LOCAL_IVC_V_REF_DCHG].iValue; break;
			case 50: data = (unsigned int)PRM_PCS[BATT_LOCAL_ICC_I_REF_CHG].iValue; break;
			case 51: data = (unsigned int)PRM_PCS[BATT_LOCAL_ICC_I_REF_DCHG].iValue; break;
			case 52: data = (unsigned int)PRM_PCS[BATT_LOCAL_PCC_P_REF_CHG].iValue; break;
			case 53: data = (unsigned int)PRM_PCS[BATT_LOCAL_PCC_P_REF_DCHG].iValue; break;
			case 54:
			case 55:
			case 56:
			case 57:
			case 58:
			case 59:
#if PARA_VERSION > 0x1002
			case 60: data = (unsigned int)PRM_PCS[BATT_POWER_METER_P].iValue; break;
			case 61: data = (unsigned int)PRM_PCS[BATT_POWER_METER_Q].iValue; break;
#endif
			case 62:
			case 63:
				break;
			case 64: data = (unsigned int)PRM_PCS[BATT_REMOTE_PCC_P_REF].iValue; break;
			case 65: data = (unsigned int)PRM_PCS[BATT_REMOTE_PCC_Q_REF].iValue; break;
			case 66: data = (unsigned int)PRM_PCS[BATT_IVC_PCC_CURRENT_LMT_CHG].iValue; break;
			case 67: data = (unsigned int)PRM_PCS[BATT_IVC_PCC_CURRENT_LMT_DCHG].iValue; break;
			case 68: data = (unsigned int)PRM_PCS[INV_CURRENT_LMT].iValue; break;
			case 70: // 년월
				data = PRM_PCS[TIME_MONTH].iValue;
				data += (PRM_PCS[TIME_YEAR].iValue - 2000) << 8;
				break;
			case 71: // 일시
				data = PRM_PCS[TIME_HOUR].iValue;
				data += (PRM_PCS[TIME_DAY].iValue) << 8;
				break;
			case 72: // 분초
				data = PRM_PCS[TIME_SECOND].iValue;
				data += (PRM_PCS[TIME_MINUTE].iValue) << 8;
				break;
			default: data = 0; break;
			}

			break;
		}
	case GRP_ETC			: //if( group_offset < ETC_BUF_MAX) data = g_u16RegETC[group_offset];
		// BESS 탭 200개 이상 부터는 맵을 쓸 수 없어서 ETC에서 이어서 쓰도록 개정.
//		temp = Q_STRAY_GAIN_U75_C+group_offset;
//
//			if( temp < PARAM_NUM)
//			{
//				data = PRM_PCS[temp].iValue;
//			}
	break;
	case GRP_TRACE_EX://
		data = group_offset;
		if( group_offset >= 0 && group_offset < 800) // 190513 16000->800
		{
			iTraceAutoRstCnt = 0;
			lastPos = group_offset;

			// TRACE 버퍼에서의 데이터 시작 지점 search
			temp = (TRACE.iTail[0/*-iTraceReqLevel*/] + 1);
			if ( temp >= TRACE.iMemorySize )
				temp = temp - TRACE.iMemorySize;


			// 데이터 요청 지점 확인.
			reqPos = temp + (group_offset / 2);
			if ( reqPos >= TRACE.iMemorySize )
				reqPos = reqPos - TRACE.iMemorySize;

			reqPos *= 2;// uint16 * 이므로, float address의 2배가 되었다. 따라서 reqPos *2를 수행.

			if( (group_offset % 2) == 0 ) // IEEE754-float MSB 순서로 보내기 위해.
				data = g_pU16RegTrace[reqPos+1];
			else
				data = g_pU16RegTrace[reqPos];
			lastPosTrcBuf = reqPos;
			trace_req_pos_noTail = group_offset;
			trace_req_pos = reqPos; // for debug

			if( group_offset == 15999 && iTraceReqNum == 5 )
			{
				// Trace Reset
				TRC_TraceReset();
			}
		}
		else
			return -1;

		break;

	default : *rReg=0; return -1;
	}

	*rReg = data;
	return 0;
}		

/*
 * WRITE ==============================================================================
 */



//for Event
String pEventBuf;

int		PARAM_MODBUS_IDX[PARAM_GROUP_MAX]=
{
	2001,		//SYSTEM
	2051,		//INVERTER
	2101,		//GRID
	2191,		//DATE_TIME
	2301,		//DIGITAL INTERFACE
	2351,		//ANALOG INTERFACE
	2401,		//CONTROLLER
	2601,		//TRACE
	2701,		//OFFSET
	2750,		//GAIN
	2801, 		//BESS ~ ETC
};

int dModbusId = 0;
static int GetParamChannelIdx(int ModbusId)
{
	int group;
	int foundgroup = -1;
	unsigned int grp_start_addr;

	ModbusId += 2001;
	dModbusId = ModbusId;

	for(group=PARAM_GROUP_MAX-1; group>=0 ; group--)
	{
		grp_start_addr = PARAM_MODBUS_IDX[group];

		if( ModbusId > 2999 )
			return -1;

		if( ModbusId >= grp_start_addr )
		{
			foundgroup = group;
			break;
		}
	}
	if( foundgroup == -1 )
		return -1;

	return ModbusId - PARAM_MODBUS_IDX[foundgroup] + PARAM_GROUP_INDEX[foundgroup];
}

int getCircIdx(int rMax, int currp, int req)
{
       int ret = 0;

       if( req == 0 )
             return currp;

       if( currp - req < 0 )
       {
             ret = (currp - req) + rMax;
       }
       else
             ret = (currp - req);

       return ret;
#if 0 // 기존 역순환.
   int ret;

   ret = 0;

   if( req > rMax - 1 )
          return 0;

   if (currp + req > rMax-1)
   {
          ret = ((currp + req) - rMax);
   }
   else
   {
          ret = currp + req;
   }

   return ret;
#endif
}


int writeRegData(int group, int group_offset, unsigned int wReg)
{
	int temp;
	//RtcTime time;

	switch(group)
	{
	case GRP_PCS_CTRL		: if( group_offset < CTRL_BUF_MAX ) g_u16RegPCSCtrl[group_offset] = wReg;		break;
	case GRP_ETC			: if( group_offset < ETC_BUF_MAX ) g_u16RegETC[group_offset] = wReg;			break;
	case GRP_PARAMETER		: break;

	default : return -1;
	}

	switch(group)
	{
	case GRP_PARAMETER:
		temp = GetParamChannelIdx(group_offset);
		if( temp != -1)
		{
			// TIME SET
			if( (temp>=TIME_YEAR)&&(temp<=TIME_SECOND) )
			{
				RtcTime time;

				time.Year = PRM_PCS[TIME_YEAR].iValue;
				time.Month = PRM_PCS[TIME_MONTH].iValue;
				time.Date = PRM_PCS[TIME_DAY].iValue;
				time.Hour = PRM_PCS[TIME_HOUR].iValue;
				time.Minute = PRM_PCS[TIME_MINUTE].iValue;
				time.Second = PRM_PCS[TIME_SECOND].iValue;

				switch(temp-TIME_YEAR)
				{
				case 0:
					if( time.Year == wReg )
						return 0;

					time.Year = wReg;
					break;
				case 1:
					if( time.Month == wReg )
						return 0;

					time.Month = wReg;
					break;
				case 2:
					if( time.Date == wReg )
						return 0;

					time.Date = wReg;
					break;
				case 3:
					if( time.Hour == wReg )
						return 0;

					time.Hour = wReg;
					break;
				case 4:
					if( time.Minute == wReg )
						return 0;

					time.Minute = wReg;
					break;
				case 5:
					time.Second = wReg;
					break;
				default:
					break;
				}

				if( RTCTIME_Set(&time) )
					return 0;
				else
					return -1;
			}

			if( temp < PARAM_NUM)
			{
				if(PARA_UpdateParameter(temp, wReg)!=TRUE)
				{
					return -1;
				}
			}
		}
		break;
	case GRP_PCS_CTRL:
	{
		switch( group_offset)
		{
			case 0:
				// PCS Command
				MF_PCS_Command(wReg);
				break;
			case 1: // PCS Command Security number ( 0xA0B0 )
			{
				if( wReg == 0xA0B0)
					bPcsCommandLocked = 0;
			}
				break;
			case 2: // PCS Test Command
				return MF_TestCmd(wReg);
#if 0
			case 3: //by JCNET
			    {

			       DIO_setStatus(wReg,1);
			    }
			    break;
            case 4: //by JCNET
                {

                  DIO_setStatus(wReg,0);
                }
                break;
#endif
			case 5: // 43006 PARAM_COL_VAL
				break;
			case 6: // 43007
				// PLC로 제어되는 GC GI Auto GI모드는 파라미터에 저장하지 않으므로,
				// 시스템이 리부팅 되면 기존에 저장된 파라미터 값이 적용 된다.
				// 43007 Reg로 제어 할 경우에도 저장할지 여부는 추후에 판단.
				if( !CTRL_MODE_Set((int)wReg))
					return -1;

				//if( !PARA_UpdateParameter(GC_IS_MODE, (int)wReg))
				//	return -1;
				break;
			case 7:
				if( !PARA_UpdateParameter(BATT_PEBB_FAULT_MODE, (int)wReg))
					return -1;
				break;
			case 15: // 43016
				if( !PARA_UpdateParameter(BATT_OV_LEVEL, (int)wReg) )
					return -1;
				break;
			case 16: // 43017
				if( !PARA_UpdateParameter(BATT_UV_LEVEL, (int)wReg) )
					return -1;
				break;
			case 17: // 43018
				if( !PARA_UpdateParameter(BATT_OC_LEVEL, (int)wReg) )
					return -1;
				break;
			case 18: // 43019
				if( !PARA_UpdateParameter(INV_OC_LEVEL, (int)wReg) )
					return -1;
				break;
			case 19: // 43020
				if( !PARA_UpdateParameter(GRID_OV_LEVEL1, (int)wReg) )
					return -1;
				break;
			case 20: // 43021
				if( !PARA_UpdateParameter(GRID_UV_LEVEL1, (int)wReg) )
					return -1;
				break;
			case 41: // 43042
				if( !PARA_UpdateParameter(BATT_V_RANGE_MAX, (int)wReg) )
					return -1;
				break;
			case 42: // 43043
				if( !PARA_UpdateParameter(BATT_V_RANGE_MIN, (int)wReg) )
					return -1;
				break;
			case 43: // 43044
				if( !PARA_UpdateParameter(BATT_ORDER_SOURCE, (int)wReg) )
					return -1;
				break;
			case 44: // 43045
				if( !PARA_UpdateParameter(BATT_REF_VAL_MODE, (int)wReg) )
					return -1;
				break;
			case 45: // 43046
				if( !PARA_UpdateParameter(BATT_LOCAL_CONTROL_MODE, (int)wReg) )
					return -1;
				break;
			case 46: // 43047
				if( !PARA_UpdateParameter(BATT_LOCAL_POWER_FLOW, (int)wReg) )
					return -1;
				break;
			case 47: // 43048
				if( !PARA_UpdateParameter(BATT_LOCAL_CONTROL_OPTION, (int)wReg) )
					return -1;
				break;
			case 48: // 43049
				if( !PARA_UpdateParameter(BATT_LOCAL_IVC_V_REF_CHG, (int)wReg) )
					return -1;
				break;
			case 49: // 43050
				if( !PARA_UpdateParameter(BATT_LOCAL_IVC_V_REF_DCHG, (int)wReg) )
					return -1;
				break;
			case 50: // 43051
				if( !PARA_UpdateParameter(BATT_LOCAL_ICC_I_REF_CHG, (int)wReg) )
					return -1;
				break;
			case 51: // 43052
				if( !PARA_UpdateParameter(BATT_LOCAL_ICC_I_REF_DCHG, (int)wReg) )
					return -1;
				break;
			case 52: // 43053
				if( !PARA_UpdateParameter(BATT_LOCAL_PCC_P_REF_CHG, (int)wReg) )
					return -1;
				break;
			case 53: // 43054
				if( !PARA_UpdateParameter(BATT_LOCAL_PCC_P_REF_DCHG, (int)wReg) )
					return -1;
				break;
#if PARA_VERSION > 0x1002
			case 60: PRM_PCS[BATT_POWER_METER_P].iValue = (int)wReg; break;
			case 61: PRM_PCS[BATT_POWER_METER_Q].iValue = (int)wReg; break;
			case 62: PRM_PCS[AUTO_CHARGE_P_REF].iValue = (int)wReg; break;
#endif
			case 64: // 43065
				PRM_PCS[BATT_REMOTE_PCC_P_REF].iValue = (int)wReg;
				break;
			case 65: // 43066
				PRM_PCS[BATT_REMOTE_PCC_Q_REF].iValue = (int)wReg;
				break;
			case 66: // 43067
				if( !PARA_UpdateParameter(BATT_IVC_PCC_CURRENT_LMT_CHG, (int)wReg) )
					return -1;
				break;
			case 67: // 43068
				if( !PARA_UpdateParameter(BATT_IVC_PCC_CURRENT_LMT_DCHG, (int)wReg) )
					return -1;
				break;
			case 68:
				if( !PARA_UpdateParameter(INV_CURRENT_LMT, (int)wReg) )
					return -1;
			case 70:/* PMS의 SOC 상한/하한으로 사용 */ break;
			case 71:/* PMS의 SOC 상한/하한으로 사용 */ break;
			case 72:
#if 0
				time.Year  = PRM_PCS[TIME_YEAR].iValue;
				time.Month = PRM_PCS[TIME_MONTH].iValue;
				time.Date  = PRM_PCS[TIME_DAY].iValue;
				time.Hour  = PRM_PCS[TIME_HOUR].iValue;
				time.Minute = PRM_PCS[TIME_MINUTE].iValue;
				time.Second = PRM_PCS[TIME_SECOND].iValue;

				if( group_offset == 70 )
				{
					time.Year = (wReg>>8) + 2000;
					time.Month = wReg & 0xFF;
				}
				elif( group_offset == 71)
				{
					time.Date = (wReg>>8);
					time.Hour = wReg & 0xFF;
				}
				elif( group_offset == 72)
				{
					time.Minute = (wReg>>8);
					time.Second = wReg & 0xFF;
				}

				if( !RTCTIME_Set(&time) )
					return -1;
#endif
				return 0;
			case 100:
				EVT_GetCacheItem_Modbus(wReg, strEventBuffer); //TODO CHECK
				break;
			case 101:
					// TRACE REQ NUM & Level
					iTraceReqNum = (wReg>>8&0xFF);	//		iTraceReqNum-=1;
					iTraceReqLevel = (wReg&0xFF);

					if( TRACE.iStop == ON )
						iTraceReqLevel = getCircIdx(TRACE_LEVEL, (TRACE.iCurrentLevel -1 <0 ? TRACE_LEVEL-1: TRACE.iCurrentLevel -1), (wReg&0xFF));
					else
						iTraceReqLevel = getCircIdx(TRACE_LEVEL, TRACE.iCurrentLevel, (wReg&0xFF));

					iTraceReqLevel = 0; //+ 180129 현재는 Trace 레벨 0

					if( iTraceReqNum >= 0 && iTraceReqNum <= /*-180111 11*/ 5  )
					{
						if( iTraceReqLevel >= 0 && iTraceReqLevel <= /*-180111 2*/ 0 )
						{
							g_pU16RegTrace = (unsigned int*)TPOOL[ iTraceReqNum ].TRACE_BUF[iTraceReqLevel];
						}
						else
						{
							iTraceReqNum = iTraceReqLevel = 0;
							return -1;
						}
					}
					else
					{
						iTraceReqNum = iTraceReqLevel = 0;
						return -1;
					}
					break;
			default:
				return 0;
		}
	}
		break;
	case GRP_ETC :
#if 0
		switch( group_offset)
		{
		case 0: // 43501
			if(g_u16RegETC[group_offset] == 1)//RS232 protocol : MODBUS RTU ==> SCP
			{
				g_u16RegETC[0] = 0;
			}
			break;
		}
#endif

		// BESS 탭 200개 이상 부터는 맵을 쓸 수 없어서 ETC에서 이어서 쓰도록 개정.
//		temp =  Q_STRAY_GAIN_U75_C + group_offset;
//		if( temp < PARAM_NUM)
//		{
//			if(PARA_UpdateParameter(temp, wReg)!=TRUE)
//			{
//				return -1;
//			}
//		}
		break;
	}
	return 0;
}		

int readReg(unsigned int addr, unsigned int *pReadReg)
{		       
	int reg_group;
	int reg_group_offset;

	//June Added 130611@
	//addr+=40001;
	reg_group = findRegGroup(addr);
	if( reg_group == -1 )
	{
		*pReadReg = 0;
		return -1;
	}

	reg_group_offset = addr - g_uiRegGrpTable[reg_group][0];
	return readRegData(reg_group, reg_group_offset, pReadReg);
}

int writeReg(unsigned int addr, unsigned int Reg)
{
	int reg_group;
	int reg_group_offset;

	//June Added 130611@
	//addr+=40001;
	reg_group = findRegGroup(addr);
	if( reg_group == -1 )
	{
		return -1;
	}

	reg_group_offset = addr - g_uiRegGrpTable[reg_group][0];
	return writeRegData(reg_group, reg_group_offset, Reg);
}

static void initRegs(void)
{
	memset(g_u16RegPCSCtrl,		0, sizeof(g_u16RegPCSCtrl));
	memset(g_u16RegETC,			0, sizeof(g_u16RegETC));

	iTraceReqNum = iTraceReqLevel = 0;
	g_pU16RegTrace = (unsigned int*)(TPOOL[0].TRACE_BUF[0]);
}

void Modbus232_Create(void)
{
	//-modbus_configure(SCI_BAUD, PRM_PCS[DGT_MOD_RTU_ID].iValue, TOTAL_REGS_SIZE);	//slaveID, RegSize
	modbus_configure(SCI_BAUD, 10, TOTAL_REGS_SIZE);	//slaveID, RegSize
	memset(frame, 0, BUFFER_SIZE);

	initRegs();

//	Modbus232_Start();
	Modbus232_Stop();
}

void Modbus232_Stop(void)
{
	m_bModbus232StopFlag = 1;
}
void Modbus232_Start(void)
{
	m_bModbus232StopFlag = 0;
}

// it's in loop
void Modbus232_Process(void)
{
	while(m_bModbus232StopFlag==0)
	{
		Modbus232_DataRefresh();
		modbus_update();

		Task_sleep(10);
	}
}

void Modbus232_DataRefresh()
{

}


unsigned int modbus_update(void)
{
	unsigned char buffer = 0;
	unsigned char overflow = 0;
	char	ch;
	unsigned short	size;

	while (!(CIRC_isEmpty(&circSCI_RXBUF[SCI_RS232])))
	{
		// The maximum number of bytes is limited to the serial buffer size of 128 bytes
		// If more bytes is received than the BUFFER_SIZE the overflow flag will be set and the 
		// serial buffer will be red until all the data is cleared from the receive buffer.
		if (overflow) 
		{
			size=1;
			CIRC_read(SCI_getCircRxBuffer(SCI_RS232), &ch, &size);
		}
		else
		{
			if (buffer == BUFFER_SIZE)
			{
				overflow = 1;
			}
			size=1;
			CIRC_read(SCI_getCircRxBuffer(SCI_RS232), &ch, &size);

			frame[buffer] = ch;
#if 0
#if DEBUG_MODE == DEBUG_DEVELOP
			if(buffer==0)
				System_printf("\n\nRX: ", frame[buffer]);
			System_printf("%02x ", frame[buffer]);
#endif
#endif
			buffer++;
		}
		Task_sleep(T1_5); // 750 us.

	}
#if DEBUG_MODE == DEBUG_DEVELOP
		System_flush();
#endif
	// If an overflow occurred increment the errorCount
	// variable and return to the main sketch without 
	// responding to the request i.e. force a timeout
	if (overflow)
	{
		//System_printf("overflow\n");
		//System_flush();
		return errorCount++;
	}
	
	// The minimum request packet is 8 bytes for function 3 & 16
	if (buffer > 7) 
	{
		unsigned char id = frame[0];

		broadcastFlag = 0;

		if (id == 0)
		{
			broadcastFlag = 1;
		}
		
		if (id == slaveID || broadcastFlag) // if the received ID matches the slaveID or broadcasting id (0), continue
		{
			unsigned int crc = ((frame[buffer - 2] << 8) | frame[buffer - 1]); // combine the crc Low & High bytes
			if (calculateCRC(buffer - 2) == crc) // if the calculated crc matches the received crc continue
			{
				function = frame[1];
				unsigned int startingAddress = ((frame[2] << 8) | frame[3]); // combine the starting address bytes
				unsigned int no_of_registers = ((frame[4] << 8) | frame[5]); // combine the number of register bytes
				unsigned int maxData = startingAddress + no_of_registers - 1;
				unsigned char index;
				unsigned char address;
				unsigned int crc16;
				
				// broadcasting is not supported for function 3
				if (!broadcastFlag && (function == 3))	//0x03 Read
				{
					if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
					{
						if (maxData /*-180110 +40000*/ <= holdingRegsSize && (maxData-startingAddress) <= NUMOFREG_MAX ) // check exception 3 ILLEGAL DATA VALUE
						{
							unsigned char noOfBytes = no_of_registers * 2;
							unsigned char responseFrameSize = 5 + noOfBytes; // ID, function, noOfBytes, (dataLo + dataHi) * number of registers, crcLo, crcHi
							frame[0] = slaveID;
							frame[1] = function;
							frame[2] = noOfBytes;
							address = 3; // PDU starts at the 4th byte
							unsigned int temp;

//							for (index = startingAddress; index < maxData; index++)
							for (index = startingAddress; index <= maxData; index++)
							{
								//temp = holdingRegs[index];
								if( readReg(index, &temp) == -1 )
								{
									exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS - june added
									return errorCount;
								}

								frame[address] = temp >> 8; // split the register into 2 bytes
								address++;
								frame[address] = temp & 0xFF;
								address++;
							}	
							
							crc16 = calculateCRC(responseFrameSize - 2);
							frame[responseFrameSize - 2] = crc16 >> 8; // split crc into 2 bytes
							frame[responseFrameSize - 1] = crc16 & 0xFF;
							sendPacket(responseFrameSize);
						}
						else	
						{
							exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
						}
					}
					else
					{
						exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
					}
				}
				//----------------------------------------------------------------------------------------------------------------
				else if (function == 6)//0x6 Write S Register
				{
					if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
					{
							if( writeReg(startingAddress, no_of_registers ) == -1 )
							{
								exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
								return errorCount;
							}

							// only the first 6 bytes are used for CRC calculation
							crc16 = calculateCRC(6);
							frame[6] = crc16 >> 8; // split crc into 2 bytes
							frame[7] = crc16 & 0xFF;

							// a function 16 response is an echo of the first 6 bytes from the request + 2 crc bytes
							if (!broadcastFlag) // don't respond if it's a broadcast message
							{
								sendPacket(8);
							}

					}
					else
					{
						exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
					}
				}
				//----------------------------------------------------------------------------------------------------------------
				else if (function == 16)//0x10 Write M Register
				{
					// check if the received number of bytes matches the calculated bytes minus the request bytes
					// id + function + (2 * address bytes) + (2 * no of register bytes) + byte count + (2 * CRC bytes) = 9 bytes
					if (frame[6] == (buffer - 9))
					{
						if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
						{
							if (maxData <= holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
							{
								address = 7; // start at the 8th byte in the frame

//								for (index = startingAddress; index < maxData; index++)
								for (index = startingAddress; index <= maxData; index++)
								{
									if( writeReg(index, (frame[address] << 8) | (frame[address + 1]) ) == -1 )
									{
										exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
										return errorCount;
									}
									address += 2;
								}

								// only the first 6 bytes are used for CRC calculation
								crc16 = calculateCRC(6);
								frame[6] = crc16 >> 8; // split crc into 2 bytes
								frame[7] = crc16 & 0xFF;

								// a function 16 response is an echo of the first 6 bytes from the request + 2 crc bytes
								if (!broadcastFlag) // don't respond if it's a broadcast message
								{
									sendPacket(8);
								}
							}
							else
							{
								exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
							}
						}
						else
						{
							exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
						}
					}
					else
					{
						errorCount++; // corrupted packet
					}
				}
				//----------------------------------------------------------------------------------------------------------------
				else
				{
					exceptionResponse(1); // exception 1 ILLEGAL FUNCTION
				}
			}
			else // checksum failed
			{
				errorCount++;
			}
		} // incorrect id	//if (id == slaveID || broadcastFlag)
	}// if(buffer > 7)
	else if (buffer > 0 && buffer < 8)
	{
		//System_printf("\nbuffer > 0 && buffer < 8\n");
		//System_flush();
		errorCount++; // corrupted packet
	}
		
	return errorCount;
}				

void exceptionResponse(unsigned char exception)
{
	errorCount++; // each call to exceptionResponse() will increment the errorCount
	if (!broadcastFlag) // don't respond if its a broadcast message
	{
		frame[0] = slaveID;
		frame[1] = (function | 0x80); // set the MSB bit high, informs the master of an exception
		frame[2] = exception;
		unsigned int crc16 = calculateCRC(3); // ID, function + 0x80, exception code == 3 bytes
		frame[3] = crc16 >> 8;
		frame[4] = crc16 & 0xFF;
		sendPacket(5); // exception response is always 5 bytes ID, function + 0x80, exception code, 2 bytes crc
	}
}

//void modbus_configure(long baud, unsigned char _slaveID, unsigned char _TxEnablePin, unsigned int _holdingRegsSize)
void modbus_configure(int _baud, unsigned char _slaveID, unsigned int _holdingRegsSize)
{
	long baud;

	//slaveID = _slaveID;
	Modbus232_SetId(_slaveID);
	//SCI_create(SCI_RS232, SCI_BAUD,0);
  
	switch(_baud)
	{
	case SCI_BAUD_3 : 	baud = 9600;	break;
	case SCI_BAUD_4 : 	baud = 19200;	break;
	case SCI_BAUD_5 : 	baud = 38400;	break;
	case SCI_BAUD_6 : 	baud = 57600;	break;
	case SCI_BAUD_7 : 	baud = 115200;	break;
	default : 			baud = 115200;	break;
	}
	
	// Modbus states that a baud rate higher than 19200 must use a fixed 750 us 
  // for inter character time out and 1.75 ms for a frame delay.
  // For baud rates below 19200 the timeing is more critical and has to be calculated.
  // E.g. 9600 baud in a 10 bit packet is 960 characters per second
  // In milliseconds this will be 960characters per 1000ms. So for 1 character
  // 1000ms/960characters is 1.04167ms per character and finaly modbus states an
  // intercharacter must be 1.5T or 1.5 times longer than a normal character and thus
  // 1.5T = 1.04167ms * 1.5 = 1.5625ms. A frame delay is 3.5T.
	
	if (baud > 19200)
	{
		T1_5 = 8;// change for task_sleep ori: 750us;
		T3_5 = 18;//1750us;
	}
	else 
	{
		T1_5 = (15000000/baud)  ;
		T3_5 = 35000000/baud; // 1T * 3.5 = T3.5
	}
	
	holdingRegsSize = _holdingRegsSize;
	errorCount = 0; // initialize errorCount
}   

unsigned int calculateCRC(unsigned char bufferSize)
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  unsigned char i,j;

  for (i = 0; i < bufferSize; i++)
  {
    temp = temp ^ frame[i];
    for (j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order. 
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  return temp; // the returned value is already swopped - crcLo byte is first & crcHi byte is last
}

void sendPacket(unsigned char bufferSize)
{
	unsigned char i, ch;
#if 0
#if DEBUG_MODE == DEBUG_DEVELOP
System_printf("\nTX: ");
System_flush();
#endif
#endif

	for(i = 0; i < bufferSize; i++)
	{
		ch = frame[i];
		SCI_write(SCI_RS232, ch);
#if 0
#if DEBUG_MODE == DEBUG_DEVELOP
System_printf("%02x ", ch);
#endif
#endif
	}
#if 0
#if DEBUG_MODE == DEBUG_DEVELOP
System_flush();
#endif
#endif
	SendFlush();

	// allow a frame delay to indicate end of transmission
	Task_sleep(T3_5);
}
