#include "SagEventHistory.h"

#include "CC.h"
#include "SYSTEM.h"
#include "Prm_pcs.h"
#include "Odt.h"
#include "RtcTime.h"
#include "CTRL_INV_SEAMLESS.h"
#include "CTRL_BYP_EVT_OPERATION.h"
#include "trace.h"


#define SAG_QUEUE_BUF_CNT 20

typedef struct CIRC_Obj {
    Uns         writeIndex;     /* write pointer for the buffer */
    Uns         readIndex;      /* read pointer fro the buffer */
    Uns         charCount;      /* buffer character count */
    Uns         size;
    SagEvt* buf;
} CIRCH_Obj, *CIRCH_Handle;


void Sag_Evt_History_Create(void);
void Sag_Evt_History_Proceed(void);
private Bool Sag_enque(SagEvt sagData );
private Bool Sag_deque(SagEvt* pSagData);


private CIRCH_Obj circSag;
private SagEvt array_sag_buf[SAG_QUEUE_BUF_CNT]; /* Sag Data Queue */
SagOperation SAG_OP;


enum {
	SAG_DATA_TRANS_SEND				,
	SAG_DATA_TRANS_UPDATE
};

#pragma CODE_SECTION (Sag_Evt_History_Create, "dp_ctrl")
void Sag_Evt_History_Create()
{
	memset(&array_sag_buf, 0, sizeof(array_sag_buf));
	memset(&SAG_OP, 0, sizeof(SAG_OP));

	INVERTER.odtSagCompTime = Odt_(&INVERTER.ODT_SAG_COMP_TIME, 30000, SAG_STATE_CALL_PERIOD);

	SAG_OP.Sag_uStatus = SAG_DATA_SEND;

	circSag.writeIndex = 0;
	circSag.readIndex = 0;
	circSag.charCount = 0;
	circSag.size = SAG_QUEUE_BUF_CNT;
	circSag.buf= array_sag_buf;
}

#pragma CODE_SECTION (Sag_trans, "dp_ctrl")
void Sag_trans(Uns tran)
{
	switch (tran)
	{
	case SAG_DATA_TRANS_SEND :

		SAG_OP.Sag_uStatus = SAG_DATA_SEND;
		break;

	case SAG_DATA_TRANS_UPDATE :

		SAG_OP.Sag_data_Instant.YearMonth = ((PRM_PCS[TIME_YEAR].iValue - 2000)<<8&(0xFF00) | (PRM_PCS[TIME_MONTH].iValue)&(0xFF));
		SAG_OP.Sag_data_Instant.DateHour  = ((PRM_PCS[TIME_DAY].iValue)<<8&(0xFF00) | (PRM_PCS[TIME_HOUR].iValue)&(0xFF));
		SAG_OP.Sag_data_Instant.MinSec    = ((PRM_PCS[TIME_MINUTE].iValue)<<8&(0xFF00) | (PRM_PCS[TIME_SECOND].iValue)&(0xFF));
		SAG_OP.Sag_uStatus = SAG_DATA_UPDATE;
		ODT_Initialize(INVERTER.odtSagCompTime);
		break;
	}
}

#pragma CODE_SECTION (Sag_Evt_History_Proceed, "dp_ctrl")
void Sag_Evt_History_Proceed(void)
{
	switch(SAG_OP.Sag_uStatus)
	{
	case SAG_DATA_SEND :

		if( !SAG_OP.Sag_data_ready_flag )
		{
			if(Sag_deque(&SAG_OP.Sag_data_Shadow))
			{
				SAG_OP.Sag_data_ready_flag = TRUE;
			}
		}

		if(INVERTER.uStatus == SYS_INV_ISLANDING || INVERTER.uStatus == SYS_INV_RE_SYNC)
			Sag_trans(SAG_DATA_TRANS_UPDATE);
		break;

	case SAG_DATA_UPDATE :

		if(INVERTER.uStatus == SYS_INV_ISLANDING)
		{
			SAG_OP.Sag_data_Instant.CompVoltR = GRID_ARG.MEASURE[0].fPhaseVoltage / ACP.PCC.RATE.Vph * 100.;
			SAG_OP.Sag_data_Instant.CompVoltS = GRID_ARG.MEASURE[1].fPhaseVoltage / ACP.PCC.RATE.Vph * 100.;
			SAG_OP.Sag_data_Instant.CompVoltT = GRID_ARG.MEASURE[2].fPhaseVoltage / ACP.PCC.RATE.Vph * 100.;
			if( SAG_OP.Sag_time_odt_flag )
				ODT_Update(INVERTER.odtSagCompTime, TRUE);
		}
		else if(INVERTER.uStatus != SYS_INV_RE_SYNC)
		{
			if(INVERTER.uStatus == SYS_INV_RUN /*&& SAG_OP.Sag_data_update_flag*/)
				SAG_OP.Sag_data_Instant.bRestore = TRUE; /* 보상 성공 */
			else
				SAG_OP.Sag_data_Instant.bRestore = FALSE;

			if(INVERTER.odtSagCompTime->uCount == 0)
				SAG_OP.Sag_data_Instant.CompT = 0;
			else if(INVERTER.odtSagCompTime->uCount < (PARAM_VAL(CTRL_COSPHI_P_10) + 1))
				SAG_OP.Sag_data_Instant.CompT = 2; /* GC->GI trans cnt 10: 2ms */
			else
				SAG_OP.Sag_data_Instant.CompT = (INVERTER.odtSagCompTime->uCount - PARAM_VAL(CTRL_COSPHI_P_10)) * SAG_STATE_CALL_PERIOD;

			/* TODO Trace Hook */

			if( !Sag_enque(SAG_OP.Sag_data_Instant) )
			{
				// Warn: Queue Full
				// TODO History?
			}
			Sag_trans(SAG_DATA_TRANS_SEND);
		}
		break;
	}


	/* 30초 넘으면 30초 고정
	   if(INVERTER.odtSagCompTime->bLiveFlag == ODT_FINISH )
		ODT_Initialize(INVERTER.odtSagCompTime);
	*/
}

private Bool Sag_enque(SagEvt sagData )
{
	if(circSag.charCount==circSag.size)
	{
		return FALSE;
	}

	// warn: memcpy 이상 시 낮은 복사 수행할 것.
	memcpy(&circSag.buf[circSag.writeIndex], &sagData, sizeof(sagData));
	//circSag.buf[circHistory.writeIndex].CompT = sagData.CompT;

	if( circSag.writeIndex + 1 > circSag.size - 1)
		circSag.writeIndex = 0;
	else
		circSag.writeIndex += 1;
	(circSag.charCount)++;
	return TRUE;
}
private Bool Sag_deque(SagEvt* pSagData)
{
	if( circSag.charCount <= 0)
		return FALSE;

	*pSagData = (circSag.buf[circSag.readIndex]);

	if( circSag.readIndex + 1 > circSag.size - 1)
		circSag.readIndex = 0;
	else
		circSag.readIndex += 1;

	(circSag.charCount)--;

	return TRUE;
}
