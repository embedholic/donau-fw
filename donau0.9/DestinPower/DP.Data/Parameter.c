/*
 * Parameter.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#include <string.h>
#include "PARAMETER.h"
#include "FAULT.h"
#include "trace.h"
//-#include "ds1743hw.h"
#include "System.h"

#include "prm_pcs.h"
#include "NvSRAM.h"
#include "DSP28x_Project.h"
#include "Version.h"
#include "Util.h"
/*
PARAMETER VERSION

XXXX :

�� : Not Defined.
*/

typedef enum
{
	LSB=0,
	MSB=1
}BYTE_REGION;

static Uns uParaVer = 0;
Uns uParaVersionState = 0;

static void OriginTableCreate(void);
static void WriteParaVersion(void);
void PARA_StoreToMemory(Uns uID);

/*
 * Call : main()-> MSI_Create() -> PARA_Create() ->
 */
// NVSRAM�� �ִ� �Ķ���� ������ �о� IRAM�� �����Ѵ�.
static void SetParameterValueToIRAM()
{
	UInt16 uL, uM, idx;
	Int32 iVal;
	//-Uns uVal;

	for ( idx = 0; idx < PARAM_NUM; idx++ )
	{

		ReadPRM_PCS(idx*2	,&uL);
		ReadPRM_PCS(idx*2+1	,&uM);
		uL = uL & 0xFF;
		uM = (uM<<8);

		// ������ �����ؾ� ��ȣ�� ���� �ȴ�.
		iVal =  ((Uint32)uM+(Uint32)uL)<<16;
		iVal =  iVal>>16;
		PRM_PCS[idx].iValue = iVal;
	}
}

// ���� �Ķ���� ���̺� �ִ� ������ NVSRAM�� ���� ��� ���̺� ����.
/*
 * Call: CopyDuplicate(), PARA_Initialize
 */
static void OriginToParameter(Bool isInit)
{
	int idx;

	for (idx=0; idx < PARAM_NUM; idx++)
	{
		PARA_TableToIRAM(idx,isInit);
		PARA_StoreToMemory(idx);
	}
}

/*
 * Call: PARA_Create(), PARA_Initialize()
 */
static void ParaVesionCheck(void)
{
	/*-FIXED for XMC4500 or delete*/
	UInt16 uProgVer, uL, uM, by1, by2, by3, by4;
	Bool bParaVerGood;

	// �Ķ���� ���� �˻�
	ReadParaVersionFromNvSram(0, &uL);
	ReadParaVersionFromNvSram(1, &uM);
	uM = uM << 8;
	uParaVer = (uM + uL);
	uProgVer = PARA_VERSION;


	by1 = uL & 0xF;
	by2 = (uL & 0xF0)>>4;
	by3 = uM & 0xF;
	by4 = (uM & 0xF0)>>4;
	if ( by1 <= 9 && by2 <= 9 && by3 <= 9 && by4 <= 9 )
		bParaVerGood = TRUE;
	else
		bParaVerGood = FALSE;

	if ( !bParaVerGood )
	{
		uParaVersionState = PARA_VER_FAILURE;
		return;
	}

	// Version Check
	if (uParaVer < PARA_VERSION)
	{
		uParaVersionState = PARA_VER_LOW;
	}
	else if ( (uParaVer & 0xF000) != (uProgVer & 0xF000) )
	{
		uParaVersionState = DIFF_MODEL;
	}
	else
	{
		if ( uParaVer == (uProgVer) ) 	//140305 Bug Fixed
			uParaVersionState = PARA_VER_EQUAL;
		else if ( uParaVer > (uProgVer) ) //140305 Bug Fixed
			uParaVersionState = PARA_VER_HIGH;
		else
			uParaVersionState = PARA_VER_LOW;
	}

	//140305 Delete@uParaVer = (uM + uL) & 0x0FFF;

}

/*******************************************************************/
/* PARA_Create - Parameter Block ����                              */
/* Parameters : void                                               */
/* Returns : void                                                  */
/*******************************************************************/
void PARA_Create(void)
{
	//-int idx;

	// �Ķ���� �ʱ�ȭ�� �̿�� Table ����
	OriginTableCreate();
	return; // by JCNET.. �׽�Ʈ������ �Ķ���ʹ� PRM_PCS_TABLE ������ ����ϵ���. nvram �� �Ķ���� ���������� �ʰ� �о������ ����..
	ParaVesionCheck();

	switch(uParaVersionState)
	{
		case PARA_VER_FAILURE:
			FLT_Raise(FLTH_PARA_VERSION);
			PARA_Initialize();
#if DEBUG_MODE == NORMAL
			//-RebootSystem();// �Ķ���� �ʱ�ȭ �ȵ� �� ��� ����� �� �� ����
#endif
			break;
		case PARA_VER_HIGH:
		case PARA_VER_LOW:
			FLT_Raise(FLTH_PARA_VERSION);
			SetParameterValueToIRAM(); //+
			//PARA_Initialize();
			//RebootSystem();
			break;
		case PARA_VER_EQUAL:
			// NvSRAM���� IRAM�� ������ ����
 // ���� ������..           PARA_Initialize(); // JCNET !! for test..
			SetParameterValueToIRAM();
 			break;
		case DIFF_MODEL :
			FLT_Raise(FLTH_PARA_VERSION);
			PARA_Initialize();
#if DEBUG_MODE == NORMAL
			//-RebootSystem();// �Ķ���� �ʱ�ȭ �ȵ� �� ��� ����� �� �� ����-> J-tag ���� ���·� �ٸ� �� ���� �ø��� ���� �߻�.
#endif
			break;

	}
}

/*
 * Call: OriginToParameter(), PARA_Create(), PARA_TableToIRAM()
 */
void PARA_TableToIRAM(Uns uID, Bool isParamInit)
{
	//PRM_PCS��
	PRM_PCS[uID].iValue = PRM_PCS_TABLE[uID].iValue;
	PRM_PCS[uID].fIncDec = PRM_PCS_TABLE[uID].fIncDec;
	//-if( isParamInit )
		//-PRM_PCS[uID].bChange  = TRUE;//�߰� ��.������ �ý��� ������ ��ȭ��Ű�� ���Ͽ�. => ��� Task�� �װ� ��.
	//-else
		PRM_PCS[uID].bChange  = PRM_PCS_TABLE[uID].bChange;
	PRM_PCS[uID].bImpossibleRunning = PRM_PCS_TABLE[uID].bImpossibleRunning;
}


/*******************************************************************/
/* PARA_StoreToMemory - ���ο� �Ķ������ ���� NVSRAM���� ����     */
/* Parameters : Uns ParaID : ����� �Ķ������ ID                  */
/*     Parameter *pPara : ����� �Ķ������ ������                 */
/* Returns : void                                                  */
/*******************************************************************/
void PARA_StoreToMemory(Uns uID)
{
	Uns uL, uM;

	// �Ķ���� ������ �幰�� �Ͼ���� ���ͷ�Ʈ�� ��Ȱ����ų �ʿ䰡 ����.
	uL = PRM_PCS[uID].iValue & 0xFF;
	uM = PRM_PCS[uID].iValue >> 8 & 0xFF;

	WritePRM_PCS(uID*2, uL);
	WritePRM_PCS(uID*2+1, uM);
	PARA_CalcChecksum(PARA_CHECKSUM_WRITE_MODE);
}

/*******************************************************************/
/* PARA_StoreToMemory - ���ο� �Ķ������ ���� NVSRAM���� ����     */
/* Parameters : Uns ParaID : ����� �Ķ������ ID                  */
/*     Parameter *pPara : ����� �Ķ������ ������                 */
/*     int Val : ����� �Ķ������ ��                              */
/* Returns : void                                                  */
/*******************************************************************/
Bool PARA_StoreToMemoryForTask(Uns uID, Int16 value)
{
	Uns uL, uM;
	//-ParaNVSRAM *pNV;

	// �Ķ���� ������ �幰�� �Ͼ���� ���ͷ�Ʈ�� ��Ȱ����ų �ʿ䰡 ����.
	uL = value & 0xFF;
	uM = value>>8 & 0xFF;
	//-pNV = nvstcPRM_PCS_NV;
	/*-
	pNV[uID].uLSB = uL;
	pNV[uID].uMSB = uM;
	*/
	if( !WritePRM_PCS(uID*2, uL) )
		return FALSE;

	if( !WritePRM_PCS(uID*2+1, uM) )
		return FALSE;

	if( !PARA_CalcChecksum(PARA_CHECKSUM_WRITE_MODE) )
		return FALSE;

	return TRUE;
}

/*******************************************************************/
/* PARA_IsVersionErrorStatus - �� ���°� Version Error ���� Ȯ��   */
/* Parameters : void                                               */
/* Returns : Bool  T ==> Version Error, F ==> Normal               */
/*******************************************************************/
Uns PARA_VersionErrorStatus(void)
{
	return uParaVersionState;
}

static void WriteParaVersion(void)
{
	WriteParaVersionToNvSram(LSB,PARA_VERSION & 0xFF);
	WriteParaVersionToNvSram(MSB,PARA_VERSION>>8);
}

/*******************************************************************/
/* PARA_Initialize - �Ķ���ͺ� �ʱ�ȭ                           */
/* Parameters :  void                                              */
/* Returns : void                                                  */
/*******************************************************************/
void PARA_Initialize(void)
{
	OriginToParameter(TRUE);
	WriteParaVersion();
	ParaVesionCheck();
	if (uParaVersionState != PARA_VER_FAILURE)
		PARA_CalcChecksum(PARA_CHECKSUM_WRITE_MODE);
	else
	{
#if DEBUG_MODE == DEBUG_DEVELOP
		error();
#endif
	}

	// �ý����� �� ���� �Ǿ�� ��� ���� �ùٸ��� ����ȴ�.
	// SCP���� ȣ�� �Ǵµ�, �Ķ���� �ʱ�ȭ �� ����� ��Ų��.
}

// ��� �Ķ���� ���̺��� ������ PRM_PCS�� ����
static void OriginTableCreate(void)
{
	int idx;
	Uns uSize;
	//-Uns uL, uM;

	uSize = PARAM_NUM;

	for (idx=0; idx<uSize; idx++)
	{
		PRM_PCS[idx].iValue = PRM_PCS_TABLE[idx].iValue;
		PRM_PCS[idx].fIncDec = PRM_PCS_TABLE[idx].fIncDec;
		PRM_PCS[idx].bChange = PRM_PCS_TABLE[idx].bChange;
		PRM_PCS[idx].bImpossibleRunning = PRM_PCS_TABLE[idx].bImpossibleRunning;
	}
}

Bool PARA_CalcChecksum(Bool Mode)
{
	UInt16	Checksum,Count,LSB,MSB,LSB1,MSB1;

	Checksum=0;
	for(Count=0;Count<PARAM_NUM;Count++)
	{
		ReadPRM_PCS(Count*2,&LSB);
		ReadPRM_PCS(Count*2+1,&MSB);
		LSB = LSB&0xFF;
		MSB = MSB&0xFF;
		Checksum = (Checksum)^LSB;
		Checksum = (Checksum)^MSB;
	}


	if(Mode==PARA_CHECKSUM_WRITE_MODE)//Write New Checksum
	{
		if( !WriteParaCheckSumToNvSram(0, (Checksum)& 0xFF) )
			return FALSE;
		if( !WriteParaCheckSumToNvSram(1, (Checksum)>>8) )
			return FALSE;

		return TRUE;
	}
	else if(Mode==PARA_CHECKSUM_VERIFY_MODE)//Verify Checksum
	{
		ReadParaCheckSumFromNvSram(0, &LSB);
		ReadParaCheckSumFromNvSram(1, &MSB);

		LSB1=Checksum& 0xFF;
		MSB1=(Checksum>>8)& 0xFF;

		if( (LSB==LSB1)&&(MSB==MSB1) )
			return TRUE;
		else
			return FALSE;

	}
	return FALSE;
}

/*
*********************************************************************************************************
*                                         Update Parameters
*
* Description: Update Parameters
*
* Arguments  : ParaID  is :
*
*              pPara   is :
*
*              Val     is :
*
*              pAux    is :
*
* Returns    : none
*********************************************************************************************************
*/
Bool PARA_UpdateParameter( Uns ParaID, int Val )
{
	//-int iTemp;
	float fTemp;

	if ( ParaID < PARAM_NUM )
	{
		//+150520 June. System memory crash ����.
		if( ParaID ==  GRID_RATED_FREQ)
		{
			// CHECK. GRID_RATED_FREQ�� Varient�� 1�� �����.
			if( Val < 45 )
				Val = 45;
			if( Val > 65 )
				Val = 65;
		}


		if ( PRM_PCS[ParaID].bImpossibleRunning )
		{
			if ( INVERTER.uStatus == SYS_INV_FAULT || INVERTER.uStatus == SYS_INV_STOP || INVERTER.uStatus == SYS_INV_DC_CHARGE
				|| INVERTER.uStatus == SYS_INV_TEST_MODE )
			{
				/*
				 * �Ķ���� �� ���� ���� NvSRAM�� ������ ������ ���� �ϵ��� �Ѵ�.
				 * �Ķ���� ���� ���� �� �� NvSRAM�� �����͸� �����Ϸ��� ���� CC ���ͷ�Ʈ�� �ɸ� �� Fault������ �߻��Ͽ�
				 * Event�� �����Ϸ��� ���� ���� �������� �ý����� ���� �ȴ�.
				 * �� ������ DBUG��忡�� �ù����̼� �� ��� Over Current�� �߻���Ű�� Ȯ�� �����ϴ�.
				 * ����, NvSRAM�� �����͸� ���� ���� �� �Ķ���� ������ ���� ����ǵ��� �ϸ� �ý����� ���׵��� �ʴ´�.
				 * �׷���, �Ķ���� ������ �ȵ�. ���߿� �ذ�! -> PARA_StoreToMemoryForTask�� �ذ�.
				 */

				//@Added Todo Check
				if( PRM_PCS[ParaID].iValue == Val)
				{
					return TRUE;
				}

				if( PARA_StoreToMemoryForTask(ParaID, Val) == FALSE)
					return FALSE;

				PRM_PCS[ParaID].iValue = Val;
				PRM_PCS[ParaID].bChange = TRUE;

				return TRUE;
			}
			else
				return FALSE;
		}
		else
		{
			if (
				ParaID == BATT_LOCAL_POWER_FLOW   ||
				ParaID == BATT_REMOTE_POWER_FLOW  ||
				ParaID == BATT_REMOTE_PCC_P_REF   ||
				ParaID == BATT_REMOTE_PCC_Q_REF
			)
			{

				PRM_PCS[ParaID].iValue = Val;
				PRM_PCS[ParaID].bChange = TRUE;

			}
			else
			{
				if( ParaID == BATT_LOCAL_ICC_I_REF_CHG || ParaID == BATT_LOCAL_ICC_I_REF_DCHG)
				{
					fTemp = ((float)PARAM_VAL(INV_CAPACITY)*1000.) / (float)PARAM_VAL(BATT_V_RANGE_MAX); // ����

					if( fTemp < (Val * PARAM_COL_VAL(ParaID)) )
					{
						//TODO TEST �Ŀ��� RESET �Ŀ��� MAX ���� ����� ����Ǿ����� Ȯ�� �ؾ� ��
						Val = fTemp / PARAM_COL_VAL(ParaID);
					}
				}

				//@Added Todo Check
				if( PRM_PCS[ParaID].iValue == Val)
				{
					return TRUE;
				}

				if( PARA_StoreToMemoryForTask(ParaID, Val) == FALSE)
						return FALSE;

				PRM_PCS[ParaID].iValue = Val;
				//-PARA_StoreToMemory(ParaID);
				PRM_PCS[ParaID].bChange = TRUE;

			}
			return TRUE;
		}
	}
	else
		return FALSE;
}



