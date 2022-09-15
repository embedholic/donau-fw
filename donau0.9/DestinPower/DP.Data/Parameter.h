/*
 * Parameter.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef PARAMETER_H_
#define PARAMETER_H_

//+June
#include "LGS_Common.h"

typedef enum
{
	PARA_VER_FAILURE,
	PARA_VER_HIGH,
	PARA_VER_LOW,
	PARA_VER_EQUAL,
	DIFF_MODEL
} ParaVerInfoState;

typedef struct {
	Int16 iValue; // TODO Check 65535 이상 값이 파라미터로 사용될 가능성이 있으므로 Uint32으로 변경
				  // NvSRAM에 저장되는 것은 모두 16비트 이므로, 다시 Int16으로 수정.
    float fIncDec;
    Bool bChange;
    Bool bImpossibleRunning;
} Parameter;

typedef struct {
	Int16 iValue;
    float fIncDec;
    Bool bChange;
    Bool bImpossibleRunning;
	//-Uns uLastModifiedVer;
} Parameter_Table_Item;

typedef struct {
	int index;
	int iValue;
} ParameterSpare;

typedef struct _ParaNVSRAM
{
	Uns	uLSB;
	Uns	uMSB;
} ParaNVSRAM;

/*
typedef struct
{
	void *pAddr;
	Uns uSize;
} ParaArgument;
*/
#define	PARA_CHECKSUM_WRITE_MODE	0
#define	PARA_CHECKSUM_VERIFY_MODE	1



Bool PARA_CalcChecksum(Bool Mode);
void PARA_Create(void);
Uns PARA_VersionErrorStatus(void);
void PARA_Initialize(void);
//void PARA_StoreToMemory(Uns uID, Parameter *pPara, void *pParaAux);
Bool PARA_UpdateParameter( Uns ParaID, int Val );
void PARA_TableToIRAM(Uns uID, Bool isParamInit);
Bool PARA_StoreToMemoryForTask(Uns uID, Int16 value);

#endif /* PARAMETER_H_ */
