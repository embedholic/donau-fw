/*
 * DIO.h
 *
 *  Created on: 2013. 1. 23.
 *      Author: destinPower
 */

#ifndef DIO_H_
#define DIO_H_

#include "LGS_Common.h"

typedef enum
{
	dio_none=0,
	dio_command_succ,
	//-dio_device_not_busy,
	dio_device_busy,
	dio_device_error,
	dio_unknown_port,
	dio_unknown_device

}DIO_CTRL_STATUS;

typedef union
{
	int all;
	struct
	{
		//LSB - DI와 DO는 역순이다. <-...??
		Uint16 PORT1:1; // A Logos:CB10	Ethos:CB10					B Logos:CB2
		Uint16 PORT2:1; // A Logos:door	Ethos:CB20					B Logos:CB3
		Uint16 PORT3:1; // A Logos:ACSMPS	Ethos:MC10				B Logos:MC3
		Uint16 PORT4:1; // A Logos:DCSMPS	Ethos:MC1A				B Logos:MC4
		Uint16 PORT5:1; // A Logos:EPO EmergencyStop<<비접점식(상태반대)>>			B Logos:MC5
		Uint16 PORT6:1; // B Logos:MC6
		Uint16 PORT7:1;	// B Logos:Door
		Uint16 PORT8:1; // B Logos:TR Temp<<비접점식(상태반대)>>
		Uint16 REV:8;
	}bit;
}DI_STATE;
#if 0
typedef union
{
	int all;
	struct
	{
		//LSB - DI와 DO는 역순이다.
		Uint16 PORT1:1;
		Uint16 PORT2:1;
		Uint16 PORT3:1;
		Uint16 PORT4:1;
		Uint16 PORT5:1;
		Uint16 PORT6:1;
		Uint16 PORT7:1;
		Uint16 PORT8:1;
		Uint16 REV:8;
	}bit;
}DO_CTRL;
#endif
typedef enum {
	DO_ISO_STX=0,
	DO_ISO_A=0,
	DO_ISO_B=1,
	DO_ISO_END /*bug fixed!!= DO_ISO_B*/
} DO_CHIP;

typedef enum {
	 DI_ISO_STX=0,
	 DI_ISO_A=0,
	 DI_ISO_B=1,
	 DI_ISO_C=2,
	 DI_ISO_END
} DI_CHIP;
typedef enum
{
	DI_STX=1,
	DI_GROUP_A=1,
	DI_1=1,	DI_2,	DI_3,	DI_4,	DI_5,	DI_6,	DI_7,	DI_8, //DDI
	DI_GROUP_B=9,
	DI_9=9,	DI_10,	DI_11,	DI_12,	DI_13,	DI_14,	DI_15,	DI_16, //ADI
	DI_GROUP_C=17,
	DI_17=17,	DI_18,	DI_19,	DI_20,	DI_21,	DI_22,	DI_23,	DI_24,
	DI_END=24
}DI_LIST;

typedef enum
{
	//XXX Digital Out Chip 1의 6번까지만 DDO로 사용하고, 7,8 번과 Chip 2 번을 ADO로 사용한다.
	DO_STX=1,
	DO_GROUP_A=1, //Chip 1 DDO, ADO
	DO_1=1,	DO_2,	DO_3,	DO_4,	DO_5,	DO_6,	/*7부터 ADO로 사용*/DO_7,	DO_8,
	DO_GROUP_B=9, //Chip 2 ADO
	DO_9=9,	DO_10,	DO_11,	DO_12,	DO_13,	DO_14,	DO_15,	DO_16,
	DO_END=16
}DO_LIST;


public void DIO_Create();
public DIO_CTRL_STATUS DIO_getStatus(DI_LIST diID, DI_STATE* pIsoface);
public DIO_CTRL_STATUS DIO_setStatus(DO_LIST doID, Bool bOnOFf);
//-public DIO_CTRL_STATUS DIO_setStatusGroup(DO_CHIP groupId, DI_STATE diState); for temp

#endif /* DIO_H_ */
