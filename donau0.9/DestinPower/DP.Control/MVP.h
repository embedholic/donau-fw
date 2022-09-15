/*
 * MVP.h
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */

#ifndef MVP_H_
#define MVP_H_

#include "LGS_Common.h"

#define MVP_EXECUTION_PERIOD 0

void MVP_Create( void );
void  MVP_ClearOneSencondTick(void);
Bool  MVP_GetOneSencondTick(void);
#if PEAK_MODULE_ENB
void MVP_UpdateVoltsOffset( void );
#endif
void MVP_Process10msec( void );
void MVP_Process20msec( void );
void  MVP_Clear100msecTick(void);
Bool  MVP_Get100msecTick(void);
void MVP_AddTrace(void);

typedef struct
{
	Int16 *ac_charge_kwh; 		//-AC 충전 누적 전력량 kWh
	Int16 *ac_charge_mwh; 		//-MWh
	Int16 *ac_charge_gwh; 		//-GWh
	Int16 *ac_discharge_kwh; 	//-AC 방전 누적 전력량 kWh
	Int16 *ac_discharge_mwh; 	//-MWh
	Int16 *ac_discharge_gwh; 	//-GWh
	Int16 *dc_charge_kwh; 		//-DC 충전 누적 전력량 kWh
	Int16 *dc_charge_mwh; 		//-MWh
	Int16 *dc_charge_gwh; 		//-GWh
	Int16 *dc_discharge_kwh; 	//-DC 방전 누적 전력량 kWh
	Int16 *dc_discharge_mwh; 	//-MWh
	Int16 *dc_discharge_gwh; 	//-GWh
}ACC_POWER;
extern ACC_POWER accPower;
#endif /* MVP_H_ */
