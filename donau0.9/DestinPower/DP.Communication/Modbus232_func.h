/*
 * Modbus232_func.h
 *
 *  Created on: 2013. 11. 8.
 *      Author: oci-ems
 */

#ifndef MODBUS232_FUNC_H_
#define MODBUS232_FUNC_H_

Int16 MT_getPcs_A(int offset);
int MT_getRealTimeData(int ID);
Uint16 MF_getStatusInfo_A();
Uint16 MF_getStatusInfo_B();

int MF_TestCmd(UInt16 val);
void MF_PCS_Command(int wReg);


#endif /* MODBUS232_FUNC_H_ */
