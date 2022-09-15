/*
 * Task.h
 *
 *  Created on: 2012. 11. 5.
 *      Author: destinPower
 */

#ifndef TASK_H_
#define TASK_H_

#include "LGS_Common.h"


typedef struct _RequestCmdOnTask
{
	Bool bReqSetCmd;
	Int16 SetCmdVal;
}REQ_CMD_IDLE;

extern REQ_CMD_IDLE ReqCmdIdle;

#endif /* TASK_H_ */
