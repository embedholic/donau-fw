/*
 * Queue.h
 *
 *  Created on: 2012. 11. 12.
 *      Author: destinPower
 */

#ifndef QUEUE_H_
#define QUEUE_H_

#include "LGS_Common.h"
#include "history.h"

public void QUEUE_Create();
public Bool QUEUE_enqueHistory(HISTORY_STORE hisData );
public Bool QUEUE_dequeHistory(HISTORY_STORE* pHisData);

#endif /* QUEUE_H_ */
