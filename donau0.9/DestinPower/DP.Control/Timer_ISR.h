/*
 * Timer_ISR.h
 *
 *  Created on: 2016. 5. 10.
 *      Author: Welcome!
 */

#ifndef DESTINPOWER_DP_CONTROL_TIMER_ISR_H_
#define DESTINPOWER_DP_CONTROL_TIMER_ISR_H_

void Timer_ISR_init(void);
interrupt void cpu_timer0_isr(void);

#endif /* DESTINPOWER_DP_CONTROL_TIMER_ISR_H_ */
