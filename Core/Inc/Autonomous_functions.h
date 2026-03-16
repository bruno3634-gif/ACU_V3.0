/*
 * Autonomous_functions.h
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno
 */

#ifndef INC_AUTONOMOUS_FUNCTIONS_H_
#define INC_AUTONOMOUS_FUNCTIONS_H_

#include "main.h"


startup_sequence_state_t initial_sequence_state = Watchdog_check;

void initial_sequence();
void continuous_monitoring();


#endif /* INC_AUTONOMOUS_FUNCTIONS_H_ */
