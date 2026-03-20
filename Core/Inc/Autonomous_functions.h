/*
 * Autonomous_functions.h
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno
 */

#ifndef INC_AUTONOMOUS_FUNCTIONS_H_
#define INC_AUTONOMOUS_FUNCTIONS_H_

#include "main.h"


void initial_sequence();
void continuous_monitoring();
int ASSI_control(uint8_t gpio_state, uint8_t ASSI_state);


#endif /* INC_AUTONOMOUS_FUNCTIONS_H_ */
