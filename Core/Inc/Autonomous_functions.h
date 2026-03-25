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


void continuous_monitoring(uint8_t sdc_status,
		struct can_timeouts *last_message_from, float Rear_pneumatic,
		float Front_pneumatic, float Rear_hydraulic, float Front_hydraulic);


int ASSI_control(uint8_t gpio_state, uint8_t ASSI_state);

#endif /* INC_AUTONOMOUS_FUNCTIONS_H_ */
