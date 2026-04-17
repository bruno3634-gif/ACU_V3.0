/*
 * Autonomous_functions.h
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno
 */

#ifndef INC_AUTONOMOUS_FUNCTIONS_H_
#define INC_AUTONOMOUS_FUNCTIONS_H_

#include "main.h"
#include <stdbool.h>

extern Main_state_machine_t Vehicle_state_machine;

void initial_sequence(struct car *v, startup_sequence_state_t *seq_status,
		Main_state_machine_t *Vehicle_state_machine);
void continuous_monitoring(uint8_t sdc_status,
		struct can_timeouts *last_message_from, float Rear_pneumatic,
		float Front_pneumatic, float Rear_hydraulic, float Front_hydraulic);
int ASSI_control(uint8_t gpio_state, uint8_t ASSI_state);
bool check_timeout(uint32_t start_time, uint32_t limit);

#endif /* INC_AUTONOMOUS_FUNCTIONS_H_ */
