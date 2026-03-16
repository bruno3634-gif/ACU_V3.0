/*
 * Autonomous_functions.c
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno
 */

#include "Autonomous_functions.h"

extern Main_state_machine_t Vehicle_state_machine;

void initial_sequence() {

	switch (initial_sequence_state) {
	case Watchdog_check:
		if (t24.SDC_feedback == 1) {
			t24.HW_WDT_Enable = 0;
		}else{
			// if 500ms timeout and SDC closed
		}
		break;
	case Pressure_check:

		break;
	case HV_activation:

		break;
	case Pressure_correlation_check:

		break;
	case Error_state:
		Vehicle_state_machine = EMERGENCY;
		break;

	default:
		Vehicle_state_machine = EMERGENCY;
		break;
	}
}
void continuous_monitoring() {

}
