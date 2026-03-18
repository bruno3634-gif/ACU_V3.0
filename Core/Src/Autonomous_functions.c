/*
 * Autonomous_functions.c
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno
 */

#include "Autonomous_functions.h"

extern Main_state_machine_t Vehicle_state_machine;
startup_sequence_state_t initial_sequence_status = Watchdog_check;
extern struct car t24;

void initial_sequence() {

	switch (initial_sequence_status) {
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
	// CAN Messages timeouts
}



int ASSI_controll(uint8_t gpio_state, uint8_t ASSI_state){

	/*
	 * bit 0 -> Yellow
	 * bit 1 -> Blue
	 *
	 * -------------------------------------------------------------------------
	 * AS Status Indicator (ASSI) - Operational States (Ref: T 14.8)
	 * -------------------------------------------------------------------------
	 * AS STATE        | ILLUMINATION STATUS
	 * ----------------|--------------------------------------------------------
	 * AS Off          | Off						0
	 * AS Ready        | Yellow (Continuous)		1
	 * AS Driving      | Yellow (Flashing)			2
	 * AS Emergency    | Blue (Flashing)			3
	 * AS Finished     | Blue (Continuous)			4
	 * -------------------------------------------------------------------------
	 *During “AS Driving” and “AS Emergency” the ASSIs must be flashing continuously with a
	 *frequency between 2 Hz and 5 Hz and a duty cycle of 50 %.
	 *
	 */

	static unsigned long prev_time;

	switch (0) {
		case value:

			break;
		case(1):

		break;
		case 2:

			break;
		case 3:

			break;
		case 4:

			break;
		default:
			break;
	}



}


