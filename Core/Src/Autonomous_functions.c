/*
 * Autonomous_functions.c
 *
 *  Created on: Mar 14, 2026
 *      Author: bruno, Vasco
 */

#include "Autonomous_functions.h"
#include "hardware_abstraction.h"

#define TIMEOUT_SDC_MS 500
#define TIMEOUT_PRESSURE_MS 2000
#define TIMEOUT_HV_MS 5000
#define EBS_MIN_BAR 6.0f
#define EBS_MAX_BAR 10.0f
#define EBS_HYD_GAIN 10.0f
#define EBS_HYD_UNLOADED_BAR 1.0f

// macros
#define IN_RANGE(val, min, max) ((val) > (min) && (val) < (max))
#define IS_CORRELATED(hyd, pneu) ((hyd) > (EBS_HYD_GAIN * (pneu)))
#define IS_UNLOADED(hyd) ((hyd) < EBS_HYD_UNLOADED_BAR)


static uint32_t state_timer = 0;

void initial_sequence(struct car *t24, startup_sequence_state_t *seq_status, Main_state_machine_t *Vehicle_state_machine) {
	switch (*seq_status) {
		case Watchdog_check:
			if (t24->SDC_feedback == 1) {
				t24->HW_WDT_Enable = 1;
				state_timer = millis();
				*seq_status = Pressure_check;
			} else if (check_timeout(state_timer, TIMEOUT_SDC_MS)) {
				*seq_status = Error_state;
			}
			break;

		/*
		 * BP1 & BP2 > 6 (bar)
		 * BP1 & BP2 < 10 (bar)
		 * */
		case Pressure_check:
			if (IN_RANGE(t24->Front_Pressure.Pneumatic, EBS_MIN_BAR, EBS_MAX_BAR) && IN_RANGE(t24->Rear_Pressure.Pneumatic, EBS_MIN_BAR, EBS_MAX_BAR)) {

				state_timer = millis();
				*seq_status = HV_activation;
			} else if (check_timeout(state_timer, TIMEOUT_PRESSURE_MS)) {
				*seq_status = Error_state;
			}
			break;

		case HV_activation:
			t24->Ignition_Request = 1;
			if (t24->Ignition_Status) {
				state_timer = millis();
				*seq_status = Pressure_correlation_check;
			} else if (check_timeout(state_timer, TIMEOUT_HV_MS)) {
				*seq_status = Error_state;
			}
			break;

		case Pressure_correlation_check:

			/*
			 * BP5 > 10 * BP1
			 * BP6 > 10 * BP2
			 * */
			if (IS_CORRELATED(t24->Front_Pressure.Hydraulic, t24->Front_Pressure.Pneumatic)
				&& IS_CORRELATED(t24->Rear_Pressure.Hydraulic, t24->Rear_Pressure.Pneumatic)) {

				if (t24->Ignition_Status == 1) {
					state_timer = millis();
					*seq_status = MB1_Check;
				}

			} else if (check_timeout(state_timer, TIMEOUT_PRESSURE_MS)) {
				*seq_status = Error_state;
			}
			break;

		case MB1_Check:
			// Open MB1, Close MB2
			t24->Solenoid1_Request = 1;
			t24->Solenoid2_Request = 0;

			if (IS_CORRELATED(t24->Front_Pressure.Hydraulic, t24->Front_Pressure.Pneumatic)
				&& IS_UNLOADED(t24->Rear_Pressure.Hydraulic)) {
				state_timer = millis();
				*seq_status = MB2_Check;
			} else if (check_timeout(state_timer, TIMEOUT_PRESSURE_MS)) {
				*seq_status = Error_state;
			}
			break;

		case MB2_Check:
			// Close MB1, Open MB2
			t24->Solenoid1_Request = 0;
			t24->Solenoid2_Request = 1;


			if (IS_CORRELATED(t24->Rear_Pressure.Hydraulic, t24->Rear_Pressure.Pneumatic)
				&& IS_UNLOADED(t24->Front_Pressure.Hydraulic)) {

				t24->Autonomous_State = AS_STATE_READY;
				t24->Solenoid1_Request = 0;
				t24->Solenoid2_Request = 0;
			} else if (check_timeout(state_timer, TIMEOUT_PRESSURE_MS)) {
				*seq_status = Error_state;
			}
			break;

		case Error_state:
			*Vehicle_state_machine = EMERGENCY;
			break;

		default:
			*seq_status = Error_state;
			break;
	}
}
void continuous_monitoring(uint8_t sdc_status,
		struct can_timeouts *last_message_from, float Rear_pneumatic,
		float Front_pneumatic, float Rear_hydraulic, float Front_hydraulic) {
	// CAN Messages timeouts
	/**
	 *
	 * TODO:	Check if sdc is open
	 * 			Check can timeouts
	 * 			Check AS system components
	 * 			Check pneumatic pressure are between 6 and 10 Bar
	 *
	 * 			note: sdc open reads 0
	 *			Reanalyze this
	 */

	if(sdc_status == 0){
		// check pressures
	}else{
		if(millis() - last_message_from->jetson > 500 || millis() - last_message_from->res > 500 || millis() - last_message_from->vcu > 500)
		{
			//enter emergency
			Vehicle_state_machine = EMERGENCY;
		}else if( (Rear_pneumatic > 10 || Rear_pneumatic < 6) || (Front_pneumatic > 10 || Front_pneumatic < 6) ){
			// enter emergency
			Vehicle_state_machine = EMERGENCY;
		}
	}
}

int ASSI_control(uint8_t gpio_state, uint8_t ASSI_state) {

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

	static unsigned long prev_time_yellow = 0;
	static unsigned long prev_time_blue = 0;

	switch (ASSI_state) {
	case 0:
		gpio_state = 0;
		break;
	case (1):
		gpio_state = 0b00000001;
		break;
	case 2:
		if (millis() - prev_time_yellow >= 330) {
			gpio_state ^= 1;
			gpio_state &= 0b00000001;
			prev_time_yellow = millis();
		}
		break;
	case 3:
		if (millis() - prev_time_blue >= 330) {
			gpio_state ^= 0b00000010;
			gpio_state &= 0b00000010;
			prev_time_blue = millis();
		}
		break;
	case 4:
		gpio_state = 0b00000010;
		break;
	default:
		break;
	}

}

bool check_timeout(uint32_t start_time, uint32_t limit) {
	if (millis() - start_time > limit) {
		return true;
	}
	return false;
}

