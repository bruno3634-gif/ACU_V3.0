#include "state_machine.h"


void Handle_autonomous_state() {
	switch (Autonomous_state) {
	case Initial_Sequence:
		initial_sequence(&t24,&startup_sequence_state,&Vehicle_state_machine);
		break;
	case Monitor_sequence:
		break;
	case Finish:
		/***
		 * Ensure car stoped safely
		 */
		if (t24.ASMS == 0) {
		}
		break;
	case AS_Emergency:
		Vehicle_state_machine = EMERGENCY;
		break;
	default:
		Vehicle_state_machine = EMERGENCY;
		break;
	}
}

void Handle_Emergency() {
	t24.HW_WDT_Enable = 0;
	t24.Ignition_Request = 0;
	t24.Emergency = 1;
	HAL_GPIO_WritePin(Front_Solenoid_GPIO_Port, Front_Solenoid_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rear_Solenoid_GPIO_Port, Rear_Solenoid_Pin,
			GPIO_PIN_RESET);
	t24.front_solenoid = 0;
	t24.rear_solenoid = 0;
}

void Handle_state(uint8_t prev_asms_state) {
	static uint8_t as_on_first_time = 0,finished_init_sequence = 0;
	switch (Vehicle_state_machine) {
	case Start:
		t24.HW_WDT_Enable = 1;

		Vehicle_state_machine = IDLE;
		break;
	case IDLE:
		//ASMS = change from 0 to 1 (1 -> btn closed)
		if (t24.ASMS == 1 && prev_asms_state == 0
				&& t24.ignition_pin_state == 0) {
			Vehicle_state_machine = AS_ON;
		} else {
			// do not break;
		}
		break;
	case AS_ON:
		if(finished_init_sequence == 0 ){
			Autonomous_state = Initial_Sequence;
			if(!as_on_first_time){
				startup_sequence_state = Watchdog_check;
				as_on_first_time = 1;
			}
		}
		Handle_autonomous_state();
		break;
	case EMERGENCY:
		Handle_Emergency();
		break;
	default:
		break;
	}
}





void toggle_wdt() {
	static unsigned long wdt_time = 0;

	if (millis() - wdt_time >= 10) {
		if (t24.HW_WDT_Enable == 1) {
			HAL_GPIO_TogglePin(WDT_PULSE_GPIO_Port, WDT_PULSE_Pin);
			//HAL_GPIO_WritePin(WDT_PULSE_GPIO_Port, WDT_PULSE_Pin, GPIO_PIN_SET);
			wdt_time = millis();
		}
	}

}
