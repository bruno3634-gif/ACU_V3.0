/*
 * hardware_abstaction.c
 *
 *  Created on: Mar 23, 2026
 *      Author: bruno
 */


#include "hardware_abstraction.h"


uint32_t millis() {

#if defined (__ARM_ARCH_7EM__)

	return HAL_GetTick();

#else
	struct timespec ts;
	    clock_gettime(CLOCK_MONOTONIC, &ts);
	    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
#endif

}



void Peripheral_aquisition(uint8_t *assi_leds) {
	t24.ASMS = HAL_GPIO_ReadPin(ASMS_GPIO_Port, ASMS_Pin);
	t24.SDC_feedback = !HAL_GPIO_ReadPin(SDC_FEEDBACK_GPIO_Port,
	SDC_FEEDBACK_Pin);
	t24.ignition_pin_state = HAL_GPIO_ReadPin(IGN_BTN_GPIO_Port, IGN_BTN_Pin);
	*assi_leds = HAL_GPIO_ReadPin(ASSI_YELLOW_GPIO_Port, ASSI_YELLOW_Pin) << 1
			| HAL_GPIO_ReadPin(ASSI_BLUE_GPIO_Port, ASSI_BLUE_Pin);
}

void Peripheral_actuation() {
	HAL_GPIO_WritePin(Front_Solenoid_GPIO_Port, Front_Solenoid_Pin,
			t24.front_solenoid);
	HAL_GPIO_WritePin(Rear_Solenoid_GPIO_Port, Rear_Solenoid_Pin,
			t24.rear_solenoid);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,
					!t24.rear_solenoid);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
					!t24.front_solenoid);

}
