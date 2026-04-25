/*
 * APP.h
 *
 *  Created on: Apr 25, 2026
 *      Author: bruno
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include "Autonomous_functions.h"
#include "hardware_abstraction.h"
#include "autonomous_t26.h"
#include "rn4871.h"



extern Main_state_machine_t Vehicle_state_machine;
extern Autonomous_System_states_t Autonomous_state;
extern startup_sequence_state_t startup_sequence_state;


void app_init();
void app();

#endif /* INC_APP_H_ */
