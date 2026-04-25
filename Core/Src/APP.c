/*
 * APP.c
 *
 *  Created on: Apr 25, 2026
 *      Author: bruno
 */

#include "APP.h"



struct car t24;


extern uint32_t ADC_Samples[4];
uint8_t prev_ASMS = 0;
uint8_t ASSI_leds_control_signal = 0;
float temporary_temp = 0;

Main_state_machine_t Vehicle_state_machine;
Autonomous_System_states_t Autonomous_state;
startup_sequence_state_t startup_sequence_state;



void app_init() {

	/***
	 * Var initialization
	 ***/

	t24.Front_Pressure.Hydraulic = 0;
	t24.Front_Pressure.Pneumatic = 0;
	t24.Rear_Pressure.Hydraulic = 0;
	t24.Rear_Pressure.Pneumatic = 0;
	t24.Ignition_Status = 0;
	t24.Ignition_Request = 0;
	t24.Solenoid1_Request = 0;
	t24.Solenoid2_Request = 0;
	t24.Speed.Speed = 0;
	t24.Speed.Target_Speed = 0;
	t24.Emergency = 0;
	t24.Res = 0;
	t24.Current_Mission = MANUAL;
	t24.Autonomous_State = AS_STATE_OFF;
	t24.HW_WDT_Enable = 1;


	 Vehicle_state_machine = IDLE;
	 Autonomous_state = OFF;
	 startup_sequence_state = Watchdog_check;


	HAL_CAN_Start(&hcan1);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_Samples, 4);

	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_Base_Start_IT(&htim2);

	/*char *enter_cmd = "$$$";
	 char *set_name = "SN,ACU\r";
	 char *reboot = "R,1\r";

	 HAL_UART_Transmit(&huart2, (uint8_t*) enter_cmd, 3, 100);
	 HAL_Delay(100); // Pequena pausa para o módulo responder CMD>

	 // 2. Definir o nome
	 HAL_UART_Transmit(&huart2, (uint8_t*) set_name, strlen(set_name), 100);
	 HAL_Delay(100);

	 // 3. Reiniciar para aplicar
	 HAL_UART_Transmit(&huart2, (uint8_t*) reboot, strlen(reboot), 100);*/
	HAL_CAN_Start(&hcan1);

	char cmd_buff[64];
	int len;

	HAL_UART_Transmit(&huart2, (uint8_t*) "$$$", 3, 100);
	HAL_Delay(200);

	rn4871_set_name(cmd_buff, sizeof(cmd_buff), "ACU_V3");
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd_buff, strlen(cmd_buff), 100);
	HAL_Delay(100);

	rn4871_cmd_exit_mode(cmd_buff, sizeof(cmd_buff));
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd_buff, strlen(cmd_buff), 100);
	HAL_Delay(100);

	// 4. Agora sim, Reboot
	rn4871_cmd_reboot(cmd_buff, sizeof(cmd_buff));
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd_buff, strlen(cmd_buff), 100);
}

void app() {
	prev_ASMS = t24.ASMS;
	Peripheral_aquisition(&ASSI_leds_control_signal);
	temporary_temp = t24.chip_temp;
	Handle_state(prev_ASMS);
	toggle_wdt();
	handle_uart_logs();
	LED_indicator_controller();
	ASSI_control(ASSI_leds_control_signal, Autonomous_state);
	Peripheral_actuation();
	HAL_GPIO_WritePin(ASSI_BLUE_GPIO_Port, ASSI_BLUE_Pin, 1);
	handle_can_tx();
}
