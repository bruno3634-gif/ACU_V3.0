/*
 * APP.c
 *
 *  Created on: Apr 25, 2026
 *      Author: bruno
 */

#include "APP.h"


struct car t24;
struct can_queue can_rx_data;


extern uint32_t ADC_Samples[4];
uint8_t prev_ASMS = 0;
uint8_t ASSI_leds_control_signal = 0;
float temporary_temp = 0;

Main_state_machine_t Vehicle_state_machine;
Autonomous_System_states_t Autonomous_state;
startup_sequence_state_t startup_sequence_state;

extern struct ring can_tx_ringbuffer;
extern struct ring can_rx_ringbuffer;



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


	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_Samples, 4);

	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_Base_Start_IT(&htim2);


	CAN_FilterTypeDef can_filter;

	can_filter.FilterBank = 0;
	can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter.FilterIdHigh = 0x0000;
	can_filter.FilterIdLow = 0x0000;
	can_filter.FilterMaskIdHigh = 0x0000;
	can_filter.FilterMaskIdLow = 0x0000;
	can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(&hcan1, &can_filter);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);

}

void app() {
	prev_ASMS = t24.ASMS;
	Peripheral_aquisition(&ASSI_leds_control_signal);
	temporary_temp = t24.chip_temp;
	Handle_state(prev_ASMS);
	toggle_wdt();
	//handle_uart_logs();
	LED_indicator_controller();
	ASSI_control(ASSI_leds_control_signal, Autonomous_state);
	Peripheral_actuation();
	HAL_GPIO_WritePin(ASSI_BLUE_GPIO_Port, ASSI_BLUE_Pin, 1);
	handle_can_tx();
	//can_buffer_pop(&can_tx_ringbuffer, 1,NULL);
	can_buffer_pop(&can_rx_ringbuffer, 0,&can_rx_data);
	dbc_decode();
}




void dbc_decode(){
	switch(can_rx_data.can_rx_header.StdId){
	case AUTONOMOUS_T26_AQT7_FRAME_ID:
		struct autonomous_t26_aqt7_t rear_dynamics;
		autonomous_t26_aqt7_unpack(&rear_dynamics, can_rx_data.tx_data, AUTONOMOUS_T26_AQT7_LENGTH);
		t24.Rear_Pressure.Hydraulic = autonomous_t26_aqt7_brk_press_decode(rear_dynamics.brk_press);
		break;
	case AUTONOMOUS_T26_VCU_IGN_R2_D_FRAME_ID:
		struct autonomous_t26_vcu_ign_r2_d_t vcu_data;
		autonomous_t26_vcu_ign_r2_d_unpack(&vcu_data, can_rx_data.tx_data, AUTONOMOUS_T26_VCU_IGN_R2_D_LENGTH);
		t24.Ignition_Status = autonomous_t26_vcu_ign_r2_d_ignition_auto_decode(vcu_data.ignition_auto);
		t24.vcu_sdc = autonomous_t26_vcu_ign_r2_d_shutdown_signal_decode(vcu_data.shutdown_signal);
		break;
	case AUTONOMOUS_T26_JETSON_FRAME_ID
		struct autonomous_t26_jetson_t jetson_data;
		autonomous_t26_jetson_unpack(&jetson_data, can_rx_data.tx_data,AUTONOMOUS_T26_JETSON_LENGTH);
		t24.as_state = autonomous_t26_jetson_as_state_decode(jetson_data.as_state);
		break;
	default:
		break;
	}
}
