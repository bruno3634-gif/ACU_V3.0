/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define RX_BUFFER_SIZE 32
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WDT_PULSE_Pin GPIO_PIN_0
#define WDT_PULSE_GPIO_Port GPIOC
#define SDC_FEEDBACK_Pin GPIO_PIN_1
#define SDC_FEEDBACK_GPIO_Port GPIOC
#define HB_Pin GPIO_PIN_5
#define HB_GPIO_Port GPIOC
#define CAN_INDICATO_Pin GPIO_PIN_0
#define CAN_INDICATO_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define Rear_Solenoid_Pin GPIO_PIN_12
#define Rear_Solenoid_GPIO_Port GPIOB
#define Front_Solenoid_Pin GPIO_PIN_13
#define Front_Solenoid_GPIO_Port GPIOB
#define ASSI_YELLOW_Pin GPIO_PIN_14
#define ASSI_YELLOW_GPIO_Port GPIOB
#define ASSI_BLUE_Pin GPIO_PIN_15
#define ASSI_BLUE_GPIO_Port GPIOB
#define MS1_Pin GPIO_PIN_6
#define MS1_GPIO_Port GPIOC
#define MS2_Pin GPIO_PIN_7
#define MS2_GPIO_Port GPIOC
#define MS3_Pin GPIO_PIN_8
#define MS3_GPIO_Port GPIOC
#define MS4_Pin GPIO_PIN_9
#define MS4_GPIO_Port GPIOC
#define MS5_Pin GPIO_PIN_8
#define MS5_GPIO_Port GPIOA
#define ASMS_Pin GPIO_PIN_2
#define ASMS_GPIO_Port GPIOD
#define MS_BTN_Pin GPIO_PIN_3
#define MS_BTN_GPIO_Port GPIOB
#define MS_BTN_EXTI_IRQn EXTI3_IRQn
#define IGN_BTN_Pin GPIO_PIN_4
#define IGN_BTN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


#define CAN_MSG_MAX_TIMEOUT 1000

typedef enum {
	AS_STATE_OFF = 1,       // 1
	AS_STATE_READY = 2,     // 2
	AS_STATE_DRIVING = 3,   // 3
	AS_STATE_EMERGENCY = 4, // 4
	AS_STATE_FINISHED = 5   // 5
} AS_STATE_t;

typedef enum {
	MANUAL,       // 0
	ACCELERATION, // 1
	SKIDPAD,      // 2   // 3
	TRACKDRIVE,   // 4
	EBS_TEST,     // 5
	INSPECTION,   // 6
	AUTOCROSS     // 7
} current_mission_t;


typedef enum{
	NONE,
	SDC_OPEN,
	RES,
	Pressure_checks,
	VCU_Timeout,
	Jetson_timeout,
	ACU_WDT_TRIGERED,
	dir_actuator_timeout,
	Dynamics_REAR_Pressure_timeout,
	UNKOWN
}Emergency_cause_t;


typedef enum{
	BLE_IDLE,
	BLE_ENTER_CMD,
	BLE_CONNECT,
	BLE_READ_CURRENT_TIME, // BLE CTS
	BLE_PROCESS_DATE,
	BLE_EXIT_CMD,
	BLE_BRIDGE
}BLE_STATE_MACHINE_t;

typedef enum {
    CTS_RESULT_PENDING,
    CTS_RESULT_OK,
    CTS_RESULT_NO_DEVICE,
    CTS_RESULT_TIMEOUT,
    CTS_RESULT_PARSE_ERROR
} CTS_Result;

struct pressure {
	float Pneumatic;
	float Hydraulic;
};

struct speed {
	uint8_t Speed;			// Km/h
	uint8_t Target_Speed;	// Km/h
};

struct car {
	struct pressure Rear_Pressure;
	struct pressure Front_Pressure;
	uint8_t front_solenoid;
	uint8_t rear_solenoid;
	uint8_t Ignition_Status;			// 0 - OFF, 1 - ON     (Real from VCU)
	uint8_t Ignition_Request;		// 0 - OFF, 1 - ON		(Request from ACU)
	uint8_t ASMS;								// 0 - OFF, 1 - ON
	uint8_t Emergency;							// 0 - No , 1 - Emergency
	uint8_t Res;					// 0 - Not active, 1 - ON , 2 - Emergency
	uint8_t HW_WDT_Enable;
	uint8_t Solenoid1_Request;					// 0 - OFF, 1 - ON
	uint8_t Solenoid2_Request;					// 0 - OFF, 1 - ON
	uint8_t ignition_pin_state;
	uint8_t SDC_feedback;
	uint8_t ASSI_state;
	uint8_t vcu_sdc;
	volatile AS_STATE_t Autonomous_State; 		// Autonomous system state
	volatile current_mission_t Current_Mission; // Current mission state
	volatile current_mission_t Jetson_mission;
	struct speed Speed;
	float chip_temp;
	int rpm;
	uint32_t VCU_LAST_TX,REAR_PRESSURE_LAST_TX,JETSON_LAST_TX,DIR_ACTUATOR_LAST_TX;
};

typedef enum {
	Start, IDLE, AS_ON, EMERGENCY
} Main_state_machine_t;



typedef enum {
	OFF,
	Initial_Sequence,
	Monitor_sequence,
	AS_Emergency,
	Finish

}Autonomous_System_states_t;


typedef enum {
	NO_TIMEOUT,
	VCU_TIMEOUT,
	JETSON_TIMEOUT,
	PRESSURE_TIMEOUT,
	DIR_TIMEOUT
}can_timeouts_t;


typedef enum{
	Watchdog_check,
	Pressure_check,
	HV_activation,
	Pressure_correlation_check,
	MB1_Check, // Solenoide 1
	MB2_Check, // Solenoide 2
	Error_state
} startup_sequence_state_t;

struct can_timeouts {
	unsigned long vcu;
	unsigned long res;
	unsigned long jetson;
};

struct can_queue {
	uint32_t TX_MAILBOX;
	CAN_TxHeaderTypeDef can_tx_header;
	CAN_RxHeaderTypeDef can_rx_header;
	uint8_t tx_data[8];
	uint32_t arrival_time;
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
