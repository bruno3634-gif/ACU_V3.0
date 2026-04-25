/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "APP.h"
#include <stdio.h>
#include "Autonomous_functions.h"
#include "hardware_abstraction.h"
#include "autonomous_t26.h"
#include "rn4871.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <stdio.h>
uint32_t ADC_Samples[4];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TS_CAL1_ADDR        ((uint16_t*)0x1FFF7A2C)   // Raw @ 30°C, Vdda=3.3V
#define TS_CAL2_ADDR        ((uint16_t*)0x1FFF7A2E)   // Raw @ 110°C, Vdda=3.3V
//#define VREFINT_CAL_ADDR    ((uint16_t*)0x1FFF7A2A)   // Raw Vref @ Vdda=3.3V

#define TEMP_CAL1_TEMPC     30.0f
#define TEMP_CAL2_TEMPC     110.0f

uint16_t raw_vref;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint32_t TX_MAILBOX;
CAN_TxHeaderTypeDef can_tx_header;
uint8_t tx_data[8];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


struct can_queue can_tx_queue[64];
int can_queue_index = -1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */



float GetTemperature(uint16_t raw_temp, uint16_t raw_vref);
void handle_uart_logs();
void LED_indicator_controller();
void handle_can_tx();
void add_can_message(uint32_t mailbox, CAN_TxHeaderTypeDef tx_header,
		uint8_t tx_data[8]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  app_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		app();
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
        t24.Front_Pressure.Pneumatic = ADC_Samples[0]; // missing conversion
        t24.Rear_Pressure.Pneumatic  = ADC_Samples[1]; // missing conversion
        t24.chip_temp = GetTemperature(ADC_Samples[2], ADC_Samples[3]);
    }
}



float GetTemperature(uint16_t raw_temp, uint16_t raw_vref) {
	if (raw_vref == 0)
		return 0.0f;

	// 1. Calcular VDDA real usando a calibração de fábrica do VREF (3.3V)
	// VDDA = 3.3V * (VREFINT_CAL / Raw_VREF)
	float vdda = 3.3f * ((float) (*VREFINT_CAL_ADDR) / (float) raw_vref);

	// 2. Ajustar o valor lido da temperatura para a escala de 3.3V
	// (A calibração TS_CAL foi feita a 3.3V, se o teu VDDA for diferente, o valor "mexe")
	float raw_equiv_3v3 = (float) raw_temp * (vdda / 3.3f);

	// 3. Interpolação Linear
	float ts_cal1 = (float) (*TS_CAL1_ADDR);
	float ts_cal2 = (float) (*TS_CAL2_ADDR);

	float temperature = ((raw_equiv_3v3 - ts_cal1)
			* (TEMP_CAL2_TEMPC - TEMP_CAL1_TEMPC) / (ts_cal2 - ts_cal1))
			+ TEMP_CAL1_TEMPC;

	return temperature;
}

void handle_uart_logs() {
	static unsigned long timestamp = 0;

	if (millis() - timestamp > 500) {
		uint8_t UART_TxBuffer[512];

		int len =
				snprintf(UART_TxBuffer, sizeof(UART_TxBuffer),
						"Chip temperature:%.2f\n\rRear pressure:%.2f\n\rFront Pressure:%0.2f\n\r\0",
						t24.chip_temp, t24.Rear_Pressure, t24.Front_Pressure);

		HAL_UART_Transmit_DMA(&huart2, UART_TxBuffer, len);
		timestamp = millis();
	}
}

void LED_indicator_controller() {
	static unsigned long timestamp = 0;

	if (millis() - timestamp >= 500) {
		HAL_GPIO_TogglePin(HB_GPIO_Port, HB_Pin);
		timestamp = millis();

		can_tx_header.IDE = CAN_ID_STD;
		can_tx_header.RTR = CAN_RTR_DATA;
		can_tx_header.DLC = 1;
		tx_data[0] = 0xFF;
		/*for (int i = 0; i < 20; i++) {
			can_tx_header.StdId = 0x99 + i;
			add_can_message(TX_MAILBOX, can_tx_header, tx_data);
		}*/

	}

}

/***
 * tmr callback
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {


		struct autonomous_t26_acu_t AS_data;

		AS_data.acu_cpu_temp = t24.chip_temp;
		AS_data.as_state = t24.Autonomous_State;
		AS_data.asms = t24.ASMS;
		AS_data.assi_state = t24.ASSI_state;
		AS_data.emergency = t24.Emergency;
		AS_data.ign = t24.Ignition_Request;
		AS_data.mission_select = t24.Current_Mission;

		autonomous_t26_acu_pack(tx_data, &AS_data, AUTONOMOUS_T26_ACU_LENGTH);

		can_tx_header.StdId = AUTONOMOUS_T26_ACU_FRAME_ID;
		can_tx_header.RTR = CAN_RTR_DATA;
		can_tx_header.DLC = AUTONOMOUS_T26_ACU_LENGTH;

		add_can_message(TX_MAILBOX, can_tx_header, tx_data);



	}

	//  	HAL_CAN_AddTxMessage(hcan, pHeader, aData, pTxMailbox)
}

void handle_can_tx() {
	static uint8_t tx_index = 0;
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 && can_queue_index > -1) {
		HAL_CAN_AddTxMessage(&hcan1, &can_tx_queue[tx_index].can_tx_header,
				can_tx_queue[tx_index].tx_data,
				&can_tx_queue[tx_index].TX_MAILBOX);
		tx_index++;
		if (can_queue_index == (tx_index - 1)) {
			can_queue_index = -1;
			tx_index = 0;
		}
	}

}

void add_can_message(uint32_t mailbox, CAN_TxHeaderTypeDef tx_header,
		uint8_t tx_data[8]) {
	can_queue_index++;
	can_tx_queue[can_queue_index].TX_MAILBOX = mailbox;
	can_tx_queue[can_queue_index].can_tx_header = tx_header;
	memcpy(can_tx_queue[can_queue_index].tx_data, tx_data, 8);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
