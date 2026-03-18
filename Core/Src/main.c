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
#include <stdio.h>
#include "Autonomous_functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <stdio.h>

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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct car t24;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Handle_state();
void Handle_autonomous_state();
void Handle_Emergency();
void Peripheral_aquisition();
void toggle_wdt();
float GetTemperature(uint16_t raw_temp, uint16_t raw_vref);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Main_state_machine_t Vehicle_state_machine = IDLE;
Autonomous_System_states_t Autonomous_state = OFF;

uint32_t ADC_Samples[4];

float temporary_temp = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	/***
	 * Var initialization
	 ***/

	t24.Front_Pressure.Hydraulic = 0;
	t24.Front_Pressure.Pneumatic = 0;
	t24.Rear_Pressure.Hydraulic = 0;
	t24.Rear_Pressure.Pneumatic = 0;
	t24.Ignition_Status = 0;
	t24.Ignition_Request = 0;
	t24.Speed.Speed = 0;
	t24.Speed.Target_Speed = 0;
	t24.Emergency = 0;
	t24.Res = 0;
	t24.Current_Mission = MANUAL;
	t24.Autonomous_State = AS_STATE_OFF;
	t24.HW_WDT_Enable = 1;

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
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Samples, 4);

  HAL_TIM_Base_Start(&htim8);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Peripheral_aquisition();
		temporary_temp = t24.chip_temp;
		Handle_state();
		toggle_wdt();
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
void Handle_state() {
	switch (Vehicle_state_machine) {
	case Start:
		t24.HW_WDT_Enable = 1;

		Vehicle_state_machine = IDLE;
		break;
	case IDLE:
		//ASMS = 1 advance
		Vehicle_state_machine = AS_ON;
		break;
	case AS_ON:
		Autonomous_state = Initial_Sequence;
		Handle_autonomous_state();
		break;
	case EMERGENCY:
		Handle_Emergency();
		break;
	default:
		break;
	}
}



void Handle_autonomous_state(){
	switch (Autonomous_state) {
		case Initial_Sequence:
			initial_sequence();
			break;
		case Monitor_sequence:
			break;
		case Finish:
			/***
			 * Ensure car stoped safely
			 */
			if(t24.ASMS == 0){
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


void Handle_Emergency(){
	t24.HW_WDT_Enable = 0;
	t24.Ignition_Request = 0;
	HAL_GPIO_WritePin(Solenoid2_GPIO_Port, Solenoid2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Solenoid1_GPIO_Port, Solenoid1_Pin, GPIO_PIN_RESET);
}




void Peripheral_aquisition(){
	t24.ASMS = HAL_GPIO_ReadPin(ASMS_GPIO_Port, ASMS_Pin);
	t24.SDC_feedback = HAL_GPIO_ReadPin(SDC_FEEDBACK_GPIO_Port, SDC_FEEDBACK_Pin);
}

void toggle_wdt(){
	static unsigned long wdt_time = 0;

	if(HAL_GetTick() - wdt_time >= 10){
		if(t24.HW_WDT_Enable == 1){
			HAL_GPIO_TogglePin(WDT_PULSE_GPIO_Port, WDT_PULSE_Pin);
			wdt_time = HAL_GetTick();
		}
	}

}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
        t24.Front_Pressure.Pneumatic = ADC_Samples[0]; // missing conversion
        t24.Rear_Pressure.Pneumatic  = ADC_Samples[1]; // missing conversion
        t24.chip_temp = GetTemperature(ADC_Samples[2], ADC_Samples[3]);
    }
}


/* USER CODE BEGIN 4 */
float GetTemperature(uint16_t raw_temp, uint16_t raw_vref)
{
    if (raw_vref == 0) return 0.0f;

    // 1. Calcular VDDA real usando a calibração de fábrica do VREF (3.3V)
    // VDDA = 3.3V * (VREFINT_CAL / Raw_VREF)
    float vdda = 3.3f * ((float)(*VREFINT_CAL_ADDR) / (float)raw_vref);

    // 2. Ajustar o valor lido da temperatura para a escala de 3.3V
    // (A calibração TS_CAL foi feita a 3.3V, se o teu VDDA for diferente, o valor "mexe")
    float raw_equiv_3v3 = (float)raw_temp * (vdda / 3.3f);

    // 3. Interpolação Linear
    float ts_cal1 = (float)(*TS_CAL1_ADDR);
    float ts_cal2 = (float)(*TS_CAL2_ADDR);

    float temperature = ((raw_equiv_3v3 - ts_cal1) * (TEMP_CAL2_TEMPC - TEMP_CAL1_TEMPC)
                        / (ts_cal2 - ts_cal1)) + TEMP_CAL1_TEMPC;

    return temperature;
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
