/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ControlMainFunction_Task    (void  * argument);
void ShutDowenSequenceFunctio_Task    (void  * argument);
void SpeedControl_Task    (void  * argument);

uint8_t FristTimeFlag = 0;
int32_t u8_RequestedSpeed = 500;
uint8_t u8_TimeOutFlag1 = 0;
uint8_t u8_TimeOutFlag4 = 0;
GPIO_PinState Switch1_State;
GPIO_PinState Switch2_State;
GPIO_PinState Switch15_State;
GPIO_PinState Switch3_State;
int8_t iterator=0;
int32_t current_speed = 500;

GPIO_PinState RighProximity_State;
GPIO_PinState LeftProximity_State;
typedef enum
{
	System_OpenSequenceOn,
	System_Operation,
	System_OpenSequenceOff,
}System_State_t;
System_State_t System_State;

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
	MX_TIM1_Init();
	MX_TIM4_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	System_State = System_OpenSequenceOn;
	xTaskCreate(ShutDowenSequenceFunctio_Task, NULL, 100 , NULL , 3 , NULL);
	xTaskCreate(ControlMainFunction_Task, NULL, 100 , NULL , 1 , NULL);
	xTaskCreate(SpeedControl_Task, NULL, 100 , NULL , 2 , NULL);
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1 , 500);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

void ControlMainFunction_Task(void *argument) {
	while (1) {
		switch (System_State) {
		case System_OpenSequenceOn:
			u8_RequestedSpeed = 500;
			iterator = 1;
			current_speed = 500;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_speed);

			// Check switches
			Switch1_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
			Switch2_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

			if (Switch1_State == GPIO_PIN_RESET && Switch2_State == GPIO_PIN_RESET) {
				//				do{
				//					Switch1_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
				//					Switch2_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
				//				}while(Switch1_State == GPIO_PIN_RESET && Switch2_State == GPIO_PIN_RESET);
				HAL_TIM_Base_Start_IT(&htim3);  // Start timer
			} else {
				HAL_TIM_Base_Stop(&htim3);  // Stop and reset timer
				__HAL_TIM_SET_COUNTER(&htim3, 500);
			}

			if (u8_TimeOutFlag1 == 1) {
				u8_TimeOutFlag1 = 0;
				//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
				HAL_TIM_Base_Stop(&htim3);
				__HAL_TIM_SET_COUNTER(&htim3, 500);
				FristTimeFlag = 1;
				System_State = System_Operation;
			}
			break;

		case System_Operation:
			if (FristTimeFlag) {
				FristTimeFlag = 0;
				vTaskDelay(500);
			}

			// Check switches
			Switch1_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
			Switch2_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

			if (Switch1_State == GPIO_PIN_SET && Switch2_State == GPIO_PIN_RESET) {
				// Increase requested speed
				do {
					Switch2_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
				} while (Switch2_State == GPIO_PIN_RESET);
				u8_RequestedSpeed += 90;
				if (u8_RequestedSpeed > 950) {
					u8_RequestedSpeed = 950;
				}
			} else if (Switch1_State == GPIO_PIN_RESET && Switch2_State == GPIO_PIN_SET) {
				// Decrease requested speed
				do {
					Switch1_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
				} while (Switch1_State == GPIO_PIN_RESET);
				u8_RequestedSpeed -= 90;
				if (u8_RequestedSpeed < 500) {
					u8_RequestedSpeed = 500;
				}
			}
			break;

		default:
			break;
		}
		vTaskDelay(30);
	}
}



void ShutDowenSequenceFunctio_Task(void *argument) {
	while (1) {
		if (System_State == System_Operation) {
			RighProximity_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
			LeftProximity_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

			// Check proximity sensors
			if (RighProximity_State != GPIO_PIN_RESET || LeftProximity_State != GPIO_PIN_RESET) {
				iterator = 0;
				current_speed = 500;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_speed);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			}

			// Check switches
			Switch1_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
			Switch2_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

			if (Switch1_State == GPIO_PIN_RESET && Switch2_State == GPIO_PIN_RESET) {
				//				do{
				//					Switch1_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
				//					Switch2_State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
				//				}while(Switch1_State == GPIO_PIN_RESET && Switch2_State == GPIO_PIN_RESET);
				HAL_TIM_Base_Start_IT(&htim3);  // Start timer
			} else {
				HAL_TIM_Base_Stop(&htim3);  // Stop and reset timer
				__HAL_TIM_SET_COUNTER(&htim3, 500);
			}

			if (u8_TimeOutFlag1 == 2) {
				u8_TimeOutFlag1 = 0;

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
				HAL_TIM_Base_Stop(&htim3);
				__HAL_TIM_SET_COUNTER(&htim3, 500);
				u8_RequestedSpeed = 500;
				iterator = 1;
				current_speed = 500;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_speed);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				System_State = System_OpenSequenceOn;
			}
		}
		vTaskDelay(30);
	}
}

void SpeedControl_Task(void *argument) {
	while (1) {
		// Read the proximity states
		RighProximity_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
		LeftProximity_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

		// Check if the system is in operation mode and both proximity sensors are not triggered
		if (System_State == System_Operation && RighProximity_State == GPIO_PIN_RESET && LeftProximity_State == GPIO_PIN_RESET) {
			// Start the PWM signal
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

			// Adjust the current speed to match the requested speed
			if (u8_RequestedSpeed > current_speed) {
				current_speed++; // Increment current speed
			} else if (u8_RequestedSpeed < current_speed) {
				current_speed--; // Decrement current speed
			}

			// Set the new compare value to adjust the duty cycle
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_speed);



			// Restart the PWM with the new compare value
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		} else {
			// If the system is not in operation or proximity sensors are triggered, stop the PWM
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		}

		// Add a delay to control the speed adjustment rate
		vTaskDelay(20); // Adjust this value as needed
	}
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM2) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	if(htim->Instance == TIM3)
	{
		static uint8_t IntErrorFlag1=0;
		IntErrorFlag1++;
		if(IntErrorFlag1 > 1)
		{
			if(System_State == System_OpenSequenceOn)
			{
				u8_TimeOutFlag1 = 1;
			}
			else if (System_State == System_Operation)
			{
				u8_TimeOutFlag1++;
			}
			else
			{

			}
		}
	}
	if(htim->Instance == TIM4)
	{
		static uint8_t IntErrorFlag4=0;
		IntErrorFlag4++;
		if(IntErrorFlag4 > 1)
		{
			u8_TimeOutFlag4 = 1;
		}
	}
	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
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
