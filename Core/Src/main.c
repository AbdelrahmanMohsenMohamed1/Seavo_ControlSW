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
#include "can.h"
#include "tim.h"
#include "usart.h"
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
//void SpeedControl_Task    (void  * argument);
void DashboardDisplay_Task    (void  * argument);


CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[8];
uint32_t              TxMailbox;


CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];

int32_t Rec_ERPM;
int16_t Rec_Current;
int16_t Rec_DutyCycle;
uint32_t Rec_AhUsed;
uint32_t Rec_AhCharged;
uint32_t Rec_WhUsed;
uint32_t Rec_WhCharged;
int16_t Rec_TempFET;
int16_t Rec_TempMotor;
int16_t Rec_CurrentIn;
int16_t Rec_PIDPos;
int32_t Rec_Tachometer;
int16_t Rec_VoltageIn;


uint8_t commandID=0;
int32_t ERPM=0;
float Current=5;
float DutyCycle =0;
float AhUsed =0;
float AhCharged=0;
uint32_t WhUsed=0;
uint32_t WhCharged=0;
int16_t TempFET = 0;
int16_t TempMotor = 0;
int16_t CurrentIn = 0;
int16_t PIDPos = 0;
int32_t Tachometer = 0;
int16_t VoltageIn =0;


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
	Displat_Init,
	Display_Black,
	Display_Intro,
	Display_Gear0,
	Display_Gear1,
	Display_Gear2,
	Display_Gear3,
	Display_Gear4,
	Display_Gear5,
}Display_t;
//Display_t prev_display= Display_Black;
Display_t current_display=Displat_Init;
typedef enum
{
	System_OpenSequenceOn,
	System_Operation,
	System_OpenSequenceOff,
}System_State_t;
System_State_t System_State;
uint8_t Display_BlackScreen[9]={0x70,0x61,0x67,0x65,0x20,0x30,0xff,0xff,0xff};
uint8_t Display_IntroScreen[9]={0x70,0x61,0x67,0x65,0x20,0x31,0xff,0xff,0xff};
uint8_t Display_Speed0[9]={0x70,0x61,0x67,0x65,0x20,0x32,0xff,0xff,0xff};
uint8_t Display_Speed1[9]={0x70,0x61,0x67,0x65,0x20,0x33,0xff,0xff,0xff};
uint8_t Display_Speed2[9]={0x70,0x61,0x67,0x65,0x20,0x34,0xff,0xff,0xff};
uint8_t Display_Speed3[9]={0x70,0x61,0x67,0x65,0x20,0x35,0xff,0xff,0xff};
uint8_t Display_Speed4[9]={0x70,0x61,0x67,0x65,0x20,0x36,0xff,0xff,0xff};
uint8_t Display_Speed5[9]={0x70,0x61,0x67,0x65,0x20,0x37,0xff,0xff,0xff};
uint8_t RPM[7]={'n','0','.','v','a','l','='};
uint8_t CUR[7]={'n','2','.','v','a','l','='};
uint8_t DUTY[7]={'n','1','.','v','a','l','='};
uint8_t WH[7]={'n','3','.','v','a','l','='};
uint8_t Termination[3] = {0xff,0xff,0xff};
int size=0;
uint8_t arr[50];

int int32ToAsciiArray(int32_t number, uint8_t arr[], int maxSize) ;
int int32ToAsciiArray(int32_t number, uint8_t arr[], int maxSize) {
	int index = 0;
	char isNegative = 0;

	// Handle negative numbers by converting them to positive and recording the sign
	if (number < 0) {
		isNegative = 1;
		number = -number;
	}

	// Determine the digits and store them in reverse order
	do {
		if (index < maxSize) {
			arr[index++] = (number % 10) + '0';
		}
		number /= 10;
	} while (number > 0);

	// Add the negative sign if the number was negative
	if (isNegative && index < maxSize) {
		arr[index++] = '-';
	}

	// Reverse the array to get the correct order
	for (int i = 0; i < index / 2; ++i) {
		uint8_t temp = arr[i];
		arr[i] = arr[index - 1 - i];
		arr[index - 1 - i] = temp;
	}

	// Return the number of characters stored in the array
	return index;
}



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
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	System_State = System_OpenSequenceOn;
	xTaskCreate(ShutDowenSequenceFunctio_Task, NULL, 128 , NULL , 4 , NULL);
	xTaskCreate(ControlMainFunction_Task, NULL, 128 , NULL , 2 , NULL);
	//xTaskCreate(SpeedControl_Task, NULL, 100 , NULL , 3 , NULL);
	xTaskCreate(DashboardDisplay_Task, NULL, 128 , NULL , 1 , NULL);

	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1 , 500);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	HAL_CAN_Start(&hcan);
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}
	//uint8_t ter = 0xff;
	//HAL_UART_Transmit(&huart1, Display_IntroScreen, sizeof(Display_IntroScreen), HAL_MAX_DELAY);

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
		// Get the message
		//			if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
		//			{
		//				// Reception error
		//				uint8_t Test=1;
		//			}

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
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
				vTaskDelay(2000);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
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
			//if (RighProximity_State != GPIO_PIN_RESET || LeftProximity_State != GPIO_PIN_RESET)
			if (RighProximity_State != GPIO_PIN_RESET ) {
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
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
				vTaskDelay(5000);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			}
		}
		// Read the proximity states
		RighProximity_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
		LeftProximity_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

		// Check if the system is in operation mode and both proximity sensors are not triggered
		//if (System_State == System_Operation && RighProximity_State == GPIO_PIN_RESET && LeftProximity_State == GPIO_PIN_RESET)
		if (System_State == System_Operation && RighProximity_State == GPIO_PIN_RESET) {
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

		vTaskDelay(30);
	}
}

//void SpeedControl_Task(void *argument) {
//	while (1) {
//		// Read the proximity states
//		RighProximity_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
//		LeftProximity_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
//
//		// Check if the system is in operation mode and both proximity sensors are not triggered
//		//if (System_State == System_Operation && RighProximity_State == GPIO_PIN_RESET && LeftProximity_State == GPIO_PIN_RESET)
//		if (System_State == System_Operation && RighProximity_State == GPIO_PIN_RESET) {
//			// Start the PWM signal
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//
//			// Adjust the current speed to match the requested speed
//			if (u8_RequestedSpeed > current_speed) {
//				current_speed++; // Increment current speed
//			} else if (u8_RequestedSpeed < current_speed) {
//				current_speed--; // Decrement current speed
//			}
//
//			// Set the new compare value to adjust the duty cycle
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_speed);
//
//
//
//			// Restart the PWM with the new compare value
//			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//		} else {
//			// If the system is not in operation or proximity sensors are triggered, stop the PWM
//			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//		}
//
//		// Add a delay to control the speed adjustment rate
//		vTaskDelay(20); // Adjust this value as needed
//	}
//}

//void DashboardDisplay_Task    (void  * argument)
//{
//	while(1)
//	{
//		switch(System_State)
//		{
//		case System_OpenSequenceOn:
//			if (current_display != Display_Black)
//			{
//				current_display = Display_Black;
//				// Send UART Display Black Page
//			}
//
//			// Black screen
//			break;
//		case System_Operation:
//			if (current_display == Display_Black) // Check if we are transitioning from OpenSequence
//			{
//				current_display = Display_Intro;
//				// Send UART command to display the Intro Page
//				//vTaskDelay(3000); // Display intro for 3 seconds
//			}
//
//			switch(u8_RequestedSpeed)
//			{
//			case 500:
//				if(current_display != Display_Gear0)
//				{
//					current_display = Display_Gear0;
//					// Send UART Display Gear 0 Page
//				}
//				break;
//			case 590:
//				if(current_display != Display_Gear1)
//				{
//					current_display = Display_Gear1;
//					// Send UART Display Gear 1 Page
//				}
//				break;
//			case 680:
//				if(current_display != Display_Gear2)
//				{
//					current_display = Display_Gear2;
//					// Send UART Display Gear 2 Page
//				}
//				break;
//			case 770:
//				if(current_display != Display_Gear3)
//				{
//					current_display = Display_Gear3;
//					// Send UART Display Gear 3 Page
//				}
//				break;
//			case 860:
//
//				if(current_display != Display_Gear4)
//				{
//					current_display = Display_Gear4;
//					// Send UART Display Gear 4 Page
//				}
//				break;
//			case 950:
//				if(current_display != Display_Gear5)
//				{
//					current_display = Display_Gear5;
//					// Send UART Display Gear 5 Page
//				}
//				break;
//			default: break;
//
//			}
//			break;
//
//
//			default : break;
//		}
//		vTaskDelay(500);
//	}
//}
void DashboardDisplay_Task(void *argument)
{
	while(1)
	{
		switch(System_State)
		{
		case System_OpenSequenceOn:
			if (current_display != Display_Black)
			{
				current_display = Display_Black;
				// Send UART command to display the Black Page
				HAL_UART_Transmit(&huart1, Display_BlackScreen, 9, 1000);
			}
			break;

		case System_Operation:
			if (current_display == Display_Black) // Check if we are transitioning from OpenSequence
			{
				current_display = Display_Intro;
				// Send UART command to display the Intro Page
				HAL_UART_Transmit(&huart1, Display_IntroScreen, 9, 1000);
				vTaskDelay(8500); // Display intro for 3 seconds
			}

			// Handle gear display based on requested speed
			switch(u8_RequestedSpeed)
			{
			case 500:
				if(current_display != Display_Gear0)
				{
					current_display = Display_Gear0;
					HAL_UART_Transmit(&huart1, Display_Speed0, 9, 1000);
					// Send UART command to display Gear 0 Page
				}
				break;
			case 590:
				if(current_display != Display_Gear1)
				{
					current_display = Display_Gear1;
					HAL_UART_Transmit(&huart1, Display_Speed1, 9, 1000);
					// Send UART command to display Gear 1 Page
				}
				break;
			case 680:
				if(current_display != Display_Gear2)
				{
					current_display = Display_Gear2;
					HAL_UART_Transmit(&huart1, Display_Speed2, 9, 1000);
					// Send UART command to display Gear 2 Page
				}
				break;
			case 770:
				if(current_display != Display_Gear3)
				{
					current_display = Display_Gear3;
					HAL_UART_Transmit(&huart1, Display_Speed3, 9, 1000);
					// Send UART command to display Gear 3 Page
				}
				break;
			case 860:
				if(current_display != Display_Gear4)
				{
					current_display = Display_Gear4;
					HAL_UART_Transmit(&huart1, Display_Speed4, 9, 1000);
					// Send UART command to display Gear 4 Page
				}
				break;
			case 950:
				if(current_display != Display_Gear5)
				{
					current_display = Display_Gear5;
					HAL_UART_Transmit(&huart1, Display_Speed5, 9, 1000);
					// Send UART command to display Gear 5 Page
				}
				break;
			default: break;
			}
			break;



			default: break;
		}

		if(DutyCycle >= 0)
		{
			size = int32ToAsciiArray(DutyCycle , arr , sizeof(arr));
			HAL_UART_Transmit(&huart1, DUTY, sizeof(DUTY), 1000);
			HAL_UART_Transmit(&huart1, arr, size, 1000);
			HAL_UART_Transmit(&huart1, Termination, sizeof(Termination), 1000);
		}


		if(Current >= 0)
		{
			size = int32ToAsciiArray(Current , arr , sizeof(arr));
			HAL_UART_Transmit(&huart1, CUR, sizeof(CUR), 1000);
			HAL_UART_Transmit(&huart1, arr, size, 1000);
			HAL_UART_Transmit(&huart1, Termination, sizeof(Termination), 1000);
		}

		if(ERPM >= 0)
		{
			size = int32ToAsciiArray(ERPM , arr , sizeof(arr));
			HAL_UART_Transmit(&huart1, RPM, sizeof(RPM), 1000);
			HAL_UART_Transmit(&huart1, arr, size, 1000);
			HAL_UART_Transmit(&huart1, Termination, sizeof(Termination), 1000);
		}

		if(WhUsed >= 0)
		{
			size = int32ToAsciiArray(WhUsed , arr , sizeof(arr));
			HAL_UART_Transmit(&huart1, WH, sizeof(WH), 1000);
			HAL_UART_Transmit(&huart1, arr, size, 1000);
			HAL_UART_Transmit(&huart1, Termination, sizeof(Termination), 1000);
		}
		vTaskDelay(250); // Task delay
	}
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	// Get the message
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
	{
		// Reception error
	}

	// Extract the Command ID from the received message
	uint8_t commandID = (rxHeader.ExtId >> 8) & 0xFF;

	switch (commandID)
	{
	case 9: // CAN_PACKET_STATUS
	{
		Rec_ERPM = (rxData[0] << 24) | (rxData[1] << 16) | (rxData[2] << 8) | rxData[3];
		ERPM = Rec_ERPM;
		Rec_Current = (rxData[4] << 8) | rxData[5];
		Current = (float)Rec_Current / 10.0;
		Rec_DutyCycle = (rxData[6] << 8) | rxData[7];
		DutyCycle = (float)Rec_DutyCycle / 10.0;

		// Process ERPM, Current, DutyCycle
	}
	break;

	case 14: // CAN_PACKET_STATUS_2
	{
		Rec_AhUsed = (rxData[0] << 24) | (rxData[1] << 16) | (rxData[2] << 8) | rxData[3];
		AhUsed = (float)Rec_AhUsed / 10000.0;
		Rec_AhCharged = (rxData[4] << 24) | (rxData[5] << 16) | (rxData[6] << 8) | rxData[7];
		AhCharged = (float)Rec_AhCharged / 10000.0;

		// Process AhUsed, AhCharged
	}
	break;

	case 15: // CAN_PACKET_STATUS_3
	{
		Rec_WhUsed = (rxData[0] << 24) | (rxData[1] << 16) | (rxData[2] << 8) | rxData[3];
		WhUsed = (float)Rec_WhUsed / 10000.0;
		Rec_WhCharged = (rxData[4] << 24) | (rxData[5] << 16) | (rxData[6] << 8) | rxData[7];
		WhCharged = (float)Rec_WhCharged / 10000.0;

		// Process WhUsed, WhCharged
	}
	break;

	case 16: // CAN_PACKET_STATUS_4
	{
		Rec_TempFET = (rxData[0] << 8) | rxData[1];
		TempFET = (float)Rec_TempFET / 10.0;
		Rec_TempMotor = (rxData[2] << 8) | rxData[3];
		TempMotor = (float)Rec_TempMotor / 10.0;
		Rec_CurrentIn = (rxData[4] << 8) | rxData[5];
		CurrentIn = (float)Rec_CurrentIn / 10.0;
		Rec_PIDPos = (rxData[6] << 8) | rxData[7];
		PIDPos = (float)Rec_PIDPos / 50.0;

		// Process TempFET, TempMotor, CurrentIn, PIDPos
	}
	break;

	case 27: // CAN_PACKET_STATUS_5
	{
		Rec_Tachometer = (rxData[0] << 24) | (rxData[1] << 16) | (rxData[2] << 8) | rxData[3];
		Tachometer = (float)Rec_Tachometer / 6.0;
		Rec_VoltageIn = (rxData[4] << 8) | rxData[5];
		VoltageIn = (float)Rec_VoltageIn / 10.0;

		// Process Tachometer, VoltageIn
	}
	break;

	case 28: // CAN_PACKET_STATUS_6
	{
		//	            int16_t ADC1 = (rxData[0] << 8) | rxData[1];
		//	            int16_t ADC2 = (rxData[2] << 8) | rxData[3];
		//	            int16_t ADC3 = (rxData[4] << 8) | rxData[5];
		//	            int16_t PPM = (rxData[6] << 8) | rxData[7];

		// Process ADC1, ADC2, ADC3, PPM
	}
	break;

	default:
		// Handle unknown or unsupported command IDs
		break;
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
