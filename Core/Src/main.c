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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "pins.h"
#include "app.h"

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
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId tcu_handlerHandle;
osThreadId conn_handlerHandle;
/* USER CODE BEGIN PV */
char msg[200];

CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];  // dados CAN (até 8 bytes)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskTCU(void const * argument);
void StartTaskconn(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	sprintf(msg, "Iniciando\r\n");
	HAL_UART_Transmit(&huart1, &msg, strlen(msg), HAL_MAX_DELAY);

	if (HAL_CAN_ActivateNotification(&hcan,
			CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO1_MSG_PENDING
					| CAN_IT_RX_FIFO1_FULL | CAN_IT_BUSOFF) != HAL_OK) {
		Error_Handler();
	}

	sprintf(msg, "CAN_START\r\n");
	HAL_UART_Transmit(&huart1, &msg, strlen(msg), HAL_MAX_DELAY);
	HAL_CAN_Start(&hcan);

	sprintf(msg, "FIM CAN_START\r\n");
	HAL_UART_Transmit(&huart1, &msg, strlen(msg), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of tcu_handler */
  osThreadDef(tcu_handler, StartTaskTCU, osPriorityNormal, 0, 256);
  tcu_handlerHandle = osThreadCreate(osThread(tcu_handler), NULL);

  /* definition and creation of conn_handler */
  osThreadDef(conn_handler, StartTaskconn, osPriorityNormal, 0, 512);
  conn_handlerHandle = osThreadCreate(osThread(conn_handler), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	if (tcu_handlerHandle == NULL) {
		sprintf(msg, "ERROR tcu_handlerHandle\r\n");
		HAL_UART_Transmit(&huart1, &msg, strlen(msg), HAL_MAX_DELAY);
	}

	if (conn_handlerHandle == NULL) {
		sprintf(msg, "ERROR conn_handlerHandle\r\n");
		HAL_UART_Transmit(&huart1, &msg, strlen(msg), HAL_MAX_DELAY);
	}
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

	CAN_FilterTypeDef CAN_FilterInitStructure;

	// Configure the filter
	CAN_FilterInitStructure.FilterBank = 0;  // Use filter bank 0
	CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterInitStructure.FilterIdHigh = 0x0000;  // ID high bits
	CAN_FilterInitStructure.FilterIdLow = 0x0000;   // ID low bits
	CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;  // Mask highS bits
	CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;   // Mask low bits
	CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO1; // Assign to FIFO 1
	CAN_FilterInitStructure.FilterActivation = ENABLE;  // Enable filter
	CAN_FilterInitStructure.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan, &CAN_FilterInitStructure);
	if (HAL_CAN_ConfigFilter(&hcan, &CAN_FilterInitStructure) != HAL_OK) {
		sprintf(msg, "HAL_CAN_ConfigFilter ERROR\r\n");
		HAL_UART_Transmit(&huart1, &msg, strlen(msg), HAL_MAX_DELAY);
		Error_Handler();
	}

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, hazard_light_Pin|REB_IPC_warning_Pin|Engine_REB_mode_Pin|REB_IPC_fault_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : hazard_light_Pin REB_IPC_warning_Pin Engine_REB_mode_Pin REB_IPC_fault_pin_Pin */
  GPIO_InitStruct.Pin = hazard_light_Pin|REB_IPC_warning_Pin|Engine_REB_mode_Pin|REB_IPC_fault_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Hazard_lights_button_Pin PA3 REB_deactivate_Pin */
  GPIO_InitStruct.Pin = Hazard_lights_button_Pin|GPIO_PIN_3|REB_deactivate_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	sprintf(msg, "trigger RxFifo \r\n");
	HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK) {
		// Exemplo de tratamento: imprime ID e os dados via UART
		int len = sprintf((char*) msg, "CAN ID: 0x%X DLC: %d Data:",
				rxHeader.StdId, rxHeader.DLC);
		for (uint8_t i = 0; i < rxHeader.DLC; i++) {
			len += sprintf((char*) (msg + len), " %02X", rxData[i]);
		}
		sprintf((char*) (msg + len), "\r\n");

		HAL_UART_Transmit(&huart1, msg, strlen((char*) msg), HAL_MAX_DELAY);

		can_frame frame = { 0 };
		frame.can_id = rxHeader.StdId;
		frame.can_dlc = rxHeader.DLC;

		for (int i = 0; i < 8; i++) {
			frame.data[i] = rxData[i];
		}

		handle_frame_can(&frame);

	} else {
		const char *err = "Erro ao ler mensagem CAN\r\n";
		HAL_UART_Transmit(&huart1, err, strlen(err), HAL_MAX_DELAY);
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	sprintf(msg, "GPIO_Pin -> %4X \r\n", GPIO_Pin);
	HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);

	if (GPIO_Pin == GPIO_PIN_2) {
		uint8_t value = 1U;
		read_pin_status(&value, HAZARD_BUTTON_PIN);
		value = !value;

		sprintf(msg, "tentativa de setar pino GPIO_PIN_2 -: %d \r\n", value);
		HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);

		set_pin_status(value, HAZARD_BUTTON_PIN);
	}
	if (GPIO_Pin == GPIO_PIN_3) {
		uint8_t value = 1U;

		sprintf(msg, "REB_ACTIVATE_PIN de setar pino GPIO_PIN_3 -: %d \r\n", value);
		HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);

		set_pin_status(value, REB_ACTIVATE_PIN);
	}
	if (GPIO_Pin == GPIO_PIN_5) {
		uint8_t value = 1U;

		sprintf(msg, "REB_DEACTIVATE_PIN de setar pino GPIO_PIN_5 -: %d \r\n", value);
		HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);

		set_pin_status(value, REB_DEACTIVATE);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	sprintf(msg, "StartDefaultTask\r\n");
	HAL_UART_Transmit(&huart1, &msg, strlen(msg), HAL_MAX_DELAY);
	UBaseType_t stack_left = uxTaskGetStackHighWaterMark(NULL);
	sprintf(msg, "Task stack left: %u words\r\n", (unsigned int)stack_left);
	HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
	/* Infinite loop */
	for (;;) {
		hazard_lights_blink();
		osDelay(0);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskTCU */
/**
 * @brief Function implementing the tcu_handler thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskTCU */
void StartTaskTCU(void const * argument)
{
  /* USER CODE BEGIN StartTaskTCU */
	/* Infinite loop */
	for (;;) {
		sprintf(msg, "binbou TCU TASK \r\n");
		HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
		UBaseType_t stack_left = uxTaskGetStackHighWaterMark(NULL);
		sprintf(msg, "Task stack left: %u words\r\n", (unsigned int)stack_left);
		HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
		monitor_tcu();
		osDelay(0);
	}
  /* USER CODE END StartTaskTCU */
}

/* USER CODE BEGIN Header_StartTaskconn */
/**
 * @brief Function implementing the conn_handler thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskconn */
void StartTaskconn(void const * argument)
{
  /* USER CODE BEGIN StartTaskconn */
	/* Infinite loop */
	for (;;) {
		sprintf(msg, "binbou CONN task \r\n");
		HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
		UBaseType_t stack_left = uxTaskGetStackHighWaterMark(NULL);
		sprintf(msg, "Task stack left: %u words\r\n", (unsigned int)stack_left);
		HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
		check_can_communication();
		osDelay(0);
	}
  /* USER CODE END StartTaskconn */
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
