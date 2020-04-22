/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/**
  * Accelerometer + Magnetometer sensor - LSM303AGR
  * - I2C ADDw = 3Ch (Mag)
  * - I2C ADDw = 32h (Acc)
  * Relative humidity and temperature sensor - HTS221
  * - I2C ADDw = BEh
  * Pressure sensor - LPS22HB
  * - I2C ADDw = BAh
  */
/**
  * PA8 could be set to GPIO output push/pull to signal WKUP1 on the slave
  * - WKUP1 is input pull down according to the doc
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;
osThreadId readWireHandle;
osThreadId writeDebugTaskHandle;
osMutexId wireMutexHandle;
/* USER CODE BEGIN PV */

volatile unsigned int btn;
volatile unsigned int zw;
volatile unsigned int u1rc;
volatile unsigned int u1hrc;
volatile unsigned int u1tc;
volatile unsigned int u1htc;
volatile unsigned int u1ec;
volatile unsigned int u1ic;
volatile unsigned int u2rc;
volatile unsigned int u2hrc;
volatile unsigned int u2tc;
volatile unsigned int u2htc;
volatile unsigned int u2ec;
volatile unsigned int u2ic;

//unsigned int C1Low[16];
//unsigned int C1High[16];
//unsigned int C2Low[16];
//unsigned int C2High[16];
unsigned int sF[20];
unsigned int sR[20];
unsigned char dbgBuf[256];
unsigned char u2rx[32];
unsigned char u1tx[256];
unsigned char u1rx[64];

volatile uint8_t MagValid;
volatile uint8_t MagValue;
volatile uint8_t AccValid;
volatile uint8_t AccValue;
volatile uint8_t HTSValid;
volatile uint8_t HTSValue;
volatile uint8_t LPSValid;
volatile uint8_t LPSValue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskReadWire(void const * argument);
void StartWriteDebug(void const * argument);

static void MX_NVIC_Init(void);
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

  // !!!!!  Make sure MX_DMA_Init(); comes before the UARTS here below...

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of wireMutex */
  osMutexDef(wireMutex);
  wireMutexHandle = osMutexCreate(osMutex(wireMutex));

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of readWire */
  osThreadDef(readWire, StartTaskReadWire, osPriorityHigh, 0, 512);
  readWireHandle = osThreadCreate(osThread(readWire), NULL);

  /* definition and creation of writeDebugTask */
  osThreadDef(writeDebugTask, StartWriteDebug, osPriorityIdle, 0, 512);
  writeDebugTaskHandle = osThreadCreate(osThread(writeDebugTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //HAL_Delay(1000);
	  // ZacWire uses 130 us per cycle (bit), with 1 rest cycle between byte 1 and byte 2
	  // xSemaphoreGiveFromISR()
	  // EXTI_GenerateSWInterrupt()
	  // xTaskNotifyFromISR () - need to cause a reschedule with last parameter
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 32;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Modem_Pin|LD2_Pin|M_WKUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin LSM303AGR_DRDY_Pin LSM303AGR_INT_Pin */
  GPIO_InitStruct.Pin = B1_Pin|LSM303AGR_DRDY_Pin|LSM303AGR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Modem_Pin LD2_Pin M_WKUP_Pin */
  GPIO_InitStruct.Pin = LED_Modem_Pin|LD2_Pin|M_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HTS221_DRDY_Pin LPS22H_INT1_Pin */
  GPIO_InitStruct.Pin = HTS221_DRDY_Pin|LPS22H_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
// examples
#if 0
void powerTaskEntry(__unused void const* argument)
{
	static uint32_t thread_notification;

	while(1)
	{
		/* Sleep until we are notified of a state change by an
		 * interrupt handler. Note the first parameter is pdTRUE,
		 * which has the effect of clearing the task's notification
		 * value back to 0, making the notification value act like
		 * a binary (rather than a counting) semaphore.  */

		thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if(thread_notification)
		{
			system_power_evaluate_state();
		}
	}
}

void system_power_interrupt_handler(uint32_t time)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(time < SYS_RESET_SHUTDOWN_THRESHOLD_MS)
	{
		// Perform a reset of computers
		computer_reset_request_ = RESET_ALL_COMPUTERS;
	}
	else if((time >= SYS_RESET_SHUTDOWN_THRESHOLD_MS) &&
			(time < SYS_RESET_HARD_CUTOFF_THRESHOLD_MS))
	{
		system_power_request_state(SYSTEM_POWER_LOW);
	}
	else
	{
		system_power_request_state(SYSTEM_POWER_OFF_HARD);
	}

	// Notify the thread so it will wake up when the ISR is complete
	vTaskNotifyGiveFromISR(powerTaskHandle,
			&xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void tcinterrupt()
{
	BaseTypet xHigherPriorityTaskWoken = psFALSE;
	/* Stop further interrupts to avoid time period shorter than 100 us. */
	tc_stop_interrupt();
	vTaskNotifyGiveFromISR( xCANTaskHandle, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void vCANTask( void pvParameter )
{
	tc_initialise();
	for( ;; )
	{
		/ Program the TC to issue an interrupt after 100 uS. /
		tc_start_interrupt( 100 );
		/ Will be woken-up by the timer-counter.
		ulTaskNotifyTake( pdTRUE, portMAXDELAY );
		cansend_message();
	}
}

/* The peripheral driver's transmit function. */
void StartTransmission( uint8_t *pcData, size_t xDataLength )
{
	/* At this point xTaskToNotify should be NULL as no transmission
    is in progress.  A mutex can be used to guard access to the
    peripheral if necessary. */
	configASSERT( xTaskToNotify == NULL );

	/* Store the handle of the calling task. */
	xTaskToNotify = xTaskGetCurrentTaskHandle();

	/* Start the transmission - an interrupt is generated when the
    transmission is complete. */
	vStartTransmit( pcData, xDatalength );
}
#endif
/**
  * @brief  Input Capture callback in non blocking mode
  * @param  htim : TIM IC handle
  * @retval None
  */
void
TimerCapture_Ch2_Callback(void)
{
	/* Get the Input Capture value */
	static unsigned int cur = 0;
	unsigned int v1 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
	unsigned int v2 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
	v2 -= v1;
	if (v1 >= 256)
		cur = 0;
	else
		cur += 1;
	if (cur < 20)
	{
		sF[cur] = v1;
		sR[cur] = v2;
	}
	if (cur == 19)
	{
		// we can compute the 2 bytes...
		unsigned int one = sR[0] - 10;
		unsigned int zero = sR[0] + 10;
		int ok = 1;
		for (unsigned int i = 1; i < 20; i++)
		{
			if (sR[i] < one) {
				sR[i] = 1;
			}
			else if (sR[i] < zero)
			{
				if (!(i == 10 && sR[i] > one))
				{
					ok = 0;
					break;
				}
			}
			else
			{
				sR[i] = 0;
			}
		}
		unsigned int b1 = 0;
		unsigned int b2 = 0;
		if (ok)
		{
			unsigned int cnt = 0;
			for (unsigned int i = 1; i < 9; i++)
			{
				b1 = (b1 << 1) | sR[i];
				cnt += sR[i];
			}
			if ((cnt & 1) != sR[9])
				ok = 0;
			cnt = 0;
			for (unsigned int i = 11; i < 19; i++)
			{
				b2 = (b2 << 1) | sR[i];
				cnt += sR[i];
			}
			if ((cnt & 1) != sR[19])
				ok = 0;
		}
		if (ok)
		{
			zw = (b1 << 8) | b2;
		}
	}
#if 0
	if (v1 < 256)
	{
		v1 >>= 4;
		C1Low[v1] += 1;
	}
	else
	{
		v1 >>= 12;
		C1High[v1] += 1;
	}
	if (v2 < 256)
	{
		v2 >>= 4;
		C2Low[v2] += 1;
	}
	else
	{
		v2 >>= 12;
		C2High[v2] += 1;
	}
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (huart->Instance == USART1)
	{
		u1tc += 1;
		/* Notify the task that the transmission is complete. */
    vTaskNotifyGiveFromISR(writeDebugTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	if (huart->Instance == USART2)
	{
		u2tc += 1;
    /* Notify the task that the transmission is complete. */
    vTaskNotifyGiveFromISR(writeDebugTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
		u1htc += 1;
	if (huart->Instance == USART2)
		u2htc += 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
		u1rc += 1;
	if (huart->Instance == USART2)
		u2rc += 1;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
		u1hrc += 1;
	if (huart->Instance == USART2)
		u2hrc += 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
		u1ec += 1;
	if (huart->Instance == USART2)
		u2ec += 1;
}

int UART_Receive(unsigned char *dest, const unsigned char *rx, UART_HandleTypeDef *huart, unsigned int *uxcc, const unsigned int max)
{
	unsigned int cc = __HAL_DMA_GET_COUNTER(huart->hdmarx);
	if (*uxcc != cc)
	{
		HAL_UART_DMAPause(huart);
  	int len = 0;
		if (cc > *uxcc)
		{
			for (unsigned int i = max - *uxcc; i < max; i++)
				dest[len++] = rx[i];
			for (unsigned int i = 0; i < max - cc; i++)
				dest[len++] = rx[i];
		}
		else
		{
			for (unsigned int i = max - *uxcc; i < max - cc; i++)
				dest[len++] = rx[i];
		}
		HAL_UART_DMAResume(huart);
  	*uxcc = cc;
  	return len;
	}
	return 0;
}

static int isRetOk(const unsigned char *s, int len, const char *r)
{
	int rl = strlen(r) - 1;
	for (unsigned int i = 0; i + rl < len; i++)
		if (memcmp(s + i, r, rl) == 0)
			return 1;
	return 0;
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
//	unsigned int cur = 0;
//	char buf[256];
  /* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		osDelay(1000);
	}
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTaskReadWire */
/**
* @brief Function implementing the readWire thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskReadWire */
void StartTaskReadWire(void const * argument)
{
  /* USER CODE BEGIN StartTaskReadWire */
	HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  /* Infinite loop */
	for(;;)
	{
  	uint8_t addrBuf[1];
  	uint8_t dataBuf[1];
  	HAL_StatusTypeDef res;
    // Accelerometer + Magnetometer sensor - LSM303AGR
    // - I2C ADDw = 3Ch (Mag)
  	// WHO_AM_I_M R 4F 01001111 01000000
  	addrBuf[0] = 0x4F;
		res = HAL_I2C_Master_Transmit(&hi2c1, 0x3C, addrBuf, 1, 40);
		if (res != HAL_OK)
			MagValid = 0;
		else
		{
			res = HAL_I2C_Master_Receive(&hi2c1, 0x3C, dataBuf, 1, 60);
			if (res != HAL_OK)
				MagValid = 0;
			else
			{
				MagValid = 1;
				MagValue = dataBuf[0];
			}
		}
    // - I2C ADDw = 32h (Acc)
  	// WHO_AM_I_A R 0F 000 1111 00110011
  	addrBuf[0] = 0x0F;
		res = HAL_I2C_Master_Transmit(&hi2c1, 0x32, addrBuf, 1, 40);
		if (res != HAL_OK)
			AccValid = 0;
		else
		{
			res = HAL_I2C_Master_Receive(&hi2c1, 0x32, dataBuf, 1, 60);
			if (res != HAL_OK)
				AccValid = 0;
			else
			{
				AccValid = 1;
				AccValue = dataBuf[0];
			}
		}
    // Relative humidity and temperature sensor - HTS221
    // - I2C ADDw = BEh
		// WHO_AM_I R 0F BC
		res = HAL_I2C_Master_Transmit(&hi2c1, 0xBE, addrBuf, 1, 40);
		if (res != HAL_OK)
			HTSValid = 0;
		else
		{
			res = HAL_I2C_Master_Receive(&hi2c1, 0xBE, dataBuf, 1, 60);
			if (res != HAL_OK)
				HTSValid = 0;
			else
			{
				HTSValid = 1;
				HTSValue = dataBuf[0];
			}
		}
    // Pressure sensor - LPS22HB
    // - I2C ADDw = BAh
		// WHO_AM_I R 0F 10110001
		res = HAL_I2C_Master_Transmit(&hi2c1, 0xBA, addrBuf, 1, 40);
		if (res != HAL_OK)
			LPSValid = 0;
		else
		{
			res = HAL_I2C_Master_Receive(&hi2c1, 0xBA, dataBuf, 1, 60);
			if (res != HAL_OK)
				LPSValid = 0;
			else
			{
				LPSValid = 1;
				LPSValue = dataBuf[0];
			}
		}
		osDelay(1000);
	}
  /* USER CODE END StartTaskReadWire */
}

/* USER CODE BEGIN Header_StartWriteDebug */
/**
* @brief Function implementing the writeDebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWriteDebug */
void StartWriteDebug(void const * argument)
{
  /* USER CODE BEGIN StartWriteDebug */
	enum modemStatus { INIT = 0, WAIT, READY, BROKEN } ms = INIT;
	uint32_t ulNotificationValue;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 2000 );
	TickType_t lastSend = uwTick;
	HAL_StatusTypeDef res;
  /* Infinite loop */
	unsigned int cur = 0;
	HAL_UART_Receive_DMA(&huart1, u1rx, 64);
	unsigned int u1cc = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	HAL_UART_Receive_DMA(&huart2, u2rx, 32);
	unsigned int u2cc = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
	// FIXME - so far I failed to use IDLE interrupts to know when receive transmission is done
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	// wait a few seconds to give the LoRa modem a chance to get ready
  osDelay(10000);
  for(;;)
  {
  	long int temp = (long int) zw * 2000 / 2047 - 500;
  	int len = snprintf((char *) dbgBuf, 256, "\r\nuwTick = %lu cur = %u btn = %u zw = %u T = %u.%u\r\n", uwTick, cur++, btn, zw, (unsigned int) temp / 10, (unsigned int) temp % 10);
  	// HAL_UART_Transmit(&huart2, dbgBuf, len, xMaxBlockTime);
  	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
  	/* Could specify which task to wake up, but for now it is fixed. */
  	// StartTransmission( ucDataToTransmit, xDataLength );
  	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
  	if(ulNotificationValue != 1)
  	{
  		/* The call to ulTaskNotifyTake() timed out. */
  		HAL_UART_DMAStop(&huart2);
  		len = snprintf((char *) dbgBuf, 256, "ulNotificationValue = %lu - seems bad\r\n", ulNotificationValue);
  		HAL_UART_Transmit(&huart2, dbgBuf, len, xMaxBlockTime);
  	}
  	len = snprintf((char *) dbgBuf, 256, "u1rc = %u u1hrc = %u u1tc = %u u1htc = %u u1ec = %u u1ic = %u u1cc = %u\r\n", u1rc, u1hrc, u1tc, u1htc, u1ec, u1ic, u1cc);
  	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
  	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
  	len = snprintf((char *) dbgBuf, 256, "u2rc = %u u2hrc = %u u2tc = %u u2htc = %u u2ec = %u u2ic = %u u2cc = %u\r\n", u2rc, u2hrc, u2tc, u2htc, u2ec, u2ic, u2cc);
  	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
  	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
  	unsigned int cc = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
  	if (u2cc != cc)
  	{
  		HAL_UART_DMAPause(&huart2);
  		len = snprintf((char *) dbgBuf, 256 - len, "%s", "Received : \"");
  		if (cc > u2cc)
  		{
  			for (unsigned int i = 32 - u2cc; i < 32; i++)
  				len += snprintf((char *) dbgBuf + len, 256 - len, "%c", u2rx[i]);
  			for (unsigned int i = 0; i < 32 - cc; i++)
  				len += snprintf((char *) dbgBuf + len, 256 - len, "%c", u2rx[i]);
  		}
  		else
  		{
  			for (unsigned int i = 32 - u2cc; i < 32 - cc; i++)
  				len += snprintf((char *) dbgBuf + len, 256 - len, "%c", u2rx[i]);
  		}
  		len += snprintf((char *) dbgBuf + len, 256 - len, "%s", "\"\r\n");
  		HAL_UART_DMAResume(&huart2);
    	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	u2cc = cc;
  	}
#if 0
  	int i;
  	len = 0;
  	for (i = 0; i < 16; i++)
  		len += snprintf((char *) dbgBuf + len, 256 - len, "%s%u%s", (i > 0) ? " " : "C1Low : ", C1Low[i], (i == 15) ? "\r\n" : "");
  	HAL_UART_Transmit(&huart2, dbgBuf, len, 65536);
  	len = 0;
  	for (i = 0; i < 16; i++)
  		len += snprintf((char *) dbgBuf + len, 256 - len, "%s%u%s", (i > 0) ? " " : "C1High : ", C1High[i], (i == 15) ? "\r\n" : "");
  	HAL_UART_Transmit(&huart2, dbgBuf, len, 65536);
  	len = 0;
  	for (i = 0; i < 16; i++)
  		len += snprintf((char *) dbgBuf + len, 256 - len, "%s%u%s", (i > 0) ? " " : "C2Low : ", C2Low[i], (i == 15) ? "\r\n" : "");
  	HAL_UART_Transmit(&huart2, dbgBuf, len, 65536);
  	len = 0;
  	for (i = 0; i < 16; i++)
  		len += snprintf((char *) dbgBuf + len, 256 - len, "%s%u%s", (i > 0) ? " " : "C2High : ", C2High[i], (i == 15) ? "\r\n" : "");
  	HAL_UART_Transmit(&huart2, dbgBuf, len, 65536);
  	len = 0;
  	for (i = 0; i < 20; i++)
  		len += snprintf((char *) dbgBuf + len, 256 - len, "%s%u%s", (i > 0) ? " " : "sF : ", sF[i], (i == 19) ? "\r\n" : "");
  	HAL_UART_Transmit(&huart2, dbgBuf, len, 65536);
  	len = 0;
  	for (i = 0; i < 20; i++)
  		len += snprintf((char *) dbgBuf + len, 256 - len, "%s%u%s", (i > 0) ? " " : "sR : ", sR[i], (i == 19) ? "\r\n" : "");
  	HAL_UART_Transmit(&huart2, dbgBuf, len, 65536);
#endif
  	len = 0;
		if (MagValid)
			len += snprintf((char *) dbgBuf + len, 256 - len, "Mag: %02X(%s) ", MagValue, MagValue == 0x40 ? "Ok" : "??");
		else
			len += snprintf((char *) dbgBuf + len, 256 - len, "Mag failed ");
		if (AccValid)
			len += snprintf((char *) dbgBuf + len, 256 - len, "Acc: %02X(%s) ", AccValue, AccValue == 0x33 ? "Ok" : "??");
		else
			len += snprintf((char *) dbgBuf + len, 256 - len, "Acc failed ");
		if (HTSValid)
			len += snprintf((char *) dbgBuf + len, 256 - len, "HTS: %02X(%s) ", HTSValue, HTSValue == 0xBC ? "Ok" : "??");
		else
			len += snprintf((char *) dbgBuf + len, 256 - len, "HTS failed ");
		if (LPSValid)
			len += snprintf((char *) dbgBuf + len, 256 - len, "LPS: %02X(%s)\r\nUART1 says :\r\n\r\n", LPSValue, LPSValue == 0xB1 ? "Ok" : "??");
		else
			len += snprintf((char *) dbgBuf + len, 256 - len, "LPS failed\r\n");
		HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
  	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
  	if (ms == INIT)
  	{
    	len = snprintf((char *) u1tx, 256, "%s\r", "AT");
  		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	//res = HAL_UART_Transmit(&huart1, (unsigned char *) "AT\r", 3, xMaxBlockTime);
  		len = 0;
  		if (res != HAL_OK || ulNotificationValue != 1)
  		{
  			ms = BROKEN;
  		}
  		else
  		{
  			osDelay(100);
  			len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
  			// see if we got OK
  			if (!isRetOk(dbgBuf, len, "OK\r"))
  	  	{
  	  		ms = BROKEN;
  	  	}
  		}
  		if (ms == INIT)
  		{
      	len = snprintf((char *) u1tx, 256, "AT+APPEUI=%s\r", AppEUI);
    		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
      	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    		len = 0;
    		if (res != HAL_OK || ulNotificationValue != 1)
    		{
    			ms = BROKEN;
    		}
    		else
    		{
    			osDelay(100);
    			len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
    			// see if we got OK
    			if (!isRetOk(dbgBuf, len, "OK\r"))
    	  	{
    	  		ms = BROKEN;
    	  	}
    		}
  		}
  		if (ms == INIT)
  		{
      	len = snprintf((char *) u1tx, 256, "AT+AK=%s\r", AppKey);
    		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
      	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    		len = 0;
    		if (res != HAL_OK || ulNotificationValue != 1)
    		{
    			ms = BROKEN;
    		}
    		else
    		{
    			osDelay(100);
    			len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
    			// see if we got OK
    			if (!isRetOk(dbgBuf, len, "OK\r"))
    	  	{
    	  		ms = BROKEN;
    	  	}
    		}
  		}
  		if (ms == INIT)
  		{
      	len = snprintf((char *) u1tx, 256, "AT+ADDR=%s\r", DevAddr);
    		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
      	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    		len = 0;
    		if (res != HAL_OK || ulNotificationValue != 1)
    		{
    			ms = BROKEN;
    		}
    		else
    		{
    			osDelay(100);
    			len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
    			// see if we got OK
    			if (!isRetOk(dbgBuf, len, "OK\r"))
    	  	{
    	  		ms = BROKEN;
    	  	}
    		}
  		}
  		if (ms == INIT)
  		{
      	len = snprintf((char *) u1tx, 256, "AT+DR=%u\r", DataRate);
    		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
      	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    		len = 0;
    		if (res != HAL_OK || ulNotificationValue != 1)
    		{
    			ms = BROKEN;
    		}
    		else
    		{
    			osDelay(100);
    			len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
    			// see if we got OK
    			if (!isRetOk(dbgBuf, len, "OK\r"))
    	  	{
    	  		ms = BROKEN;
    	  	}
    		}
  		}
  		if (ms == INIT)
  		{
      	len = snprintf((char *) u1tx, 256, "AT+JOIN=%u\r", 1);
    		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
      	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    		len = 0;
    		if (res != HAL_OK || ulNotificationValue != 1)
    		{
    			ms = BROKEN;
    		}
    		else
    		{
    			osDelay(100);
    			lastSend = uwTick;
    			len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
    			// see if we got OK
    			if (!isRetOk(dbgBuf, len, "OK\r"))
    	  	{
    	  		ms = BROKEN;
    	  	}
    			else
    			{
    				if (isRetOk(dbgBuf, len, "+JoinAccepted"))
    					ms = READY;
    				else
    					ms = WAIT;
    			}
    		}
  		}
     	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
  	}
  	if (ms == WAIT)
  	{
  		// should timeout after a while
  		len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
			if (isRetOk(dbgBuf, len, "+JoinAccepted"))
				ms = READY;
			else
			{
				len = 9;
				memcpy(dbgBuf,"WAITING\r\n",len);
				if (uwTick - lastSend > pdMS_TO_TICKS( 60000 ))
					ms = BROKEN;
			}
	   	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
	  	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
  	}
  	// FIXME - Nothing reads the modem output when the time is not elapsed
  	// Could check for received data, e.g. :
  	// # +RCV=99,4,22334400
  	if (ms == READY && uwTick - lastSend > pdMS_TO_TICKS( 30000 ))
  	{
  		len = snprintf((char *) u1tx, 256, "AT+JSTA\r");
  		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
  		ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
  		osDelay(100);
  		len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
  		if (isRetOk(dbgBuf, len, "1\r\nOK\r"))
  		{
  			// LPP_TEMPERATURE         103     // 2 bytes, 0.1°C signed
  	  	HAL_GPIO_TogglePin(LED_Modem_GPIO_Port,LED_Modem_Pin);
    		len = snprintf((char *) u1tx, 256, "AT+SEND=%u,%02X%02X%02X%02X,%u\r",99,1,LPP_TEMPERATURE,(unsigned char) (temp >> 8) & 0xff,(unsigned char) temp & 0xff,0);
    		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
    		ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
      	HAL_GPIO_TogglePin(LED_Modem_GPIO_Port,LED_Modem_Pin);
      	lastSend = uwTick;
    		osDelay(2000);
    		len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
    		if (!isRetOk(dbgBuf, len, "OK\r"))
    			ms = BROKEN;
  		}
  		else
  			ms = BROKEN;
     	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
  	}
  	if (ms == BROKEN)
		{
    	len = snprintf((char *) u1tx, 256, "AT+JSTA\r");
  		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	osDelay(100);
    	len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
     	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	len = snprintf((char *) u1tx, 256, "AT+EUI\r");
  		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	osDelay(100);
    	len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
     	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	len = snprintf((char *) u1tx, 256, "AT+BAND\r");
  		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	osDelay(100);
    	len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
     	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	len = snprintf((char *) u1tx, 256, "AT+VER\r");
  		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	osDelay(100);
    	len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
     	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	len = snprintf((char *) u1tx, 256, "AT+APPEUI\r");
  		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	osDelay(100);
    	len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
     	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	len = snprintf((char *) u1tx, 256, "AT+ADDR\r");
  		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	osDelay(100);
    	len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
     	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	len = snprintf((char *) u1tx, 256, "AT+DR\r");
  		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	osDelay(100);
    	len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
     	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	len = snprintf((char *) u1tx, 256, "AT+DEFMODE\r");
  		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	osDelay(100);
    	len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
     	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	len = snprintf((char *) u1tx, 256, "AT+AK\r");
  		res = HAL_UART_Transmit_DMA(&huart1, u1tx, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    	osDelay(100);
    	len = UART_Receive(dbgBuf, u1rx, &huart1, &u1cc, 64);
     	HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
    	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
		}
  	//HAL_GPIO_TogglePin(LED_Modem_GPIO_Port,LED_Modem_Pin);
    osDelay(10000);
  }
  /* USER CODE END StartWriteDebug */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
