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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "myLib.h"
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

/* Buffer that store USART data via DMA */

volatile uint8_t UART_Buffer[UART_RX_BUFFER_SIZE];
volatile uint16_t NumberOfBytesReceive = 0;

uint8_t ucUserString[UART_RX_BUFFER_SIZE];
/* Total Error during init procedure */
static uint32_t xErrorCount = 0;

/* Store pos in string */
static volatile char aPos[5];

/* Use in SUPERLOOP */
uint32_t xTickNow = 0;
uint32_t xTickPrev_LED = 0;
uint32_t xTickPrev_MENU = 0;

/* USART Pointer offset */
static size_t old_pos;
size_t pos;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void vUSART_Check(void);
void vUSART_ProcessData(uint8_t *data, size_t len);
uint8_t ucParseUserString(void);
void vCompareString(uint8_t *str);
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
	MX_USART2_UART_Init();
	offAllLed;
	/* USER CODE BEGIN 2 */
	if (xErrorCount != 0)
	{
		print("Initialize failed\r\n");
		print(&xErrorCount);
	}
	else
	{
		print("Initialize successful\r\n");
	}

	xTickPrev_LED = HAL_GetTick();
	xTickPrev_MENU = HAL_GetTick();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if (HAL_GetTick() - xTickPrev_LED >= defineDELAY_LED)
		{
			toggleLed4;
			xTickPrev_LED = HAL_GetTick();
		}

		if (HAL_GetTick() - xTickPrev_MENU >= defineDELAY_USART_MENU)
		{
			if (ucParseUserString())
			{
				vCompareString((uint8_t *)ucUserString);
			}
			xTickPrev_MENU = HAL_GetTick();
			/* USER CODE BEGIN 3 */
		}
		/* USER CODE END 3 */
	}
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
	uint32_t xStatus = 0;
	/* USER CODE BEGIN USART2_Init 0 */
	// LL_DMA_InitTypeDef DMA_TX_Handle = {0};
	LL_DMA_InitTypeDef DMA_RX_Handle = {0};
	/* USER CODE END USART2_Init 0 */
	LL_USART_InitTypeDef USART_InitStruct = {0};
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	/**USART2 GPIO Configuration
	 PA2   ------> USART2_TX
	 PA3   ------> USART2_RX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	xStatus = LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	if (xStatus != SUCCESS)
		xErrorCount++;

	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART2 DMA Init */
	/* Configure DMA for USART RX */

	LL_DMA_StructInit(&DMA_RX_Handle);
	DMA_RX_Handle.MemoryOrM2MDstAddress = (uint32_t)UART_Buffer;
	DMA_RX_Handle.PeriphOrM2MSrcAddress = (uint32_t)&USART2->DR;
	DMA_RX_Handle.NbData = UART_RX_BUFFER_SIZE;
	DMA_RX_Handle.Priority = LL_DMA_PRIORITY_VERYHIGH;
	DMA_RX_Handle.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	DMA_RX_Handle.Mode = LL_DMA_MODE_CIRCULAR;
	DMA_RX_Handle.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_RX_Handle.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
	DMA_RX_Handle.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
	DMA_RX_Handle.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
	xStatus = LL_DMA_Init(DMA1, LL_DMA_CHANNEL_6, &DMA_RX_Handle);
	if (xStatus != SUCCESS)
		xErrorCount++;

	/* Enable DMA1 Channel6 Tranmission Complete Interrupt DMA_CCR_TCIE & DMA_CCR_HTIE*/
	/* Enable HT & TC interrupts */
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_6);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);

	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

	/* Configure DMA for USART TX */
	// LL_DMA_StructInit(&DMA_TX_Handle);
	// DMA_TX_Handle.MemoryOrM2MDstAddress  = (uint32_t)UART_Buffer;
	// DMA_TX_Handle.PeriphOrM2MSrcAddress  = (uint32_t)&USART2->DR;
	// DMA_TX_Handle.NbData                 = UART_RX_BUFFER_SIZE;
	// DMA_TX_Handle.Priority               = LL_DMA_PRIORITY_VERYHIGH;
	// DMA_TX_Handle.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	// DMA_TX_Handle.Mode                   = LL_DMA_MODE_NORMAL;
	// DMA_TX_Handle.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
	// DMA_TX_Handle.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
	// DMA_TX_Handle.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
	// DMA_TX_Handle.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
	/* Enable DMA1 Channel7 Tranmission Complete Interrupt */
	// LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);
	// xStatus = LL_DMA_Init(DMA1, LL_DMA_CHANNEL_7, &DMA_TX_Handle);
	// if(xStatus != SUCCESS) xErrorCount++;
	/* DMA1_Channel7_IRQn interrupt configuration */
	// HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 2, 0);
	// HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	/* USER CODE BEGIN USART2_Init 1 */
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	xStatus = LL_USART_Init(USART2, &USART_InitStruct);
	if (xStatus != SUCCESS)
		xErrorCount++;

	LL_USART_ConfigAsyncMode(USART2);
	/* Enable RX DMA Request USART_CR3_DMAR*/
	LL_USART_EnableDMAReq_RX(USART2);
	/* Enable IDLE Interrupt USART_CR1_IDLEIE*/
	LL_USART_EnableIT_IDLE(USART2);
	// /* Enable TX DMA Request USART_CR3_DMAT*/
	// LL_USART_EnableDMAReq_TX(USART2);
	/* USART2 interrupt Init */
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	/* Set bit USART_CR1_UE */
	LL_USART_Enable(USART2);
	/* Enable DMA USART RX Stream DMA_CCR_EN*/
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);
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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, USER_LED_3_Pin | USER_LED_2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, USER_LED_1_Pin | USER_LED_4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : BT_UP_Pin BT_CENTER_Pin BT_DOWN_Pin */
	GPIO_InitStruct.Pin = BT_UP_Pin | BT_CENTER_Pin | BT_DOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : USER_LED_3_Pin USER_LED_2_Pin */
	GPIO_InitStruct.Pin = USER_LED_3_Pin | USER_LED_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : USER_LED_1_Pin USER_LED_4_Pin */
	GPIO_InitStruct.Pin = USER_LED_1_Pin | USER_LED_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void vUSART_Check(void)
{
	toggleLed1;

	/* Calculate current position in buffer */
	pos = UART_RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6);
}

/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void vUSART_ProcessData(uint8_t *data, size_t len)
{
	const uint8_t *d = data;
	uint32_t ulBlockTime = 10000;
	/*
	 * This function is called on DMA TC and HT events, as well as on UART IDLE (if enabled) line event.
	 *
	 * For the sake of this example, function does a loop-back data over UART in polling mode.
	 * Check ringbuff RX-based example for implementation with TX & RX DMA transfer.
	 */

	for (; len > 0; --len, ++d)
	{
		LL_USART_TransmitData8(USART2, (uint8_t)*d);
		while (!LL_USART_IsActiveFlag_TC(USART2))
		{
			if ((ulBlockTime--) == 0)
				break;
		}
	}
	LL_USART_TransmitData8(USART2, (uint8_t)13);
	LL_USART_TransmitData8(USART2, (uint8_t)10);
}

/**
 * @brief Get string from user into a buffer 
 * 
 */
uint8_t ucParseUserString(void)
{
	uint32_t index = 0, i;
	uint8_t isNewString = 0;
	if (pos != old_pos)
	{ /* Check change in received data */
		printVar(pos);
		endln;
		isNewString = 1;
		memset(ucUserString, 0, UART_RX_BUFFER_SIZE);
		if (pos > old_pos)
		{ /* Current position is over previous one */
			/* We are in "linear" mode */
			/* Process data directly by subtracting "pointers" */
			// print("User string (pos > old_pos): ");
			// vUSART_ProcessData((uint8_t *)(UART_Buffer + old_pos),
			// 				   pos - old_pos);
			// endln;

			for (i = old_pos; i < pos; i++)
			{
				ucUserString[index] = UART_Buffer[i];
				index++;
			}
		}
		else
		{
			/* We are in "overflow" mode */
			/* First process data to the end of buffer */

			// print("User string (pos < old_pos): ");
			// vUSART_ProcessData((uint8_t *)(UART_Buffer + old_pos),
			// 				   UART_RX_BUFFER_SIZE - old_pos);
			// vUSART_ProcessData((uint8_t *)(UART_Buffer + 0), pos);
			// endln;

			for (i = old_pos; i < UART_RX_BUFFER_SIZE; i++)
			{
				ucUserString[index] = UART_Buffer[i];
				index++;
			}
			for (i = 0; i < pos; i++)
			{
				ucUserString[index] = UART_Buffer[i];
				index++;
			}
			endln;
		}
		old_pos = pos; /* Save current position as old */
	}
	/* Check and manually update if we reached end of buffer */
	if (old_pos == UART_RX_BUFFER_SIZE)
	{
		endln;
		old_pos = 0;
	}
	return isNewString;
}

/**
 * @brief Compare user string with predefined string 
 * 
 * @param str 
 * @return int8_t value: 1 if src > des, 0 if src == des, -1 if src < des
 */
void vCompareString(uint8_t *str)
{
	if (IsString(str, "led3 1\r"))
	{
		onLed3;
		return;
	}
	if (IsString(str, "led3 0\r"))
	{
		offLed3;
		return;
	}
	if (IsString(str, "led2 1\r"))
	{

		onLed2;
		return;
	}
	if (IsString(str, "led2 0\rn"))
	{
		offLed2;
		return;
	}
	if (IsString(str, "leds 1\r"))
	{
		onAllLed;
		return;
	}
	if (IsString(str, "leds 0\r"))
	{
		offAllLed;
		return;
	}
	if (IsString(str, "help\r"))
	{
		print("---------HELP MENU---------\r\n");
		print("leds <state>: control state of all leds\r\n");
		print("led<x> <state>: control state of led x\r\n");
		print("1: ON, 0: OFF\r\n\r\n");
		return;
	}
	print("Unknown Command\r\n");
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM4)
	{
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

#ifdef USE_FULL_ASSERT
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
     tex: print("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
