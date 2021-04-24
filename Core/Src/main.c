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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_up;

/* USER CODE BEGIN PV */
//extern uint8_t buffer[];
//extern uint32_t NewDataFromUsb;//0-no new data, <>0 new data, value=New Data lenght
//extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE]; //Data that comes from USB
extern uint16_t aOutputWave [BUFFER_SIZE];
extern uint8_t retWave;
extern uint8_t aConfig[8];//array for configuration data
uint16_t uFrqSP; // set point for Frequency
uint16_t VppSP; // set point for Amplitude
uint8_t uOffsSP; // set point for Offset
uint8_t uPwmSP=50; // set point for PWM on SQUARE Wave
enum SamplesPerWave  {SPW360, SPW180, SPW90, SPW45, SPW24}; // Samples per wave
enum AmpPower { x2_0, x1_5, x1_0, x0_5 }; // Amplification
enum SamplesPerWave eSPW = SPW360;
enum AmpPower eAmpPow;

/*static const uint16_t aSRC_Const_Buffer[BUFFER_SIZE] =
{
  0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F10,
  0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E, 0x1F20,
  0x2122, 0x2324, 0x2526, 0x2728, 0x292A, 0x2B2C, 0x2D2E, 0x2F30,
  0x3132, 0x3334, 0x3536, 0x3738, 0x393A, 0x3B3C, 0x3D3E, 0x3F40,
  0x4142, 0x4344, 0x4546, 0x4748, 0x494A, 0x4B4C, 0x4D4E, 0x4F50,
  0x5152, 0x5354, 0x5556, 0x5758, 0x595A, 0x5B5C, 0x5D5E, 0x5F60,
  0x6162, 0x6364, 0x6566, 0x6768, 0x696A, 0x6B6C, 0x6D6E, 0x6F70,
  0x7172, 0x7374, 0x7576, 0x7778, 0x797A, 0x7B7C, 0x7D7E, 0x7F80
};*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void TransferComplete(DMA_HandleTypeDef *hdma_tim6_up);
static void TransferError(DMA_HandleTypeDef *hdma_tim6_up);
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
  MX_TIM1_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /*Square wave generation and put it into aOutputWave */
  // as default wave on power up
  for (int i = 0; i < BUFFER_SIZE; ++i)
  {
	  if (i < BUFFER_SIZE/2) aOutputWave[i]=0;
	  else aOutputWave[i]=0xFFFF;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  HAL_Delay(500);
	  //  uint8_t HiMsg[] = "hello\r\n";
	    if(retWave==1)
	    {
	    	// b[i] = (a[j] << 8) | a[j + 1];
	        // set point for Frequency
	    	uFrqSP = (aConfig[0] << 8) | aConfig[1];
	        // set point for Amplitude
	    	VppSP = (aConfig[2] << 8) | aConfig[3];
	        // set point for Offset
	    	uOffsSP = aConfig[4];
	        // set point for PWM on SQUARE Wave
	    	uPwmSP = aConfig[5];
	        // Samples per wave
	    	if (eSPW != aConfig[6])
	    	{
	    		uint32_t BufLnght;
	    		HAL_DMA_Abort_IT(htim1.hdma[TIM_DMA_ID_UPDATE]);
	    		eSPW = aConfig[6];
	    		if (eSPW == SPW360) BufLnght = 360;

	    		if (eSPW == SPW180)
	    		{
	    			for(int i =0;i<180; ++i)
	    				aOutputWave[i] = aOutputWave[2*i];
	    			BufLnght = 180;
	    		}

	    		if (eSPW == SPW90)
	    		{
	    			 for(int i =0;i<90; ++i)
	    			    aOutputWave[i] = aOutputWave[4*i];
	    			 BufLnght = 90;
	    		}

	    		if (eSPW == SPW45)
	    		{
	    			 for(int i =0;i<45; ++i)
	    			    aOutputWave[i] = aOutputWave[8*i];
	    			 BufLnght = 45;
	    		}

	    		if (eSPW == SPW24)
	    		{
	    			 for(int i =0;i<24; ++i)
	    			    aOutputWave[i] = aOutputWave[15*i];
	    			 BufLnght = 24;
	    		}

	    		HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_UPDATE],(uint32_t)&aOutputWave,
	    		  		(uint32_t)&GPIOB->ODR, BufLnght);

	    	}
	        // Amplification
	    	eAmpPow = aConfig[7];

	    	TIM1->ARR = uFrqSP;
	    	//htim1.Init.Period = uFrqSP;
//	    	CDC_Transmit_FS(HiMsg, sizeof(HiMsg));
	    	CDC_Transmit_FS((uint8_t*)aOutputWave, 720);
	    	retWave =0;
	    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
	__HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0; // 1000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100; //7-65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA configuration */
  hdma_tim1_up.Instance = DMA1_Channel5;
  hdma_tim1_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim1_up.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 16 bits
  hdma_tim1_up.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_tim1_up.Init.Mode = DMA_CIRCULAR;//DMA_NORMAL;
  hdma_tim1_up.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&hdma_tim1_up);
//  __HAL_DMA1_REMAP(HAL_DMA1_CH3_TIM6_UP);
  __HAL_LINKDMA(&htim1,hdma[TIM_DMA_ID_UPDATE],hdma_tim1_up);
  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* Select Callbacks functions called after Transfer complete and Transfer error */
  HAL_DMA_RegisterCallback(&hdma_tim1_up, HAL_DMA_XFER_CPLT_CB_ID, TransferComplete);
  HAL_DMA_RegisterCallback(&hdma_tim1_up, HAL_DMA_XFER_ERROR_CB_ID, TransferError);
  /* (Callbacks for DMA IRQs) */
//  htim1->hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = data_tramsmitted_handler;
//  htim1->hdma[TIM_DMA_ID_UPDATE]->XferErrorCallback = transmit_error_handler;
  /*  (Enable DMA) */
  HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_UPDATE],(uint32_t)&aOutputWave,
  		(uint32_t)&GPIOB->ODR, BUFFER_SIZE);
  /*  (Enable TIM for DMA events) */
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);

  /*  (Run TIM) */
  __HAL_TIM_ENABLE(&htim1);
  __HAL_RCC_TIM1_CLK_DISABLE();//stop clocking interrupt timer
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DACA_0_Pin|DACA_1_Pin|DACA_2_Pin|DACA_3_Pin
                          |DACA_4_Pin|DACA_5_Pin|DACA_6_Pin|DACA_7_Pin
                          |LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DACB_0_Pin|DACB_1_Pin|DACB_2_Pin|DACB_10_Pin
                          |DACB_11_Pin|DACB_12_Pin|DACB_13_Pin|DACB_14_Pin
                          |DACB_15_Pin|DACB_3_Pin|DACB_4_Pin|DACB_5_Pin
                          |DACB_6_Pin|DACB_7_Pin|DACB_8_Pin|DACB_9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_0_Pin */
  GPIO_InitStruct.Pin = LED_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DACA_0_Pin DACA_1_Pin DACA_2_Pin DACA_3_Pin
                           DACA_4_Pin DACA_5_Pin DACA_6_Pin DACA_7_Pin */
  GPIO_InitStruct.Pin = DACA_0_Pin|DACA_1_Pin|DACA_2_Pin|DACA_3_Pin
                          |DACA_4_Pin|DACA_5_Pin|DACA_6_Pin|DACA_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DACB_0_Pin DACB_1_Pin DACB_2_Pin DACB_10_Pin
                           DACB_11_Pin DACB_12_Pin DACB_13_Pin DACB_14_Pin
                           DACB_15_Pin DACB_3_Pin DACB_4_Pin DACB_5_Pin
                           DACB_6_Pin DACB_7_Pin DACB_8_Pin DACB_9_Pin */
  GPIO_InitStruct.Pin = DACB_0_Pin|DACB_1_Pin|DACB_2_Pin|DACB_10_Pin
                          |DACB_11_Pin|DACB_12_Pin|DACB_13_Pin|DACB_14_Pin
                          |DACB_15_Pin|DACB_3_Pin|DACB_4_Pin|DACB_5_Pin
                          |DACB_6_Pin|DACB_7_Pin|DACB_8_Pin|DACB_9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB_MODE_Pin PB_UP_Pin PB_DOWN_Pin */
  GPIO_InitStruct.Pin = PB_MODE_Pin|PB_UP_Pin|PB_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*  (DMA IRQ callbacks) */
void TransferComplete(DMA_HandleTypeDef *hdma)
{
    /* Stop timer */
    __HAL_TIM_DISABLE(&htim1);
    /* Reconfigure DMA */
    HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_UPDATE],(uint32_t)&aOutputWave,
    		(uint32_t)&GPIOB->ODR, BUFFER_SIZE);
    /* Start timer for new data transmit */
    __HAL_TIM_ENABLE(&htim1);
}

void TransferError(DMA_HandleTypeDef *hdma)
{
    /* Stop timer */
    __HAL_TIM_DISABLE(&htim1);
    /* !!! Some error handle for future implementation */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
