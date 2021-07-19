/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "AD7799.h"
unsigned	char	buf[4] = {0,0,0,0};
AD7799    ad7799[3];
#define   AD7799_GAIN  128					//�������Ϊ64��,�������Ϊ64
#define   AD7799_CHIP_GAIN  AD7799_GAIN_2 	//�������Ϊ64��,�������ΪAD7799_GAIN_64

#define   AD7799_RefmV    3300				//��׼��ѹ 3300mV	
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void tx_delay_100us(uint16_t nus)	//??0-8191us
{
        __HAL_TIM_SetCounter(&htim3, 0);//htim8

        __HAL_TIM_ENABLE(&htim3);

        while(__HAL_TIM_GetCounter(&htim3) < (8 * nus));//????8MHz,8???1us
        /* Disable the Peripheral */
        __HAL_TIM_DISABLE(&htim3);
}
double analyzeAD7799_Data(u32 data)
{
	long value = (data - 0X800000);
	return (float)((float)value*(float)AD7799_RefmV)/(0X800000*AD7799_GAIN);	//0X800000:2.048V    0X000000:0V

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void AD7799_test()
{
	
	u8 CurrentChannelValue=1;		//��ǰѡ������ĸ�ͨ��,����1~2 
	u8 i;
	long lADValue[2];
	u8 ChannelBuf[2]={AD7799_CH_AIN1P_AIN1M,AD7799_CH_AIN2P_AIN2M};		//ͨ��1  ͨ��2
	double ADValues[2];
	char buff[256];
	/*ad7799��ʼ��*/
	AD7799_gpio_init();
	while(!AD7799_Init())
	{
			//LED0 = 0;
			HAL_Delay(50);
	}	
  AD7799_Reset();
	//LED0 = 1;
	// AD7799_Calibrate();

	AD7799_SetGain(AD7799_CHIP_GAIN);		
	// AD7799_SetBurnoutCurren(0);				//�ر�BO
	// AD7799_SetBufMode(0);					//��������Ҫ��ĵ�ѹ����100mV,��������ΪUnbuffered Mode
	// AD7799_SetPolar(1);
	//AD7799_SetChannel(ChannelBuf[0]);		//ͨ������.
	AD7799_SetMode(AD7799_MODE_CONT,5);		//Ĭ��˫����   Ƶ��Ϊ5
	AD7799_SetReference(1);					//�رղο����,��Ϊ���ǵ� AD7799_RefmV �ο���ѹ����0.5V
	
	while(1)
	{
	
			for(i=0;i<2;i++)
			{
		    AD7799_SetChannel(ChannelBuf[i]);//ͨ������.		0~1
				
				HAL_Delay(10);
				AD7799_GetRegisterValue(AD7799_REG_DATA,3);//���֮ǰ��AD
				
				while( !AD7799_Ready())		//1~2
				{
					HAL_Delay(5);
				}
			  lADValue[i]=AD7799_GetRegisterValue(AD7799_REG_DATA,3);//0:ͨ��1 1:ͨ��2
				ADValues[i]=  analyzeAD7799_Data(lADValue[i]);
			}
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
		   // HAL_UART_Transmit(&huart1,(uint8_t *)&lADValue[0],4,1000);
        sprintf(buff, "AD1 = %x ,%.2f mv AD2= %x ,%.2f mv",lADValue[0], ADValues[0],lADValue[1],ADValues[1]);
			  HAL_UART_Transmit(&huart1,(uint8_t *)buff,strlen(buff),1000);
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
		
				
		//	printf("��ǰͨ��Ϊ:%d %.3fmV %.3fmV \r\n",CurrentChannelValue,ADValues[0],ADValues[1]);

//		LED0 =!LED0;
		   HAL_Delay(1000);
	}

}
/* USER CODE END 0 */
/* USER CODE BEGIN 0 */
void AD7799_test2()
{
	
	u8 CurrentChannelValue=1;		//��ǰѡ������ĸ�ͨ��,����1~2 
	u8 i;
	long lADValue[2];
	u8 ChannelBuf[2]={AD7799_CH_AIN1P_AIN1M,AD7799_CH_AIN2P_AIN2M};		//ͨ��1  ͨ��2
	double ADValues[2];
	char buff[256];
	/*ad7799��ʼ��*/
  ad7799[0].channel=0;
  ad7799[0].CSPort=CS1_GPIO_Port;
  ad7799[0].CSPin=CS1_Pin;
  ad7799[0].DIPort=DI1_GPIO_Port;
  ad7799[0].DIPin=DI1_Pin;
  ad7799[0].DOPort=DO1_GPIO_Port;
  ad7799[0].DOPin=DO1_Pin;
  ad7799[0].SCKPort=SCK1_GPIO_Port;
  ad7799[0].SCKPin=SCK1_Pin;
	//AD7799_gpio_init();
  AD7799_Reset2(&ad7799[0]);
	while(AD7799_Init2(&ad7799[0]))
	{
			//LED0 = 0;
			HAL_Delay(50);
	}	
  AD7799_Reset2(&ad7799[0]);
	//LED0 = 1;
	//AD7799_Calibrate();
// AD7799_SetBurnoutCurren(0);				//�ر�BO
	AD7799_SetGain2(&ad7799[0],ad7799[0].gain);		//128λ
  AD7799_SetPolarity2(&ad7799[0],ad7799[0].polarity);//˫����
  // AD7799_SetRate2(&ad7799[0],ad7799[0].rate);//������ 4.7hz
	//AD7799_SetBurnoutCurren2(0);				//�ر�BO
	//AD7799_SetBufMode2(0);					//��������Ҫ��ĵ�ѹ����100mV,��������ΪUnbuffered Mode
	AD7799_SetMode2(&ad7799[0],ad7799[0].mode);		//����ģʽ
	AD7799_SetReference2(&ad7799[0],1);					//�رղο����,��Ϊ���ǵ� AD7799_RefmV �ο���ѹ����0.5V
	
	while(1)
	{
	
			for(i=0;i<2;i++)
			{
		    AD7799_SetChannel2(&ad7799[0],ChannelBuf[i]);//ͨ������.		0~1
				
				HAL_Delay(10);
				AD7799_GetRegisterValue2(&ad7799[0],AD7799_REG_DATA,3);//���֮ǰ��AD
				
				while( !AD7799_Ready2(&ad7799[0]))		//1~2
				{
					HAL_Delay(1);
				}
			  lADValue[i]=AD7799_GetRegisterValue2(&ad7799[0],AD7799_REG_DATA,3);//0:ͨ��1 1:ͨ��2
				ADValues[i]=  analyzeAD7799_Data(lADValue[i]);
			}
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
		   // HAL_UART_Transmit(&huart1,(uint8_t *)&lADValue[0],4,1000);
        sprintf(buff, "AD1 = %x ,%.2f mv AD2= %x ,%.2f mv rate=%d",lADValue[0], ADValues[0],lADValue[1],ADValues[1],1<<ad7799[0].gain);
			  HAL_UART_Transmit(&huart1,(uint8_t *)buff,strlen(buff),1000);
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
		//	printf("��ǰͨ��Ϊ:%d %.3fmV %.3fmV \r\n",CurrentChannelValue,ADValues[0],ADValues[1]);
//		LED0 =!LED0;
		   HAL_Delay(1000);
	}

}
/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t data[3]={1,2,3};
	uint16_t i=0;
	unsigned long lData[2]=0;
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//	AD7799_gpio_init();
//	while(!AD7799_Init())
//	{
//			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
//			HAL_Delay(1000);
//		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
//		 // HAL_UART_Transmit(&huart1,data,3,1000);
//		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
//	}	
  AD7799_test2();
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED2_Pin);
      		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
		 HAL_UART_Transmit(&huart1,data,3,1000);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DI3_Pin|LED1_Pin|LED2_Pin|GPIO_PIN_8
                          |DI2_Pin|CS2_Pin|SCK2_Pin|SCK1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS1_Pin|DI1_Pin|SCK3_Pin|CS3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DI3_Pin LED1_Pin LED2_Pin PA8
                           DI2_Pin CS2_Pin SCK2_Pin */
  GPIO_InitStruct.Pin = DI3_Pin|LED1_Pin|LED2_Pin|GPIO_PIN_8     ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DO3_Pin DO2_Pin */
  GPIO_InitStruct.Pin = DO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SCK1_Pin */
  GPIO_InitStruct.Pin = SCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SCK1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS1_Pin */
  GPIO_InitStruct.Pin = CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DO1_Pin */
  GPIO_InitStruct.Pin = DO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DI1_Pin */
  GPIO_InitStruct.Pin = DI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DI1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCK3_Pin CS3_Pin */
  GPIO_InitStruct.Pin = SCK3_Pin|CS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
