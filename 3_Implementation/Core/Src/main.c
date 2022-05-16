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
#include "stm32f1xx_hal.h"
#include "string.h"
#include "stdio.h"
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
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
uint8_t req_counter = 0;
uint8_t led_no = 0;
//float encconstant=6.6*4;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);

static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
void CAN_Filter_Config(void);
void CAN1_Tx(void);


CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;

float inputval=0,rightlastval=0,leftlastval=0,lastval=0;
float degree=0.0;
int encvalue=0;
int encfeedback=0;
int temp=0,running=0,i=0;
uint16_t count=0;



int main(void)
{

	HAL_Init();


	SystemClock_Config();


	MX_GPIO_Init();
	MX_CAN_Init();

	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	CAN_Filter_Config();


	char msg[200];
	sprintf(msg,"Initializations Success\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

	//uint32_t adcval=0;
	uint16_t count=0;
	//int encfeedback=0;
	if(HAL_CAN_ActivateNotification(&hcan,CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF)!= HAL_OK)
	{
		Error_Handler();
	}

	if( HAL_CAN_Start(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	//int running=1;
	//i=0;
	//Starting Encoder
	HAL_TIM_Encoder_Start(&htim2,   TIM_CHANNEL_1|TIM_CHANNEL_2);
	sprintf(msg,"CAN and Encoder Started.\r\nWaiting for Input..\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
	//float inputval=0,rightlastval=0,leftlastval=100;
	//float degree=0.0;
	//int encvalue=0;
	//int countering=0;
	//int min=4096;
	//int max=0;
	//int values[]={50,25,5,150,125,105};int zindex=0;
	inputval=140;
	while (1)
	{
		memset(msg,'\0',sizeof(msg));  // Clearing the msg buffer
		count=TIM2->CNT;                //Counting the encoder pulse

//		int x=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0);
//		if(x==1)
//		{
//			zindex+=1;
//		}
//		sprintf(msg,"Encoder value: [%d]\r\n",count);
//		HAL_UART_Transmit(&huart2,(uint8_t*)msg,sizeof(msg),0x1000);

		//test!!!!!!!
		temp=0;
			if(inputval!=lastval)
			{
				running=1;
			}
			else
			{
				running=0;
			}
			while(running)
			{
				memset(msg,'\0',sizeof(msg));  // Clearing the msg buffer
				count=TIM2->CNT;               //Counting the encoder pulse
				sprintf(msg,"Encoder value: %d\r\n",count);
				HAL_UART_Transmit(&huart2,(uint8_t*)msg,sizeof(msg),0x1000);
//				sprintf(msg,"Input value: %d\r\n",(int)inputval);
//				HAL_UART_Transmit(&huart2,(uint8_t*)msg,sizeof(msg),0x1000);
				encfeedback=(int)count;

				if(inputval>92 && inputval<178)
				{

					//inputval = CAN and Angle    encvalue || encvalue+1 ||encvalue-1 || encvalue+2 || encvalue-2
					//enc val= inputval *0.583
					degree=inputval;
					encvalue=(int)(degree*0.526);
					sprintf(msg,"Required Encoder Value: %d\r\n",encvalue);
					HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
					if(inputval>=rightlastval)
					{
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
						if(temp==0)
						{
//							sprintf(msg,"Started PWM\r\n");
//							HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
							HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
						}
						if(encfeedback>2 && encfeedback<94 && (encfeedback>=encvalue ))//|| encfeedback>=encvalue+1 || encfeedback>=encvalue-1 || encfeedback>= encvalue+2 ||  encfeedback>=encvalue-2))
						{
							HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
							sprintf(msg,"Required Position attained\r\n");
							HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

							temp=1;
							running=0;
							rightlastval=inputval;
							lastval=inputval;

						}
					}
					else
					{
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
						if(temp==0)
						{
							sprintf(msg,"Started PWM\r\n");
							HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
							HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
						}
						if(encfeedback>0 && encfeedback<46 &&encfeedback<=(encvalue))
						{
							sprintf(msg,"Required Position attained\r\n");
							HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
							HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
							temp=1;
							running=0;
							rightlastval=inputval;
							lastval=inputval;

						}
					}

				}
				else if(inputval>4 && inputval<88)
				{
					degree=inputval;
					encvalue=2560-(int)(degree*0.526);
					sprintf(msg,"Required Encoder Value: %d\r\n",encvalue);
					HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
					if(inputval>=leftlastval)
					{
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
						if(temp==0)
						{
							HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
						}
						if(encfeedback<2560 && encfeedback >2514 && encfeedback<=encvalue)
						{
							sprintf(msg,"Required Position attained\r\n");
							HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
							HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
							temp=1;
							running=0;
							leftlastval=inputval;
							lastval=inputval;

						}
					}
					else
					{
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
						if(temp==0)
						{
							HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
						}
						if(encfeedback<3199 && encfeedback >3147 && encfeedback>=(encvalue))
						{
							sprintf(msg,"Required Position attained\r\n");
							HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
							HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
							temp=1;
							running=0;
							leftlastval=inputval;
							lastval=inputval;

						}
					}

				}
				else if((inputval>=0 && inputval<=4) || (inputval>=88 && inputval<=90))
				{
					running=0;
					leftlastval=inputval;
					lastval=inputval;

				}
				else if((inputval>=91 && inputval<=94)|| (inputval>=180 && inputval<=178))
				{
					running=0;
					rightlastval=inputval;
					lastval=inputval;

				}




	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 249;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2560-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse =   htim3.Init.Period *1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);


	/*Configure GPIO pins : PC6 PC8 PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	// GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA7_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// CAN_TxHeaderTypeDef TxHeader;

	CAN1_Tx();



}
void CAN_Filter_Config(void)
{
	CAN_FilterTypeDef can1_filter_init;

	can1_filter_init.FilterActivation = ENABLE;
	can1_filter_init.FilterBank  = 0;
	can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
	can1_filter_init.FilterIdHigh = 0x0000;
	can1_filter_init.FilterIdLow = 0x0000;
	can1_filter_init.FilterMaskIdHigh = 0x0000;
	can1_filter_init.FilterMaskIdLow = 0x0000;
	can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;

	if( HAL_CAN_ConfigFilter(&hcan,&can1_filter_init) != HAL_OK)
	{
		Error_Handler();
	}

}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	char msg[100];
	sprintf(msg,"Message Transmitted:M0\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

}


void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	char msg[100];
	sprintf(msg,"Message Transmitted:M1\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	char msg[100];
	sprintf(msg,"Message Transmitted:M2\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	// CAN_RxHeaderTypeDef RxHeader;
	uint8_t rcvd_msg[8];

	char msg[100];
	int b1=0,b2=0,b3=0,b4=0,b5=0,b6=0,b7=0,b8=0;

	//while(! HAL_CAN_GetRxFifoFillLevel(&hcan,CAN_RX_FIFO0));
	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,rcvd_msg) != HAL_OK)
	{
		Error_Handler();
	}
	if (RxHeader.StdId == 0x65D && RxHeader.RTR==0)
	{
		//LED_Manage_Output(rcvd_msg[0]);
		b1=rcvd_msg[0], b2=rcvd_msg[1], b3=rcvd_msg[2], b4=rcvd_msg[3],b5=rcvd_msg[4], b6=rcvd_msg[5], b7=rcvd_msg[6],b8=rcvd_msg[7];
		sprintf(msg,"Message Received : ID-%x %d %d %d %d %d %d %d %d \r\n",RxHeader.StdId,b1,b2,b3,b4,b5,b6,b7,b8);
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
	}
	//Working Code Begin

	inputval=(float)b3; //CAN VALUE is received as int. so converting to float
	temp=0;
	if(inputval!=lastval)
	{
		running=1;
	}
	else
	{
		running=0;
	}
	while(running)
	{
		memset(msg,'\0',sizeof(msg));  // Clearing the msg buffer
		count=TIM1->CNT;               //Counting the encoder pulse
		sprintf(msg,"Encoder value: [%d]\r\n",count);
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,sizeof(msg),0x1000);
		sprintf(msg,"Input value: [%d]\r\n",(int)inputval);
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,sizeof(msg),0x1000);
		encfeedback=(int)count;

		if(inputval>92 && inputval<178)
		{

			//inputval = CAN and Angle
			//enc val= inputval *0.583
			degree=inputval;
			encvalue=(int)(degree*0.583);
			sprintf(msg,"Required Encoder Value: %d\r\n",encvalue);
			HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
			if(inputval>=rightlastval)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
				if(temp==0)
				{
					sprintf(msg,"Started PWM\r\n");
					HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
					HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
				}
				if(encfeedback>0 && encfeedback<52 && encfeedback>=(encvalue))
				{
					sprintf(msg,"Required Position attained\r\n");
					HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
					HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
					temp=1;
					running=0;
					rightlastval=inputval;
					lastval=inputval;

				}
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
				if(temp==0)
				{
					sprintf(msg,"Started PWM\r\n");
					HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
					HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
				}
				if(encfeedback>0 && encfeedback<52 &&encfeedback<=(encvalue))
				{
					sprintf(msg,"Required Position attained\r\n");
					HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
					HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
					temp=1;
					running=0;
					rightlastval=inputval;
					lastval=inputval;

				}
			}

		}
		else if(inputval>4 && inputval<88)
		{
			degree=inputval;
			encvalue=3199-(int)(degree);
			sprintf(msg,"Required Encoder Value: %d\r\n",encvalue);
			HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
			if(inputval>=leftlastval)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
				if(temp==0)
				{
					HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
				}
				if(encfeedback<3199 && encfeedback >3147 && encfeedback<=encvalue)
				{
					sprintf(msg,"Required Position attained\r\n");
					HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
					HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
					temp=1;
					running=0;
					leftlastval=inputval;
					lastval=inputval;

				}
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
				if(temp==0)
				{
					HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
				}
				if(encfeedback<3199 && encfeedback >3147 && encfeedback>=(encvalue))
				{
					sprintf(msg,"Required Position attained\r\n");
					HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
					HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
					temp=1;
					running=0;
					leftlastval=inputval;
					lastval=inputval;

				}
			}

		}
		else if((inputval>=0 && inputval<=4) || (inputval>=88 && inputval<=90))
		{
			running=0;
			leftlastval=inputval;
			lastval=inputval;

		}
		else if((inputval>=91 && inputval<=94)|| (inputval>=180 && inputval<=178))
		{
			running=0;
			rightlastval=inputval;
			lastval=inputval;

		}



	}


	//Working Code End



	//HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}


void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{

	char msg[100];
	sprintf(msg,"CAN Error Detected\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}
void CAN1_Tx()
{
	//CAN_TxHeaderTypeDef TxHeader;

	uint32_t TxMailbox;

	//	uint8_t message;
	uint8_t a[8] = {0,100,0,0,1,0,1,'F'};;

	TxHeader.DLC = 8;
	TxHeader.StdId = 0x65D;
	TxHeader.IDE   = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	char msg[100];
	sprintf(msg,"Inside CAN1 Tx \r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);


	if( HAL_CAN_AddTxMessage(&hcan,&TxHeader,a,&TxMailbox) != HAL_OK)
	{
		sprintf(msg,"CAN Error.. \r\n");
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
		Error_Handler();
	}

}






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
