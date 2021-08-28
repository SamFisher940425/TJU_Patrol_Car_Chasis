/*¼ÇÂ¼
*µÚÒ»Ì¨
1.ÈýÉ«µÆ PI4---ºìµÆ£¬PI5---»ÆµÆ£¬PI6---ÂÌµÆ£¬PI7---ÃùµÑ
2.UART1_RxBuffer[10]¿ØÖÆÈýÉ«µÆ£¬00---ÂÌµÆÁÁ£¬01---»ÆµÆÁÁ£¬02---ºìµÆÁÁ£¬03---ÃùµÑ£¬04---¹Ø±ÕÈýÉ«µÆ£¬FF---ºìµÆÁÁ+ÃùµÑ
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t Time_Cnt=0;
uint8_t Init_Flag=0;
uint8_t Ask_Speed_L_Cnt=0;
uint8_t Ask_Speed_R_Cnt=0;

uint8_t UART1_RxByte[1];
uint8_t UART1_RxBuffer[16];
uint8_t UART1_RxBufPtr = 0;
uint8_t UART1_RxFlag = 0;//0 1 2 3 101
uint8_t UART1_TxByte[1];
uint8_t UART1_TxBuffer[16];
uint8_t UART1_TxBufPtr = 0;
uint8_t UART1_TxFlag = 0;//0 1
uint8_t UART1_return = 0;//0 1


uint8_t CAN1_TxFlag = 0;//bit0 set speed_L,bit1 set speed_R,bit2 asked speed_L,bit3 asked speed_R
uint8_t CAN1_RxFlag = 0;//bit0 get speed_L,bit1 get speed_R
CanTxMsgTypeDef CAN1_TxMessage;
CanRxMsgTypeDef CAN1_RxMessage;

uint8_t Motor_Enable = 0;
uint16_t Motor_Stop_Cnt = 0;
uint8_t Motor_Left_Error_Flag = 0;
uint8_t Motor_Left_Error_Clear_Flag = 0;
uint8_t Motor_Right_Error_Flag = 0;
uint8_t Motor_Right_Error_Clear_Flag = 0;

int16_t Speed_Want_Left = 0;
int16_t Speed_Want_Right = 0;
int16_t Speed_Real_Left = 0;
int16_t Speed_Real_Right = 0;

uint8_t Light_Want_Status = 0;
uint8_t Light_Real_Status = 0;

//right wheel can id is 01 and speed direction is positive

union Hex_Float_Transfer
{
	uint8_t Hex_Num[4];
	float Float_Num;
}Hex_to_Float,Float_to_Hex;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void SystemClock_Config(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void CAN1_Filter_Configure(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Delay(__IO uint32_t nCount);

void Delay(__IO uint32_t nCount)
{
  while(nCount--){}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  SystemClock_Config();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
	hcan1.pRxMsg = &CAN1_RxMessage;
	hcan1.pTxMsg = &CAN1_TxMessage;
	CAN1_Filter_Configure();
	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO1);
	HAL_UART_Receive_IT(&huart1,UART1_RxByte,1);
	HAL_TIM_Base_Start_IT(&htim7);
	
	if(Init_Flag == 0)
	{
		HAL_Delay(3000);
		
		CAN1_TxMessage.StdId = 0x01;
		CAN1_TxMessage.ExtId = 0x01;
		CAN1_TxMessage.IDE = CAN_ID_STD;
		CAN1_TxMessage.RTR = CAN_RTR_DATA;
		CAN1_TxMessage.DLC = 8;
		CAN1_TxMessage.Data[0]=0x00;
		CAN1_TxMessage.Data[1]=0x1A;
		CAN1_TxMessage.Data[2]=0x02;
		CAN1_TxMessage.Data[3]=0x00;
		CAN1_TxMessage.Data[4]=0xC4;
		CAN1_TxMessage.Data[5]=0x0A;
		CAN1_TxMessage.Data[6]=0x1E;
		CAN1_TxMessage.Data[7]=0x1E;
		HAL_CAN_Transmit_IT(&hcan1);//speed mode acc 0x0a
		
		HAL_Delay(10);
		
		CAN1_TxMessage.StdId = 0x02;
		CAN1_TxMessage.ExtId = 0x02;
		CAN1_TxMessage.IDE = CAN_ID_STD;
		CAN1_TxMessage.RTR = CAN_RTR_DATA;
		CAN1_TxMessage.DLC = 8;
		CAN1_TxMessage.Data[0]=0x00;
		CAN1_TxMessage.Data[1]=0x1A;
		CAN1_TxMessage.Data[2]=0x02;
		CAN1_TxMessage.Data[3]=0x00;
		CAN1_TxMessage.Data[4]=0xC4;
		CAN1_TxMessage.Data[5]=0x0A;
		CAN1_TxMessage.Data[6]=0x1E;
		CAN1_TxMessage.Data[7]=0x1E;
		HAL_CAN_Transmit_IT(&hcan1);
		
		HAL_Delay(100);
		
		Init_Flag = 1;
	}
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(UART1_RxFlag == 3 && (CAN1_TxFlag & 0x03) == 0)
		{
			Motor_Enable = 1;//enable motor
			Motor_Stop_Cnt = 0;
			
			Hex_to_Float.Hex_Num[0] = UART1_RxBuffer[2];
			Hex_to_Float.Hex_Num[1] = UART1_RxBuffer[3];
			Hex_to_Float.Hex_Num[2] = UART1_RxBuffer[4];
			Hex_to_Float.Hex_Num[3] = UART1_RxBuffer[5];
			Speed_Want_Left = (int16_t)Hex_to_Float.Float_Num;
			if(Speed_Want_Left>=3000.0)Speed_Want_Left=3000.0;
			if(Speed_Want_Left<=-3000.0)Speed_Want_Left=-3000.0;
			CAN1_TxFlag |= 0x01;
			
			Hex_to_Float.Hex_Num[0] = UART1_RxBuffer[6];
			Hex_to_Float.Hex_Num[1] = UART1_RxBuffer[7];
			Hex_to_Float.Hex_Num[2] = UART1_RxBuffer[8];
			Hex_to_Float.Hex_Num[3] = UART1_RxBuffer[9];
			Speed_Want_Right = (int16_t)Hex_to_Float.Float_Num;
			if(Speed_Want_Right>=3000.0)Speed_Want_Right=3000.0;
			if(Speed_Want_Right<=-3000.0)Speed_Want_Right=-3000.0;
			CAN1_TxFlag |= 0x02;
			
			Light_Want_Status = UART1_RxBuffer[10] & 0x0F; // get low 4 bit
			Motor_Left_Error_Clear_Flag = (UART1_RxBuffer[10] >> 4) & 0x01; // get 5th bit
			Motor_Right_Error_Clear_Flag = Motor_Left_Error_Clear_Flag;
			
			for(int i = 0;i < 16;i++)
			{
				UART1_RxBuffer[i]=0;
			}
			UART1_RxBufPtr = 0;
			UART1_RxFlag = 0;
			UART1_return = 1;

		}
		
		switch(Light_Want_Status)
		{
			case 0x00 : 
			{
				Light_Real_Status = 0x00;
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_RESET); //ÂÌµÆÁÁ
				break;
			}
			case 0x01 : 
			{
				Light_Real_Status = 0x01;
				//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);   //LED0¶ÔÓ¦Òý½ÅPB1À­¸ß£¬Ãð£¬µÈÍ¬ÓÚLED0(1)
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_5,GPIO_PIN_RESET); //»ÆµÆÁÁ
				break;
			}
			case 0x02 : 
			{
				Light_Real_Status = 0x02;
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_RESET); //ºìµÆÁÁ
				break;
			}
			case 0x03 : 
			{
				Light_Real_Status = 0x03;
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_RESET); //ÃùµÑ
				break;
			}
			case 0x04 : 
			{
				Light_Real_Status = 0x04;
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_SET);//¹Ø±ÕÈýÉ«µÆ
				break;
			}
			default : 
			{
				Light_Real_Status = 0x0F; //changed high 4 bit for motor error infomation
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_RESET); //ºìµÆÁÁ+ÃùµÑ
				break;
			}
		}
		
		if(CAN1_RxFlag == 0x03 && UART1_TxFlag == 0 && UART1_return == 1)//both bit0 and bit1 are 1
		{
			UART1_TxBuffer[0] = 0x55;
			UART1_TxBuffer[1] = 0xAA;
			
			Float_to_Hex.Float_Num = (float)Speed_Real_Left;
			CAN1_RxFlag &= ~0x01;
			UART1_TxBuffer[2] = Float_to_Hex.Hex_Num[0];
			UART1_TxBuffer[3] = Float_to_Hex.Hex_Num[1];
			UART1_TxBuffer[4] = Float_to_Hex.Hex_Num[2];
			UART1_TxBuffer[5] = Float_to_Hex.Hex_Num[3];
			
			Float_to_Hex.Float_Num = (float)Speed_Real_Right;
			CAN1_RxFlag &= ~0x02;
			UART1_TxBuffer[6] = Float_to_Hex.Hex_Num[0];
			UART1_TxBuffer[7] = Float_to_Hex.Hex_Num[1];
			UART1_TxBuffer[8] = Float_to_Hex.Hex_Num[2];
			UART1_TxBuffer[9] = Float_to_Hex.Hex_Num[3];
			
			UART1_TxBuffer[10] = Motor_Left_Error_Flag; //Light_Real_Status
			UART1_TxBuffer[11] = 0x0A;
			
			UART1_TxFlag = 1;
			
		}
		
		if(Speed_Want_Left>=3000.0)Speed_Want_Left=3000.0;
		if(Speed_Want_Left<=-3000.0)Speed_Want_Left=-3000.0;
		if(Speed_Want_Right>=3000.0)Speed_Want_Right=3000.0;
		if(Speed_Want_Right<=-3000.0)Speed_Want_Right=-3000.0;
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
void MX_NVIC_Init(void)
{
  /* CAN1_TX_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* CAN1_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 3);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* CAN1_SCE_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 4);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 1, 3);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_8TQ;
  hcan1.Init.BS2 = CAN_BS2_6TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = ENABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan1);

}

/* TIM7 init function */
void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DS0_Pin|DS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DS0_Pin DS1_Pin */
  GPIO_InitStruct.Pin = DS0_Pin|DS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

	GPIO_InitStruct.Pin=GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7; //PB1,0
  GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;  //ÍÆÍìÊä³ö
  GPIO_InitStruct.Pull=GPIO_PULLUP;          //ÉÏÀ­
  GPIO_InitStruct.Speed=GPIO_SPEED_HIGH;     //¸ßËÙ
  HAL_GPIO_Init(GPIOI,&GPIO_InitStruct);
	

}

/* USER CODE BEGIN 4 */
void CAN1_Filter_Configure(void)
{
	CAN_FilterConfTypeDef  CAN1_FilerConf;
	
	CAN1_FilerConf.FilterIdHigh=0X0000;     //32Î»ID
	CAN1_FilerConf.FilterIdLow=0X0000;
  CAN1_FilerConf.FilterMaskIdHigh=0X0000; //32Î»MASK
  CAN1_FilerConf.FilterMaskIdLow=0X0000;  
  CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//¹ýÂËÆ÷0¹ØÁªµ½FIFO0
  CAN1_FilerConf.FilterNumber=0;          //¹ýÂËÆ÷0
  CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDMASK;
  CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_32BIT;
  CAN1_FilerConf.FilterActivation=ENABLE; //¼¤»îÂË²¨Æ÷0
  CAN1_FilerConf.BankNumber=14;
	
	HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf);
	
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
