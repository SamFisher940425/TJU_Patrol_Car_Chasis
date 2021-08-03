/*记录
*第一台
1.UART1_RxBuffer[9]控制补光灯，0A---开，00---关
2.UART1_RxBuffer[10]控制充电开关，02---开，01---关
3.PI4---补光灯，PI6---充电开关
*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    //具体哪个串口可以更改huart1为其它串口
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1 , 0xffff);
    return ch;
}

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t Time_Cnt=0;
uint16_t UART1_Time_Cnt=0;
uint8_t Ask_Position_Yaw_Cnt=0; //20210802
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

uint8_t CAN1_TxFlag = 0;//bit0 set speed_L,bit1 set speed_R,bit2 asked speed_L,bit3 asked speed_R
uint8_t CAN1_RxFlag = 0;//bit0 get speed_L,bit1 get speed_R
CanTxMsgTypeDef CAN1_TxMessage;
CanRxMsgTypeDef CAN1_RxMessage;

uint8_t Motor_Enable = 0;
uint16_t Motor_Stop_Cnt = 0;

int16_t Speed_Want_Left = 0;
int16_t Speed_Want_Right = 0;
int16_t Speed_Real_Left = 0;
int16_t Speed_Real_Right = 0;

uint8_t Light_Want_Status = 0;
uint8_t Light_Real_Status = 0;

int32_t Yaw_Positon = 0; //yaw motor real position //20210802
uint8_t Ask_Yaw_Position_Flag = 0; //0 no new position_want 1 get new position_want and moving 2 get real position and calculating //20210802

int16_t phjd = 0;
int16_t fyjd = 0;
int64_t fyjd0 = 0;
int64_t phjd0 = 0;

int phjd1=0;
int phjd2=0;
int phjd3=0;
int fyjd1=0;
int fyjd2=0;
int fyjd3=0;
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

int32_t abs(int32_t a)
{
	if(a < 0)a = -a;
	return a;
}
/* USER CODE END 0 */

int main(void)
{

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
		HAL_Delay(4000);
		
		CAN1_TxMessage.StdId = 0x0605;
		CAN1_TxMessage.ExtId = 0x0605;
		CAN1_TxMessage.IDE = CAN_ID_STD;
		CAN1_TxMessage.RTR = CAN_RTR_DATA;
		CAN1_TxMessage.DLC = 8;
		CAN1_TxMessage.Data[0]=0x2f;
		CAN1_TxMessage.Data[1]=0x05;
		CAN1_TxMessage.Data[2]=0x60;
		CAN1_TxMessage.Data[3]=0x00;
		CAN1_TxMessage.Data[4]=0x00;
		CAN1_TxMessage.Data[5]=0x00;
		CAN1_TxMessage.Data[6]=0x00;
		CAN1_TxMessage.Data[7]=0x00;
		HAL_CAN_Transmit_IT(&hcan1);//帧ID605,偏航电机模式 位置
	  
		HAL_Delay(100);		
		CAN1_TxMessage.StdId = 0x0605;
		CAN1_TxMessage.ExtId = 0x0605;
		CAN1_TxMessage.IDE = CAN_ID_STD;
		CAN1_TxMessage.RTR = CAN_RTR_DATA;
		CAN1_TxMessage.DLC = 8;
		CAN1_TxMessage.Data[0]=0x2f;
		CAN1_TxMessage.Data[1]=0x0e;
		CAN1_TxMessage.Data[2]=0x60;
		CAN1_TxMessage.Data[3]=0x00;
		CAN1_TxMessage.Data[4]=0x01;
		CAN1_TxMessage.Data[5]=0x00;
		CAN1_TxMessage.Data[6]=0x00;
		CAN1_TxMessage.Data[7]=0x00;
		HAL_CAN_Transmit_IT(&hcan1);//帧ID605,偏航电机使能2f 0e 60 00 01 00 00 00
	  
		HAL_Delay(100);		
		
	  CAN1_TxMessage.StdId = 0x0605;
		CAN1_TxMessage.ExtId = 0x0605;
		CAN1_TxMessage.IDE = CAN_ID_STD;
		CAN1_TxMessage.RTR = CAN_RTR_DATA;
		CAN1_TxMessage.DLC = 8;
		CAN1_TxMessage.Data[0]=0x23;
		CAN1_TxMessage.Data[1]=0x03;
		CAN1_TxMessage.Data[2]=0x60;
		CAN1_TxMessage.Data[3]=0x00;
		CAN1_TxMessage.Data[4]=0xb8;
		CAN1_TxMessage.Data[5]=0x0b;
		CAN1_TxMessage.Data[6]=0x00;
		CAN1_TxMessage.Data[7]=0x00;
		HAL_CAN_Transmit_IT(&hcan1);//偏航电机最大速度--400，23 03 60 00 90 01 00 00
						
		HAL_Delay(100);
		
		CAN1_TxMessage.StdId = 0x00;
		CAN1_TxMessage.ExtId = 0x00;
		CAN1_TxMessage.IDE = CAN_ID_STD;
		CAN1_TxMessage.RTR = CAN_RTR_DATA;
		CAN1_TxMessage.DLC = 8;
		CAN1_TxMessage.Data[0]=0x01;
		CAN1_TxMessage.Data[1]=0x00;
		CAN1_TxMessage.Data[2]=0x00;
		CAN1_TxMessage.Data[3]=0x00;
		CAN1_TxMessage.Data[4]=0x00;
		CAN1_TxMessage.Data[5]=0x00;
		CAN1_TxMessage.Data[6]=0x00;
		CAN1_TxMessage.Data[7]=0x00;
		HAL_CAN_Transmit_IT(&hcan1);//俯仰电机，使从站进入OP模式
		
		HAL_Delay(100);
		
		CAN1_TxMessage.StdId = 0x0201;
		CAN1_TxMessage.ExtId = 0x0201;
		CAN1_TxMessage.IDE = CAN_ID_STD;
		CAN1_TxMessage.RTR = CAN_RTR_DATA;
		CAN1_TxMessage.DLC = 8;
		CAN1_TxMessage.Data[0]=0x06;
		CAN1_TxMessage.Data[1]=0x00;
		CAN1_TxMessage.Data[2]=0x00;
		CAN1_TxMessage.Data[3]=0x00;
		CAN1_TxMessage.Data[4]=0x00;
		CAN1_TxMessage.Data[5]=0x00;
		CAN1_TxMessage.Data[6]=0x00;
		CAN1_TxMessage.Data[7]=0x00;
		HAL_CAN_Transmit_IT(&hcan1);//俯仰电机进入操作模式
		
		HAL_Delay(100);
		
		CAN1_TxMessage.StdId = 0x0201;
		CAN1_TxMessage.ExtId = 0x0201;
		CAN1_TxMessage.IDE = CAN_ID_STD;
		CAN1_TxMessage.RTR = CAN_RTR_DATA;
		CAN1_TxMessage.DLC = 8;
		CAN1_TxMessage.Data[0]=0x07;
		CAN1_TxMessage.Data[1]=0x00;
		CAN1_TxMessage.Data[2]=0x00;
		CAN1_TxMessage.Data[3]=0x00;
		CAN1_TxMessage.Data[4]=0x00;
		CAN1_TxMessage.Data[5]=0x00;
		CAN1_TxMessage.Data[6]=0x00;
		CAN1_TxMessage.Data[7]=0x00;
		HAL_CAN_Transmit_IT(&hcan1);//俯仰电机，使能电机
		
		HAL_Delay(100);
		
		CAN1_TxMessage.StdId = 0x0301;
	  CAN1_TxMessage.ExtId = 0x0301;
		CAN1_TxMessage.IDE = CAN_ID_STD;
		CAN1_TxMessage.RTR = CAN_RTR_DATA;
		CAN1_TxMessage.DLC = 8;
		CAN1_TxMessage.Data[0]=0x01;
		CAN1_TxMessage.Data[1]=0x00;
		CAN1_TxMessage.Data[2]=0x00;
		CAN1_TxMessage.Data[3]=0x00;
		CAN1_TxMessage.Data[4]=0x00;
		CAN1_TxMessage.Data[5]=0x00;
		CAN1_TxMessage.Data[6]=0x00;
		CAN1_TxMessage.Data[7]=0x00;
		HAL_CAN_Transmit_IT(&hcan1);//俯仰电机找原点
			
		HAL_Delay(2000);
				 		
		Init_Flag = 1;
	}

  while (1)
  {		
		if(UART1_RxFlag == 3 && (CAN1_TxFlag & 0x03) == 0)
		{
			
			Motor_Enable = 1;//enable motor
			Motor_Stop_Cnt = 0;
			phjd =0;
			fyjd =0;
	
			CAN1_TxFlag |= 0x01;
		
			CAN1_TxFlag |= 0x02;
			
			if(UART1_RxBuffer[9]==0x0a)//补光灯开关
			{
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_RESET);
			}
			if(UART1_RxBuffer[9]==0x00)
			{
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_SET);
			}
			/*if(UART1_RxBuffer[2]==0xff && UART1_RxBuffer[3]==0xff && UART1_RxBuffer[4]==0xff && UART1_RxBuffer[5]==0xff)
			{
				CAN1_TxMessage.StdId = 0x0605;
				CAN1_TxMessage.ExtId = 0x0605;
				CAN1_TxMessage.IDE = CAN_ID_STD;
				CAN1_TxMessage.RTR = CAN_RTR_DATA;
				CAN1_TxMessage.DLC = 8;
				CAN1_TxMessage.Data[0]=0x2f;
				CAN1_TxMessage.Data[1]=0x20;
				CAN1_TxMessage.Data[2]=0x60;
				CAN1_TxMessage.Data[3]=0x00;
				CAN1_TxMessage.Data[4]=0x00;
				CAN1_TxMessage.Data[5]=0x00;
				CAN1_TxMessage.Data[6]=0x00;
				CAN1_TxMessage.Data[7]=0x00;
				HAL_CAN_Transmit_IT(&hcan1);//偏航电机停止
			}*/
			
			phjd=(int16_t)(UART1_RxBuffer[2]*256 + UART1_RxBuffer[3]);
			if (phjd>=3500) phjd=3500;
			
			phjd0=(int64_t)(phjd*46);
			phjd1=phjd0/65535;
			phjd2=(phjd0-phjd1*65535)/256;
			phjd3=phjd0%256;
			
			fyjd=(int16_t)(UART1_RxBuffer[4]*256 + UART1_RxBuffer[5]);
			if (fyjd>=1800) fyjd=1800;
			
			
			fyjd0=(int64_t)(fyjd*30.25*4.55);
			
			fyjd1=fyjd0/65535;
			
			fyjd2=(fyjd0-fyjd1*65535)/256;
			
			fyjd3=fyjd0%256;
			
			HAL_Delay(50);
			
			CAN1_TxMessage.StdId = 0x0605;
			CAN1_TxMessage.ExtId = 0x0605;
			CAN1_TxMessage.IDE = CAN_ID_STD;
			CAN1_TxMessage.RTR = CAN_RTR_DATA;
			CAN1_TxMessage.DLC = 8;
			CAN1_TxMessage.Data[0]=0x23;
			CAN1_TxMessage.Data[1]=0x1c;
			CAN1_TxMessage.Data[2]=0x60;
			CAN1_TxMessage.Data[3]=0x00;
			CAN1_TxMessage.Data[4]=(uint8_t)phjd3;
			CAN1_TxMessage.Data[5]=(uint8_t)phjd2;
			CAN1_TxMessage.Data[6]=(uint8_t)phjd1;
			CAN1_TxMessage.Data[7]=0x00;
			HAL_CAN_Transmit_IT(&hcan1);//绝对位移指令23 1c 60 00 xx xx xx xx
			if(Ask_Yaw_Position_Flag == 0)Ask_Yaw_Position_Flag = 1; // 20210802
			
			HAL_Delay(300);
			
			if(UART1_RxBuffer[6]==0xff && UART1_RxBuffer[7]==0xff)
		{
			CAN1_TxMessage.StdId = 0x0301;
			CAN1_TxMessage.ExtId = 0x0301;
			CAN1_TxMessage.IDE = CAN_ID_STD;
			CAN1_TxMessage.RTR = CAN_RTR_DATA;
			CAN1_TxMessage.DLC = 8;
			CAN1_TxMessage.Data[0]=0x01;
			CAN1_TxMessage.Data[1]=0x00;
			CAN1_TxMessage.Data[2]=0x00;
			CAN1_TxMessage.Data[3]=0x00;
			CAN1_TxMessage.Data[4]=0x00;
			CAN1_TxMessage.Data[5]=0x00;
			CAN1_TxMessage.Data[6]=0x00;
			CAN1_TxMessage.Data[7]=0x00;
			HAL_CAN_Transmit_IT(&hcan1);
			
			HAL_Delay(3000);
		}
			
			CAN1_TxMessage.StdId = 0x0201;
		  CAN1_TxMessage.ExtId = 0x0201;
		  CAN1_TxMessage.IDE = CAN_ID_STD;
			CAN1_TxMessage.RTR = CAN_RTR_DATA;
			CAN1_TxMessage.DLC = 8;
			CAN1_TxMessage.Data[0]=0x0f;
			CAN1_TxMessage.Data[1]=0x00;
			CAN1_TxMessage.Data[2]=0xff-(uint8_t)fyjd3;
			CAN1_TxMessage.Data[3]=0xff-(uint8_t)fyjd2;
			CAN1_TxMessage.Data[4]=0xff-(uint8_t)fyjd1;
			CAN1_TxMessage.Data[5]=0xff;
			CAN1_TxMessage.Data[6]=0xff;
			CAN1_TxMessage.Data[7]=0xff;
			HAL_CAN_Transmit_IT(&hcan1);//俯仰电机，电机转动
			
			HAL_Delay(300);
			
			CAN1_TxMessage.StdId = 0x0201;
		  CAN1_TxMessage.ExtId = 0x0201;
		  CAN1_TxMessage.IDE = CAN_ID_STD;
			CAN1_TxMessage.RTR = CAN_RTR_DATA;
			CAN1_TxMessage.DLC = 8;
			CAN1_TxMessage.Data[0]=0x5f;
			CAN1_TxMessage.Data[1]=0x00;
			CAN1_TxMessage.Data[2]=0xff-(uint8_t)fyjd3;
			CAN1_TxMessage.Data[3]=0xff-(uint8_t)fyjd2;
			CAN1_TxMessage.Data[4]=0xff-(uint8_t)fyjd1;
			CAN1_TxMessage.Data[5]=0xff;
			CAN1_TxMessage.Data[6]=0xff;
			CAN1_TxMessage.Data[7]=0xff;
			HAL_CAN_Transmit_IT(&hcan1);//俯仰电机，电机转动
						
			Light_Want_Status = UART1_RxBuffer[10];
			
			for(int i = 0;i < 16;i++)
			{
				UART1_RxBuffer[i]=0;
			}
			UART1_RxBufPtr = 0;
			UART1_RxFlag = 0;
			
		}
		
		if(Ask_Position_Yaw_Cnt >=50) //20210802
		{
			CAN1_TxMessage.StdId = 0x0605;
			CAN1_TxMessage.ExtId = 0x0605;
			CAN1_TxMessage.IDE = CAN_ID_STD;
		  CAN1_TxMessage.RTR = CAN_RTR_DATA;
		  CAN1_TxMessage.DLC = 8;
			CAN1_TxMessage.Data[0]=0x40;
			CAN1_TxMessage.Data[1]=0x0c;
			CAN1_TxMessage.Data[2]=0x60;
			CAN1_TxMessage.Data[3]=0x00;
			CAN1_TxMessage.Data[4]=0x00;
			CAN1_TxMessage.Data[5]=0x00;
			CAN1_TxMessage.Data[6]=0x00;
			CAN1_TxMessage.Data[7]=0x00;
			HAL_CAN_Transmit_IT(&hcan1);//read real positon 40 0c 60 00 00 00 00 00
			Ask_Position_Yaw_Cnt = 0;
		}
			
		if(Ask_Yaw_Position_Flag == 2) // 20210802
		{
			if(abs((int32_t)phjd0 - abs(Yaw_Positon)) <= 1000) // the error between positon_want and real position
			{
				UART1_TxBuffer[0]=0x55;
				UART1_TxBuffer[1]=0xAA;
				UART1_TxBuffer[2]=0x01;
				UART1_TxBuffer[3]=0x01;
				UART1_TxBuffer[4]=0x00;
				UART1_TxBuffer[5]=0x00;
				UART1_TxBuffer[6]=0x00;
				UART1_TxBuffer[7]=0x00;
				UART1_TxBuffer[8]=0x00;
				UART1_TxBuffer[9]=0x00;
				UART1_TxBuffer[10]=0x00;
				UART1_TxBuffer[11]=0x0A;
				UART1_TxFlag = 1;
				Ask_Yaw_Position_Flag = 0;
			}
			else
			{
				Ask_Yaw_Position_Flag = 1;
			}
		}
		
		switch(Light_Want_Status)//充电开关
		{
			case 0x01 : 
			{
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
				break;
			}
			case 0x02 : 
			{
				HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_RESET);
				break;
			}			
		}	
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
	
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void CAN1_Filter_Configure(void)
{
	CAN_FilterConfTypeDef  CAN1_FilerConf;
	
	CAN1_FilerConf.FilterIdHigh=0X0000;     //32位ID
	CAN1_FilerConf.FilterIdLow=0X0000;
  CAN1_FilerConf.FilterMaskIdHigh=0X0000; //32位MASK
  CAN1_FilerConf.FilterMaskIdLow=0X0000;  
  CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
  CAN1_FilerConf.FilterNumber=0;          //过滤器0
  CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDMASK;
  CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_32BIT;
  CAN1_FilerConf.FilterActivation=ENABLE; //激活滤波器0
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
