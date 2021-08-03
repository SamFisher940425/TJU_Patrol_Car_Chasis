/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
extern uint16_t Time_Cnt;
extern uint16_t UART1_Time_Cnt;
extern uint8_t Ask_Position_Yaw_Cnt;//20210802
extern uint8_t Init_Flag;
extern uint8_t Ask_Speed_L_Cnt;
extern uint8_t Ask_Speed_R_Cnt;

extern uint8_t UART1_RxByte[1];
extern uint8_t UART1_RxBuffer[16];
extern uint8_t UART1_RxBufPtr;
extern uint8_t UART1_RxFlag;//0 1 2 3 101

extern uint8_t UART1_TxByte[1];
extern uint8_t UART1_TxBuffer[16];
extern uint8_t UART1_TxBufPtr;
extern uint8_t UART1_TxFlag;//0 1

extern uint8_t CAN1_TxFlag;//0 1
extern uint8_t CAN1_RxFlag;//0 1
extern CanTxMsgTypeDef CAN1_TxMessage;
extern CanRxMsgTypeDef CAN1_RxMessage;

extern uint8_t Motor_Enable;
extern uint16_t Motor_Stop_Cnt;

extern int16_t Speed_Want_Left;
extern int16_t Speed_Want_Right;
extern int16_t Speed_Real_Left;
extern int16_t Speed_Real_Right;

extern uint8_t Ask_Yaw_Position_Flag; //20210802
extern int32_t Yaw_Positon; //20210802

void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_DebugMonitor_IRQn 0 */
    /* USER CODE END W1_DebugMonitor_IRQn 0 */
  }
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles CAN1 TX interrupts.
*/
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
* @brief This function handles CAN1 RX0 interrupts.
*/
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
* @brief This function handles CAN1 RX1 interrupt.
*/
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */
	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO1);
  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
* @brief This function handles CAN1 SCE interrupt.
*/
void CAN1_SCE_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_SCE_IRQn 0 */

  /* USER CODE END CAN1_SCE_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_SCE_IRQn 1 */
	
  /* USER CODE END CAN1_SCE_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	
  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==(&htim7))
	{
//		Time_Cnt+=1;
//		if(Time_Cnt>=100)
//		{
//			Time_Cnt = 0;
//			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
//		}
		
		if(Motor_Enable == 1)
		{
			Motor_Stop_Cnt += 1;
			if(Motor_Stop_Cnt >= 100)
			{
				Speed_Want_Left = 0;
				Speed_Want_Right = 0;
				Motor_Enable = 0;
				Motor_Stop_Cnt = 0;
				CAN1_TxFlag |= 0x03;//send massage immediately
			}
		}
		
		//if CAN1_RxFlag bit0 is 0,ask speed_L
		if((CAN1_RxFlag & 0x01) == 0 && (CAN1_TxFlag & 0x04) == 0 && (CAN1_TxFlag & 0x01) == 0 && Init_Flag != 0)
		{
			/*CAN1_TxMessage.StdId = 0x02;
			CAN1_TxMessage.ExtId = 0x02;
			CAN1_TxMessage.IDE = CAN_ID_STD;
			CAN1_TxMessage.RTR = CAN_RTR_DATA;
			CAN1_TxMessage.DLC = 8;
			CAN1_TxMessage.Data[0]=0x00;
			CAN1_TxMessage.Data[1]=0x2A;
			CAN1_TxMessage.Data[2]=0xe4;
			CAN1_TxMessage.Data[3]=0x00;
			CAN1_TxMessage.Data[4]=0x00;
			CAN1_TxMessage.Data[5]=0xe4;
			CAN1_TxMessage.Data[6]=0x00;
			CAN1_TxMessage.Data[7]=0x00;
			HAL_CAN_Transmit_IT(&hcan1);*/
			
			CAN1_TxFlag |= 0x04;
			Ask_Speed_L_Cnt = 0;
		}
		
		//if CAN1_RxFlag bit1 is 0,ask speed_R
		if((CAN1_RxFlag & 0x02) == 0 && (CAN1_TxFlag & 0x08) == 0 && (CAN1_TxFlag & 0x02) == 0 && Init_Flag != 0)
		{
			/*CAN1_TxMessage.StdId = 0x01;
			CAN1_TxMessage.ExtId = 0x01;
			CAN1_TxMessage.IDE = CAN_ID_STD;
			CAN1_TxMessage.RTR = CAN_RTR_DATA;
			CAN1_TxMessage.DLC = 8;
			CAN1_TxMessage.Data[0]=0x00;
			CAN1_TxMessage.Data[1]=0x2A;
			CAN1_TxMessage.Data[2]=0xe4;
			CAN1_TxMessage.Data[3]=0x00;
			CAN1_TxMessage.Data[4]=0x00;
			CAN1_TxMessage.Data[5]=0xe4;
			CAN1_TxMessage.Data[6]=0x00;
			CAN1_TxMessage.Data[7]=0x00;
			HAL_CAN_Transmit_IT(&hcan1);*/
			
			CAN1_TxFlag |= 0x08;
			Ask_Speed_R_Cnt = 0;
		}
		
		//CAN send message L
		if((CAN1_TxFlag & 0x01) != 0 && Init_Flag != 0)
		{
			/*CAN1_TxMessage.StdId = 0x02;
			CAN1_TxMessage.ExtId = 0x02;
			CAN1_TxMessage.IDE = CAN_ID_STD;
			CAN1_TxMessage.RTR = CAN_RTR_DATA;
			CAN1_TxMessage.DLC = 8;
			CAN1_TxMessage.Data[0]=0x00;
			CAN1_TxMessage.Data[1]=0x1A;
			CAN1_TxMessage.Data[2]=0x00;
			CAN1_TxMessage.Data[3]=0x00;
			if(Motor_Enable == 1)
				CAN1_TxMessage.Data[4]=0x01;
			else
				CAN1_TxMessage.Data[4]=0x00;
			CAN1_TxMessage.Data[5]=0x06;
			CAN1_TxMessage.Data[6]=(((int16_t)((-1.0)*Speed_Want_Left/3000.0*8192.0)) >> 8) & 0x00FF;
			CAN1_TxMessage.Data[7]=((int16_t)((-1.0)*Speed_Want_Left/3000.0*8192.0)) & 0x00FF;
			
			HAL_CAN_Transmit_IT(&hcan1);//speed set motor enable/disable*/
			
			CAN1_TxFlag &= ~0x01;
		}
		
		//CAN send message R
		if((CAN1_TxFlag & 0x02) != 0 && Init_Flag != 0)
		{
			/*CAN1_TxMessage.StdId = 0x01;
			CAN1_TxMessage.ExtId = 0x01;
			CAN1_TxMessage.IDE = CAN_ID_STD;
			CAN1_TxMessage.RTR = CAN_RTR_DATA;
			CAN1_TxMessage.DLC = 8;
			CAN1_TxMessage.Data[0]=0x00;
			CAN1_TxMessage.Data[1]=0x1A;
			CAN1_TxMessage.Data[2]=0x00;
			CAN1_TxMessage.Data[3]=0x00;
			if(Motor_Enable == 1)
				CAN1_TxMessage.Data[4]=0x01;
			else
				CAN1_TxMessage.Data[4]=0x00;
			CAN1_TxMessage.Data[5]=0x06;
			CAN1_TxMessage.Data[6]=(((int16_t)(Speed_Want_Right/3000.0*8192.0)) >> 8) & 0x00FF;
			CAN1_TxMessage.Data[7]=((int16_t)(Speed_Want_Right/3000.0*8192.0)) & 0x00FF;
			
			HAL_CAN_Transmit_IT(&hcan1);//speed set motor enable/disable*/
			
			CAN1_TxFlag &= ~0x02;
		}
		
		if(UART1_TxFlag == 1)
		{
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
			HAL_UART_Transmit_IT(&huart1,UART1_TxBuffer,12);
			UART1_TxFlag = 0;
		}
		
		if((CAN1_TxFlag & 0x04) != 0)
		{
			Ask_Speed_L_Cnt +=1;
			if(Ask_Speed_L_Cnt >= 100)
			{
				CAN1_TxFlag &= ~0x04;
				Ask_Speed_L_Cnt = 0;
			}
		}
		
		if((CAN1_TxFlag & 0x08) != 0)
		{
			Ask_Speed_R_Cnt +=1;
			if(Ask_Speed_R_Cnt >= 100)
			{
				CAN1_TxFlag &= ~0x08;
				Ask_Speed_R_Cnt = 0;
			}
		}
		
		if(Ask_Yaw_Position_Flag != 0) // 20210802
		{
			Ask_Position_Yaw_Cnt += 1;
		}
		else
		{
			if(Ask_Position_Yaw_Cnt != 0)Ask_Position_Yaw_Cnt = 0;
		}

		if(UART1_RxFlag == 1 || UART1_RxFlag == 2)
		{
			UART1_Time_Cnt+=1;
			if(UART1_Time_Cnt>=25)
			{
				for(int i = 0;i < 16;i++)
				{
					UART1_RxBuffer[i]=0;
				}
				UART1_RxBufPtr = 0;
				UART1_RxFlag = 0;
				UART1_Time_Cnt = 0;
			}
		}
		
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==(&huart1))
	{
		//Byte 1//
		if(UART1_RxFlag == 0)
		{
			if(UART1_RxByte[0] == 0x55)
			{
				UART1_RxBuffer[UART1_RxBufPtr] = UART1_RxByte[0];
				UART1_RxBufPtr += 1;
				UART1_RxFlag = 1;
			}
			else
			{
				UART1_RxFlag = 101;
			}
		}
		//Byte 2//
		else if (UART1_RxFlag == 1)
		{
			if(UART1_RxByte[0] == 0xAA)
			{
				UART1_RxBuffer[UART1_RxBufPtr] = UART1_RxByte[0];
				UART1_RxBufPtr += 1;
				UART1_RxFlag = 2;
			}
			else
			{
				UART1_RxFlag = 101;
			}
		}
		//Byte 3-11//
		else if (UART1_RxFlag == 2)
		{
			if(UART1_RxBufPtr >= 11)
			{
				if(UART1_RxByte[0] == 0x0A && UART1_RxBufPtr == 11)
				{
					UART1_RxFlag = 3;
				}
				else
				{
					UART1_RxFlag = 101;
				}
			}
			else
			{
				UART1_RxBuffer[UART1_RxBufPtr] = UART1_RxByte[0];
				UART1_RxBufPtr += 1;
			}
		}
		//error//
		if(UART1_RxFlag == 101)
		{
			for(int i = 0;i < 16;i++)
			{
				UART1_RxBuffer[i]=0;
			}
			UART1_RxBufPtr = 0;
			UART1_RxFlag = 0;
		}
		
		HAL_UART_Receive_IT(&huart1,UART1_RxByte,1);
	}
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan==(&hcan1))
	{
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);//
//		if(CAN1_RxMessage.StdId == 0x02 && (CAN1_TxFlag & 0x04) != 0)
//		{
//			if(CAN1_RxMessage.Data[1] == 0x2b && CAN1_RxMessage.Data[2] == 0xe4 && (CAN1_RxFlag & 0x01) == 0)
//			{
//				Speed_Real_Left = (-1)*(((int16_t)((CAN1_RxMessage.Data[3] << 8) | CAN1_RxMessage.Data[4]))/8192.0*3000.0);//
//				CAN1_RxFlag |= 0x01;
//				CAN1_TxFlag &= ~0x04;
//			}
//			else if(CAN1_RxMessage.Data[1] == 0x2c && CAN1_RxMessage.Data[2] == 0xe4)
//			{
//				CAN1_RxFlag &= ~0x01;
//				CAN1_TxFlag &= ~0x04;
//			}
//		}
//		
//		if(CAN1_RxMessage.StdId == 0x01 && (CAN1_TxFlag & 0x08) != 0)
//		{
//			if(CAN1_RxMessage.Data[1] == 0x2b && CAN1_RxMessage.Data[2] == 0xe4 && (CAN1_RxFlag & 0x02) == 0)
//			{
//				Speed_Real_Right = ((int16_t)((CAN1_RxMessage.Data[3] << 8) | CAN1_RxMessage.Data[4]))/8192.0*3000.0;//((CAN1_RxMessage.Data[3] << 8) | CAN1_RxMessage.Data[4])/8192.0*3000.0
//				CAN1_RxFlag |= 0x02;
//				CAN1_TxFlag &= ~0x08;
//			}
//			else if(CAN1_RxMessage.Data[1] == 0x2c && CAN1_RxMessage.Data[2] == 0xe4)
//			{
//				CAN1_RxFlag &= ~0x02;
//				CAN1_TxFlag &= ~0x08;
//			}
//		}

    if(CAN1_RxMessage.StdId == 0x0585)
		{
			if(CAN1_RxMessage.Data[0] == 0x43 && CAN1_RxMessage.Data[1] == 0x0c && CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[3] == 0x00)
			{
				Yaw_Positon = (CAN1_RxMessage.Data[7] << 24) | (CAN1_RxMessage.Data[6] << 16) | (CAN1_RxMessage.Data[5] << 8)  | CAN1_RxMessage.Data[4];
				if(Ask_Yaw_Position_Flag == 1)Ask_Yaw_Position_Flag = 2;
			}
		}

	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
