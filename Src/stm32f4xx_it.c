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
extern uint8_t UART1_RxByte[1];
extern uint8_t UART1_RxBuffer[16];
extern uint8_t UART1_RxBufPtr;
extern uint8_t UART1_RxFlag;

extern uint8_t UART1_TxByte[1];
extern uint8_t UART1_TxBuffer[16];
extern uint8_t UART1_TxBufPtr;
extern uint8_t UART1_TxFlag;//0 1 101

extern uint8_t CAN1_TxFlag;//0 1
extern uint8_t CAN1_RxFlag;//0 1
extern CanTxMsgTypeDef CAN1_TxMessage;
extern CanRxMsgTypeDef CAN1_RxMessage;


extern uint8_t Motor_Enable;
extern uint16_t Motor_Stop_Cnt;

extern int Speed_Want_Left;
extern int Speed_Want_Right;

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
//	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO1);
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
	HAL_UART_Receive_IT(&huart1,UART1_RxByte,1);
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
			if(Motor_Stop_Cnt >= 200)
			{
				Speed_Want_Left = 0;
				Speed_Want_Right = 0;
				Motor_Enable = 0;
				Motor_Stop_Cnt = 0;
				CAN1_TxFlag = 1;//send massage immediately
			}
		}
		
		//CAN send message include speed_want and speed ask
		if(CAN1_TxFlag == 1)
		{
			CAN1_TxMessage.StdId = 0x12;
			CAN1_TxMessage.ExtId = 0x12;
			CAN1_TxMessage.IDE = CAN_ID_STD;
			CAN1_TxMessage.RTR = CAN_RTR_DATA;
			CAN1_TxMessage.DLC = 8;
			CAN1_TxMessage.Data[0]=0x01;
			CAN1_TxMessage.Data[1]=0x02;
			CAN1_TxMessage.Data[2]=0x03;
			CAN1_TxMessage.Data[3]=0x04;
			CAN1_TxMessage.Data[4]=0x05;
			CAN1_TxMessage.Data[5]=0x06;
			CAN1_TxMessage.Data[6]=0x07;
			CAN1_TxMessage.Data[7]=0x08;
			HAL_CAN_Transmit(&hcan1,1000);
			
			CAN1_TxFlag = 0;
		}
		
		if(UART1_TxFlag == 1)
		{
			HAL_UART_Transmit(&huart1,UART1_TxBuffer,11,2);
			UART1_TxFlag = 0;
		}

		if(UART1_RxFlag == 1 || UART1_RxFlag == 2)
		{
			Time_Cnt+=1;
			if(Time_Cnt>=20)
			{
				for(int i = 0;i < 16;i++)
				{
					UART1_RxBuffer[i]=0;
				}
				UART1_RxBufPtr = 0;
				UART1_RxFlag = 0;
				Time_Cnt = 0;
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
			if(UART1_RxBufPtr >= 10)
			{
				if(UART1_RxByte[0] == 0x0A && UART1_RxBufPtr == 10)
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
	}
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan==(&hcan1))
	{
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
