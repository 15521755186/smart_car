/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t RX_buffer[20] = {0};
uint8_t RX_buffer_sub[20] = {0};
_car_data car_data1;
_car_data car_data2;
uint8_t current_car;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
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
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
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
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	delay_ms(10);
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1)
	{
		uart3_send_string("1");
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	delay_ms(10);
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)==0)
	{
		delay_ms(10);
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)==0)
		uart3_send_string("2");
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	delay_ms(10);
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==0)
	{
		delay_ms(10);
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==0)
		uart3_send_string("3");
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	delay_ms(10);
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==0)
	{
		delay_ms(10);
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==0)
		uart3_send_string("4");
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	delay_ms(10);
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)==0)
	{
		delay_ms(10);
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)==0)
		{
			if(current_car == 1)
			{
				current_car =0;
			}
			else
			{
				current_car =1;
			}
		}
	}
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)==0)
	{
		delay_ms(10);
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)==0)
		{
			if(car_data1.transport_done_flag == 1)
			{
				switch(car_data1.qr_code_data)
				{
					case 0x31:uart3_send_string("3");break;
					case 0x32:uart3_send_string("4");break;
					case 0x33:uart3_send_string("1");break;
					case 0x34:uart3_send_string("2");break;
					default:break;
				}
			}
			else{}
		}
	}
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  uint8_t ch;
	int8_t index; 
	//uint8_t command_flag;
	//uint8_t command_type;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	if (__HAL_UART_GET_FLAG( &huart1, UART_FLAG_RXNE ) != RESET)
    {
			 ch=(uint16_t)READ_REG(huart1.Instance->DR);
			 for(index=1; index<=19; index++)
			 {
				 RX_buffer[index-1] = RX_buffer[index];
			 }
			 RX_buffer[19]=ch;
//			 uart1_send_char(RX_buffer[19]);
//			 __HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_RXNE);
			 if((RX_buffer[4]==0xff)&&(RX_buffer[3]==0xff)&&(RX_buffer[2]==0x00))
			 {
				 car_data1.arm_angle4 = RX_buffer[5];
				 car_data1.arm_angle3 = RX_buffer[6];
				 car_data1.arm_angle2 = RX_buffer[7];
				 car_data1.arm_angle1 = RX_buffer[8];
				 
				 car_data1.arm_op_state =(RX_buffer[9]&0x0C)>>2;
				 car_data1.arm_state =RX_buffer[9]&0x03;
				 
				 car_data1.ultra_sonic_distance = (RX_buffer[10]+((RX_buffer[11]&0x000000ff)<<8))*8;
				 
				 car_data1.ir_data = RX_buffer[12];
				 
				 car_data1.qr_code_data = RX_buffer[13];
				 
				 car_data1.car_speed4 = RX_buffer[14];
				 car_data1.car_speed3 = RX_buffer[15];
				 car_data1.car_speed2 = RX_buffer[16];
				 car_data1.car_speed1 = RX_buffer[17];
				 
				 car_data1.car_state = RX_buffer[18]&0x0F;
				 
				 car_data1.transport_done_flag = RX_buffer[18]>>4;
				 
				 car_data1.motor_state = RX_buffer[19];
				 
				 
			 }
				 
     }
	
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	uint8_t ch;
	int8_t index; 
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
		if (__HAL_UART_GET_FLAG( &huart3, UART_FLAG_RXNE ) != RESET)
    {
			 ch=(uint16_t)READ_REG(huart3.Instance->DR);
			 for(index=1; index<=19; index++)
			 {
				 RX_buffer_sub[index-1] = RX_buffer_sub[index];
			 }
			 RX_buffer_sub[19]=ch;
			 if((RX_buffer_sub[4]==0xff)&&(RX_buffer_sub[3]==0xff)&&(RX_buffer_sub[2]==0x00))
			 {
				 car_data2.arm_angle4 = RX_buffer_sub[5];
				 car_data2.arm_angle3 = RX_buffer_sub[6];
				 car_data2.arm_angle2 = RX_buffer_sub[7];
				 car_data2.arm_angle1 = RX_buffer_sub[8];
				 
				 car_data2.arm_op_state =(RX_buffer_sub[9]&0x0C)>>2;
				 car_data2.arm_state =RX_buffer_sub[9]&0x03;
				 
				 car_data2.ultra_sonic_distance = (RX_buffer_sub[10]+((RX_buffer_sub[11]&0x000000ff)<<8))*8;
				 
				 car_data2.ir_data = RX_buffer_sub[12];
				 
				 car_data2.qr_code_data = RX_buffer_sub[13];
				 
				 car_data2.car_speed4 = RX_buffer_sub[14];
				 car_data2.car_speed3 = RX_buffer_sub[15];
				 car_data2.car_speed2 = RX_buffer_sub[16];
				 car_data2.car_speed1 = RX_buffer_sub[17];
				 
				 car_data2.car_state = RX_buffer_sub[18]&0x0F;
				 
				 car_data2.transport_done_flag = RX_buffer_sub[18]>>4;
				 
				 car_data2.motor_state = RX_buffer_sub[19];
			 }
			 
		 }

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
