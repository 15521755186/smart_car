/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
#include "delay.h"
#include "lcd.h"
#include "stdio.h"
#include "stm32f4xx_it.h"
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
uint8_t a=1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	//u8 lcd_id[12];				//���LCD ID�ַ���


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
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	delay_init(168);
	LCD_Init();
	LCD_Display_Dir(1);	
	POINT_COLOR=BLACK;
	LCD_Clear(WHITE);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

POINT_COLOR=BLUE;
Chinese_Show_one(0,0,0,24,0);
Chinese_Show_one(24,0,1,24,0);
	
POINT_COLOR=BLACK;
Chinese_Show_one(16,96+32,9,16,0);
Chinese_Show_one(32,96+32,10,16,0);
Chinese_Show_one(48,96+32,13,16,0);

Chinese_Show_one(16,160+48,11,16,0);
Chinese_Show_one(32,160+48,12,16,0);
Chinese_Show_one(48,160+48,13,16,0);

Chinese_Show_one(16,224+64,19,16,0);
Chinese_Show_one(32,224+64,20,16,0);
Chinese_Show_one(48,224+64,21,16,0);

Chinese_Show_one(200,32,0,16,0);
Chinese_Show_one(216,32,2,16,0);
Chinese_Show_one(264,32,25,16,0);

Chinese_Show_one(360,32,0,16,0);
Chinese_Show_one(376,32,3,16,0);
Chinese_Show_one(422,32,25,16,0);

Chinese_Show_one(200,316,1,16,0);
Chinese_Show_one(216,316,2,16,0);
Chinese_Show_one(264,316,25,16,0);

Chinese_Show_one(360,316,1,16,0);
Chinese_Show_one(376,316,3,16,0);
Chinese_Show_one(422,316,25,16,0);

LCD_ShowString(16,304,144,16,16,"(.01mm)");

Chinese_Show_one(360-80,400,0,24,0);
Chinese_Show_one(384-80,400,1,24,0);
Chinese_Show_one(408-80,400,19,24,0);
Chinese_Show_one(432-80,400,20,24,0);

Chinese_Show_one(360-110+324,400,2,24,0);
Chinese_Show_one(384-110+324,400,3,24,0);
Chinese_Show_one(408-110+324,400,4,24,0);
Chinese_Show_one(432-110+324,400,19,24,0);
Chinese_Show_one(432-86+324,400,20,24,0);


show_pic_car(224,64);
show_pic_arm(548,64);

LCD_ShowString(384-110+324,310,32,16,16,"2:");
LCD_ShowString(432-76+324,235,32,16,16,"1:");
LCD_ShowString(360-110+324,32,32,16,16,"4:");
LCD_ShowString(432-86+324,32,32,16,16,"3:");

Chinese_Show_one(384-110+324+64,310,22,16,0);
Chinese_Show_one(432-76+324+64,235,22,16,0);
Chinese_Show_one(360-110+324+64,32,22,16,0);
Chinese_Show_one(432-86+324+64,32,22,16,0);

  while (1)
  {	  
		while(current_car==0)
		{
			POINT_COLOR=BLUE;
			LCD_ShowString(48,0,24,24,24,"1");
			POINT_COLOR=RED;		
			LCD_ShowNum(80,128,car_data1.ir_data,3,16);
		  LCD_ShowNum(80,192+16,car_data1.qr_code_data,3,16);
		  LCD_ShowNum(80,256+32,car_data1.ultra_sonic_distance,6,16);
			
			LCD_ShowNum(296,32,car_data1.car_speed1,3,16);
			LCD_ShowNum(438,32,car_data1.car_speed2,3,16);
			LCD_ShowNum(296,316,car_data1.car_speed3,3,16);
			LCD_ShowNum(438,316,car_data1.car_speed4,3,16);
			
			LCD_ShowNum(296,32,car_data1.car_speed1,3,16);
			LCD_ShowNum(438,32,car_data1.car_speed2,3,16);
			LCD_ShowNum(296,316,car_data1.car_speed3,3,16);
			LCD_ShowNum(438,316,car_data1.car_speed4,3,16);
			
			LCD_ShowNum(384-110+324+32,310,car_data1.arm_angle2,3,16);
			LCD_ShowNum(432-76+324+32,235,car_data1.arm_angle1,3,16);
			LCD_ShowNum(360-110+324+32,32,car_data1.arm_angle4,3,16);
			LCD_ShowNum(432-86+324+32,32,car_data1.arm_angle2,3,16);
			
			switch(car_data1.motor_state)
			{
				case 0xAA:LCD_Fill(248,32,264,48,WHITE);LCD_Fill(248,316,264,332,WHITE);LCD_Fill(376+32,32,376+48,48,WHITE);LCD_Fill(376+32,316,376+48,332,WHITE);POINT_COLOR=GREEN;Chinese_Show_one(248,32,23,16,0);Chinese_Show_one(376+32,32,23,16,0);Chinese_Show_one(216+32,316,23,16,0);Chinese_Show_one(376+32,316,23,16,0);break;
				case 0x99:LCD_Fill(248,32,264,48,WHITE);LCD_Fill(248,316,264,332,WHITE);LCD_Fill(376+32,32,376+48,48,WHITE);LCD_Fill(376+32,316,376+48,332,WHITE);POINT_COLOR=GREEN;Chinese_Show_one(248,32,23,16,0);POINT_COLOR=RED;Chinese_Show_one(376+32,32,24,16,0);POINT_COLOR=GREEN;Chinese_Show_one(216+32,316,23,16,0);POINT_COLOR=RED;Chinese_Show_one(376+32,316,24,16,0);break;
				case 0x55:LCD_Fill(248,32,264,48,WHITE);LCD_Fill(248,316,264,332,WHITE);LCD_Fill(376+32,32,376+48,48,WHITE);LCD_Fill(376+32,316,376+48,332,WHITE);POINT_COLOR=RED;Chinese_Show_one(248,32,24,16,0);POINT_COLOR=GREEN;Chinese_Show_one(376+32,32,23,16,0);POINT_COLOR=RED;Chinese_Show_one(216+32,316,24,16,0);POINT_COLOR=GREEN;Chinese_Show_one(376+32,316,23,16,0);break;
				case 0x00:LCD_Fill(248,32,264,48,WHITE);LCD_Fill(248,316,264,332,WHITE);LCD_Fill(376+32,32,376+48,48,WHITE);LCD_Fill(376+32,316,376+48,332,WHITE);POINT_COLOR=BLACK;Chinese_Show_one(248,32,26,16,0);Chinese_Show_one(376+32,32,26,16,0);Chinese_Show_one(216+32,316,26,16,0);Chinese_Show_one(376+32,316,26,16,0);break;
				default:break;
			}
			POINT_COLOR=BLACK;
			if(car_data1.arm_state==1){LCD_Fill(360-110+324,400+24,700,480,WHITE);POINT_COLOR=RED;Chinese_Show_one(360-110+324,400+24,14,24,0);Chinese_Show_one(360-110+324+24,400+24,15,24,0);}
			else if(car_data1.arm_state==2){LCD_Fill(360-110+324,400+24,700,480,WHITE);POINT_COLOR=RED;Chinese_Show_one(360-110+324,400+24,16,24,0);Chinese_Show_one(360-110+324+24,400+24,17,24,0);}
			else if(car_data1.arm_op_state== car_data1.arm_state){LCD_Fill(360-110+324+48,400+24,700,480,WHITE);POINT_COLOR=GREEN;Chinese_Show_one(360-110+324+48,400+24,6,24,0);Chinese_Show_one(360-110+324+72,400+24,7,24,0);}
			else{}
			POINT_COLOR=BLACK;
				
			if((car_data1.car_state==0)&&(car_data1.transport_done_flag==1)){LCD_Fill(360-80,400+24,550,480,WHITE);POINT_COLOR=GREEN;Chinese_Show_one(360-80,400+24,6,24,0);Chinese_Show_one(384-80,400+24,7,24,0);Chinese_Show_one(408-80,400+24,8,24,0);Chinese_Show_one(432-80,400+24,9,24,0);}
			else if(car_data1.car_state==0){LCD_Fill(360-80,400+24,550,480,WHITE);POINT_COLOR=RED;Chinese_Show_one(360-80,400+24,21,24,0);Chinese_Show_one(384-80,400+24,22,24,0);}
			else{LCD_Fill(360-80,400+24,550,480,WHITE);POINT_COLOR=BLACK;Chinese_Show_one(360-80,400+24,8,24,0);Chinese_Show_one(384-80,400+24,9,24,0);Chinese_Show_one(408-80,400+24,18,24,0);}
			POINT_COLOR=BLACK;
			delay_ms(150);
		} 
		while(current_car==1)
		{
			POINT_COLOR=BLUE;
			LCD_ShowString(48,0,24,24,24,"2");
			POINT_COLOR=RED;		
			LCD_ShowNum(80,128,car_data2.ir_data,3,16);
		  LCD_ShowNum(80,192+16,car_data2.qr_code_data,3,16);
		  LCD_ShowNum(80,256+32,car_data2.ultra_sonic_distance,6,16);
			
			LCD_ShowNum(296,32,car_data2.car_speed1,3,16);
			LCD_ShowNum(438,32,car_data2.car_speed2,3,16);
			LCD_ShowNum(296,316,car_data2.car_speed3,3,16);
			LCD_ShowNum(438,316,car_data2.car_speed4,3,16);
			
			LCD_ShowNum(296,32,car_data2.car_speed1,3,16);
			LCD_ShowNum(438,32,car_data2.car_speed2,3,16);
			LCD_ShowNum(296,316,car_data2.car_speed3,3,16);
			LCD_ShowNum(438,316,car_data2.car_speed4,3,16);
			
			LCD_ShowNum(384-110+324+32,310,car_data2.arm_angle2,3,16);
			LCD_ShowNum(432-76+324+32,235,car_data2.arm_angle1,3,16);
			LCD_ShowNum(360-110+324+32,32,car_data2.arm_angle4,3,16);
			LCD_ShowNum(432-86+324+32,32,car_data2.arm_angle2,3,16);
			
			switch(car_data2.motor_state)
			{
				case 0xAA:LCD_Fill(248,32,264,48,WHITE);LCD_Fill(248,316,264,332,WHITE);LCD_Fill(376+32,32,376+48,48,WHITE);LCD_Fill(376+32,316,376+48,332,WHITE);POINT_COLOR=GREEN;Chinese_Show_one(248,32,23,16,0);Chinese_Show_one(376+32,32,23,16,0);Chinese_Show_one(216+32,316,23,16,0);Chinese_Show_one(376+32,316,23,16,0);break;
				case 0x99:LCD_Fill(248,32,264,48,WHITE);LCD_Fill(248,316,264,332,WHITE);LCD_Fill(376+32,32,376+48,48,WHITE);LCD_Fill(376+32,316,376+48,332,WHITE);POINT_COLOR=GREEN;Chinese_Show_one(248,32,23,16,0);POINT_COLOR=RED;Chinese_Show_one(376+32,32,24,16,0);POINT_COLOR=GREEN;Chinese_Show_one(216+32,316,23,16,0);POINT_COLOR=RED;Chinese_Show_one(376+32,316,24,16,0);break;
				case 0x55:LCD_Fill(248,32,264,48,WHITE);LCD_Fill(248,316,264,332,WHITE);LCD_Fill(376+32,32,376+48,48,WHITE);LCD_Fill(376+32,316,376+48,332,WHITE);POINT_COLOR=RED;Chinese_Show_one(248,32,24,16,0);POINT_COLOR=GREEN;Chinese_Show_one(376+32,32,23,16,0);POINT_COLOR=RED;Chinese_Show_one(216+32,316,24,16,0);POINT_COLOR=GREEN;Chinese_Show_one(376+32,316,23,16,0);break;
				case 0x00:LCD_Fill(248,32,264,48,WHITE);LCD_Fill(248,316,264,332,WHITE);LCD_Fill(376+32,32,376+48,48,WHITE);LCD_Fill(376+32,316,376+48,332,WHITE);POINT_COLOR=BLACK;Chinese_Show_one(248,32,26,16,0);Chinese_Show_one(376+32,32,26,16,0);Chinese_Show_one(216+32,316,26,16,0);Chinese_Show_one(376+32,316,26,16,0);break;
				default:break;
			}
			POINT_COLOR=BLACK;
			if(car_data2.arm_state==1){LCD_Fill(360-110+324,400+24,700,480,WHITE);POINT_COLOR=RED;Chinese_Show_one(360-110+324,400+24,14,24,0);Chinese_Show_one(360-110+324+24,400+24,15,24,0);}
			else if(car_data2.arm_state==2){LCD_Fill(360-110+324,400+24,700,480,WHITE);POINT_COLOR=RED;Chinese_Show_one(360-110+324,400+24,16,24,0);Chinese_Show_one(360-110+324+24,400+24,17,24,0);}
			else if(car_data2.arm_op_state== car_data2.arm_state){LCD_Fill(360-110+324+48,400+24,700,480,WHITE);POINT_COLOR=GREEN;Chinese_Show_one(360-110+324+48,400+24,6,24,0);Chinese_Show_one(360-110+324+72,400+24,7,24,0);}
			else{}
			POINT_COLOR=BLACK;
				
			if((car_data2.car_state==0)&&(car_data2.transport_done_flag==1)){LCD_Fill(360-80,400+24,550,480,WHITE);POINT_COLOR=GREEN;Chinese_Show_one(360-80,400+24,6,24,0);Chinese_Show_one(384-80,400+24,7,24,0);Chinese_Show_one(408-80,400+24,8,24,0);Chinese_Show_one(432-80,400+24,9,24,0);}
			else if(car_data2.car_state==0){LCD_Fill(360-80,400+24,550,480,WHITE);POINT_COLOR=RED;Chinese_Show_one(360-80,400+24,21,24,0);Chinese_Show_one(384-80,400+24,22,24,0);}
			else{LCD_Fill(360-80,400+24,550,480,WHITE);POINT_COLOR=BLACK;Chinese_Show_one(360-80,400+24,8,24,0);Chinese_Show_one(384-80,400+24,9,24,0);Chinese_Show_one(408-80,400+24,18,24,0);}
			POINT_COLOR=BLACK;
			delay_ms(150);
			
		}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
