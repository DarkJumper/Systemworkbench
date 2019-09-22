/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DELAY_FAST 50
#define DELAY_SLOW 1000
#define LEDS 9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void led(int number, int Zustand)
{
	switch(number)
	{
	case 1:
		  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,Zustand);
		  break;
	case 2:
		  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,Zustand);
		  break;
	case 3:
		  HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,Zustand);
		  break;
	case 4:
		  HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,Zustand);
		  break;
	case 5:
		  HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,Zustand);
		  break;
	case 6:
		  HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,Zustand);
		  break;
	case 7:
		  HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,Zustand);
		  break;
	case 8:
		  HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,Zustand);
		  break;
	}
}

void Blink()
{
	  HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	  HAL_Delay(DELAY_SLOW);
}

void Lampen_Test(int i)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,i);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,i);
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,i);
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,i);
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,i);
	HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,i);
	HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,i);
	HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,i);
	HAL_Delay(DELAY_SLOW);
}
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  Blink();

	  while(HAL_GPIO_ReadPin(Taster1_GPIO_Port,Taster1_Pin))
	  {
		  int a = 1;
		  Lampen_Test(a);
		  if(!HAL_GPIO_ReadPin(Taster1_GPIO_Port,Taster1_Pin))
		  {
			  a = 0;
			  Lampen_Test(a);
		  }
	  }


	  while(HAL_GPIO_ReadPin(Taster2_GPIO_Port,Taster2_Pin))
	  {
		  for (int i=1; i<LEDS; i++)
		  {

			  led(i,1);
			  HAL_Delay(DELAY_FAST);
		  }
		  for (int i=LEDS; i>1; i--)
		  {
			  led(i,0);
			  HAL_Delay(DELAY_FAST);
		  }
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
