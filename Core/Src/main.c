/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
uint32_t count;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Stepper_Forward(uint16_t n);
void Stepper_Backward(uint16_t n);
void Servo_Forward(uint16_t C,uint16_t T);
void Servo_Backward(uint16_t C,uint16_t T);
void Robot_Init(void);

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_GPIO_WritePin(GPIOA, EN_Pin, GPIO_PIN_RESET);
  
  Robot_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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

/* USER CODE BEGIN 4 */

/**
  * @brief  This function is for stepper forward
  * @param  n is the number of loop #uint16_t
  * @retval None
  */

void Stepper_Forward(uint16_t n) 
{
	   HAL_GPIO_WritePin(GPIOA, DIR_Pin,GPIO_PIN_RESET);
	   HAL_Delay(1);
		  
		for(count = 0; count < n; count++){
			HAL_GPIO_WritePin(GPIOA, STP_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA, STP_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
		}
}

/**
  * @brief  This function is for stepper backword
  * @param  n is the number of loop #uint16_t
  * @retval None
  */

void Stepper_Backward(uint16_t n) 
{
	   HAL_GPIO_WritePin(GPIOA, DIR_Pin,GPIO_PIN_SET);
	   HAL_Delay(1);
		  
		for(count = 0; count < n; count++){
			HAL_GPIO_WritePin(GPIOA, STP_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA, STP_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
		}
}

/**
  * @brief  This function is for servo forword
  * @param  C is the channel of tim1 && T is the time for delay #uint16_t
  * @retval None
  */

void Servo_Forward(uint16_t C,uint16_t T)
{	
	if(C == 1)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 130);
		HAL_Delay(T);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 0);	
		
	}else if(C == 2)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 130);
		HAL_Delay(T);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 0);	
	}else if(C == 3)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 130);
		HAL_Delay(T);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 0);	
	}
	
}

/**
  * @brief  This function is for servo backward
  * @param  t is the time for delay #uint16_t
  * @retval None
  */

void Servo_Backward(uint16_t C,uint16_t T)
{
	if(C == 1)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 170);
		HAL_Delay(T);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 0);
	}else if(C == 2)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 170);
		HAL_Delay(T);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 0);
	}
	else if(C == 3)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 170);
		HAL_Delay(T);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 0);
	}
	
}

/**
  * @brief  This function is for robot init its state
  * @param  None
  * @retval None
  */

void Robot_Init(void)
{
	for(int i = 0; i < 20; i++)
	{
		Servo_Forward(1, 10);
		Servo_Forward(2, 10);
		
	}	
	
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
