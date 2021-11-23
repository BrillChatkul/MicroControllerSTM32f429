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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t hex1 = 501;
int adc_avg_8;
int adc_avg_16;
//volatile uint32_t adc_val = 0;
volatile uint32_t adc_val[2];
volatile int ConversionComplete = 0;
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
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  //	  Number 1
//	  uint32_t hex1 = 501;
//	  displayHEX(hex1);
//	  while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==RESET){}
//	  HAL_UART_Transmit(&huart1, (uint8_t *) "\r\n", 4, 100);
//	  HAL_Delay(400);

	  // Number 3
//
//	  HAL_ADC_Start(&hadc1);
//      while(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK){}
//	  adc_val = HAL_ADC_GetValue(&hadc1);
//	  adc_avg_8 = average_8(adc_val);
//	  adc_avg_16 = average_16(adc_val);
//
//	  HAL_UART_Transmit(&huart1, (unsigned char*)"ADC1_CH10 ", 10, 100);
//	  displayHEX(adc_val);
//	  HAL_UART_Transmit(&huart1, (unsigned char*)" Vin = ", 7, 100);
//	  char tmp[10];
//	  int a = sprintf(tmp, "%.2f", adc_val * 3.33 / 4096);
//	  HAL_UART_Transmit(&huart1, (unsigned char*)tmp, a, 100);
//	  HAL_UART_Transmit(&huart1, (unsigned char*)"V\r\n", 3, 100);
//	  HAL_Delay(400);

//	  //Number 5
	  adc_val[0] = 0;
	  adc_val[1] = 0;
	  HAL_ADC_Start_DMA(&hadc1,adc_val,2);
	  while(ConversionComplete==0){}
	  ConversionComplete = 0;
	  adc_val[0] += 10;
	  adc_val[1] += 10;
	  HAL_UART_Transmit(&huart1, (unsigned char*)"ADC1_CH10 ", 10, 100);
	  displayHEX(adc_val[0]);
	  HAL_UART_Transmit(&huart1, (unsigned char*)"--------", 3, 100);
	  HAL_UART_Transmit(&huart1, (unsigned char*)"ADC1_CH11 ", 10, 100);
	  displayHEX(adc_val[1]);
	  HAL_UART_Transmit(&huart1, (unsigned char*)"\r\n", 3, 100);
	  HAL_Delay(400);

//	  // Number 4
//
//	  	  HAL_ADC_Start(&hadc1);
//	        while(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK){}
//	  	  adc_val = HAL_ADC_GetValue(&hadc1);
//	  	  adc_avg_8 = average_8(adc_val);
//	  	  adc_avg_16 = average_16(adc_val);
//	  	  HAL_UART_Transmit(&huart1, (unsigned char*)"ADC1_CH10 ", 10, 100);
//	  	  displayHEX(adc_val);
//	  	  HAL_UART_Transmit(&huart1, (unsigned char*)" Vin = ", 7, 100);
//	  	  char tmp[10];
//	  	  int a = sprintf(tmp, "%.2f", adc_val * 3.33 / 4096);
//	  	  HAL_UART_Transmit(&huart1, (unsigned char*)tmp, a, 100);
//	  	  HAL_UART_Transmit(&huart1, (unsigned char*)"V", 1, 100);
//	  	  HAL_Delay(400);
//
//	  	//level 1
//	  	if(adc_val<= 820){
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
//	  		HAL_UART_Transmit(&huart1, (unsigned char*)" Level 1  \r\n", 15, 100);
//	  	}
//	  	//level 2
//	  	else if(adc_val<= 1640 && adc_val>820){
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
//	  		HAL_UART_Transmit(&huart1, (unsigned char*)" Level 2  \r\n", 15, 100);
//	  	}
//	  	//level 3
//	  	else if(adc_val<= 2460 && adc_val>1640){
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
//	  		HAL_UART_Transmit(&huart1, (unsigned char*)" Level 3  \r\n", 15, 100);
//	  	}
//	  	//level 4
//	  	else if(adc_val<= 3200 && adc_val>2460){
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
//	  		HAL_UART_Transmit(&huart1, (unsigned char*)" Level 4  \r\n", 15, 100);
//	  	}
//	  	//level 5
//	  	else{
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
//	  		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
//	  		HAL_UART_Transmit(&huart1, (unsigned char*)" Level 5  \r\n", 15, 100);
//	  	}

//
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
void displayHEX(uint32_t hex){
	char hex_val[10];
	sprintf(hex_val, "0x%08X", hex);
	while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==RESET){}
	HAL_UART_Transmit(&huart1, (unsigned char*) hex_val, 10,1000);

}
int average_8(int x){
	static int samples[8];
	static int i = 0;
	static int total = 0;

	/* Update the moving average	 */
	total += x - samples[i];
	samples[i] = x;

	/*Update the index	 */
	i = (i==7 ? 0: i+1);
	return total>>3;
}

int average_16(int x){
	static int samples[16];
	static int i = 0;
	static int total = 0;

	/* Update the moving average	 */
	total += x - samples[i];
	samples[i] = x;

	/*Update the index	 */
	i = (i==15 ? 0: i+1);
	return total>>4;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	ConversionComplete = 1;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
