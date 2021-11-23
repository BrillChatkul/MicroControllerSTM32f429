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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
float h=30.0,t=40.0;

uint8_t Buffer[25] = {0};
uint8_t Space[] = " - ";
uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t CRC16_2(uint8_t *, uint8_t);
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
	char str[50];
	uint8_t cmdBuffer[3] = { 0x03, 0x00, 0x04 };
	uint8_t dataBuffer[8];

	uint8_t i = 0, ret;
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
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, StartMSG, sizeof(StartMSG), 10000);
  for(i=1; i<128; i++)
  {
      ret = HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(i<<1), 3, 5);
      if (ret != HAL_OK) /* No ACK Received At That Address */
      {
          HAL_UART_Transmit(&huart1, Space, sizeof(Space), 10000);
      }
      else if(ret == HAL_OK)
      {
          sprintf(Buffer, "0x%X", i);
          HAL_UART_Transmit(&huart1, Buffer, sizeof(Buffer), 10000);
      }
  }
  HAL_UART_Transmit(&huart1, EndMSG, sizeof(EndMSG), 10000);


  sprintf(str, "\n\rAM2320 I2C DEMO Starting ...\n\r");
  HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 200);
//  cmdBuffer[0] = 0x03;
//  cmdBuffer[1] = 0x00;
//  cmdBuffer[2] = 0x04;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  sprintf(str, "Temperature = %4.1f\tHumidity = %4.1f\n\r", t, h);
	  while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==RESET){}
	  HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 200);

	  HAL_Delay(5000);
	  HAL_I2C_Master_Transmit(&hi2c3, 0x5C<<1, 0x00, 0, HAL_MAX_DELAY);
	  HAL_Delay(1);
	  HAL_I2C_Master_Transmit(&hi2c3, 0x5C<<1, cmdBuffer, 3, HAL_MAX_DELAY);
	  HAL_Delay(1);
	  HAL_I2C_Master_Receive(&hi2c3, 0x5C<<1,dataBuffer, 8, HAL_MAX_DELAY);
	  uint16_t Rcrc = dataBuffer[7] << 8;
	  Rcrc += dataBuffer[6];
	  if(Rcrc == CRC16_2(dataBuffer, 6)){
		  uint16_t temperature = ((dataBuffer[4] & 0x7F) << 8) + dataBuffer[5];
		  t =  temperature / 10.0;
		  t = (((dataBuffer[4] & 0x80) >> 7)==1)?(t*(-1)) :t;

		  uint16_t humidity = (dataBuffer[2] << 8) + dataBuffer[3];
		  h = humidity / 10.0;
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 120;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t CRC16_2(uint8_t *ptr, uint8_t length)
{
	uint16_t crc = 0xFFFF;
	uint8_t s = 0x00;

	while(length--){
		crc ^= *ptr++;
		for(s = 0;s<8;s++){
			if((crc & 0x01) != 0){
				crc >> 1;
				crc ^= 0xA001;
			} else crc >>= 1;
		}
	}
	return crc;
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
