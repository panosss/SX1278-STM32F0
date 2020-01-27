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
#include "SX1278.h"
#include "spi.h"
#include "usart.h"
#include "serial.h"
#include "gpio.h"
#include "stdint.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

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
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uintptr_t) ptr, len, 50);  // uintptr_t  uint8_t
	return len;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

int master;
int ret;
char buffer[100];
int message;
int message_length;

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	master = 1;
	printf("Mode: Master\r\n");

	//initialize LoRa module
	/*
	 LED_GPIO_Port GPIOA
	 LED_Pin GPIO_PIN_0     LED=PA0
	 NSS_GPIO_Port GPIOA
	 NSS_Pin GPIO_PIN_4     NSS=PA4
	 DIO0_GPIO_Port GPIOA
	 DIO0_Pin GPIO_PIN_10   DIO0=PA10
	 RESET_GPIO_Port GPIOA
	 RESET_Pin GPIO_PIN_9   RESET =PA9
	 MODE_GPIO_Port GPIOB            // I have no mode pin
	 MODE_Pin GPIO_PIN_1    MODE=PB1 // I have no mode pin
	 */
	SX1278_hw.dio0.port = DIO0_GPIO_Port;
	SX1278_hw.dio0.pin = DIO0_Pin;
	SX1278_hw.nss.port = NSS_GPIO_Port;
	SX1278_hw.nss.pin = NSS_Pin;
	SX1278_hw.reset.port = RESET_GPIO_Port;
	SX1278_hw.reset.pin = RESET_Pin;
	SX1278_hw.spi = &hspi1;

	SX1278.hw = &SX1278_hw;


	//SX1278_setSyncWord(&SX1278, 0x12);

	printf("Configuring LoRa module\r\n");
	// SX1278_begin(&SX1278, SX1278_433MHZ, SX1278_POWER_20DBM, SX1278_LORA_SF_7,
	//      SX1278_LORA_BW_125KHZ, 2);
	//SX1278_begin(&SX1278, 433E6, SX1278_POWER_17DBM, SX1278_LORA_SF_7,
	//	   SX1278_LORA_BW_125KHZ, 28);
	// SX1278_LORA_BW_62_5KHZ = 6. Arduino: long Bandwidth = 62.5E3;
	// bw=20.8kHz=3. Arduino: long Bandwidth = 20.8E3;
	// packetLength = 10

	printf("Done configuring LoRaModule\r\n");

	ret = LoRa_begin(&SX1278, 433E6);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		printf("Master ...\r\n");
		HAL_Delay(2500);
		printf("Sending package...\r\n");

		message_length = sprintf(buffer, "I say HI and Hello %d", message);
		ret = LoRa_beginPacket(&SX1278, 0);HAL_Delay(200);
		ret = LoRa_write_b(&SX1278, (uint8_t *) buffer, message_length);HAL_Delay(200);
		//printf("1.mainc RegFIFO = %s\r\n", LoRa_readRegister(&SX1278, RegFIFO));

		HAL_GPIO_WritePin(SX1278.hw->dio0.port, SX1278.hw->dio0.pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(SX1278.hw->dio0.port, SX1278.hw->dio0.pin, GPIO_PIN_RESET);

		LoRa_endPacket(&SX1278, false);HAL_Delay(200);



		message += 1;

		printf("Transmission: %d\r\n", ret);
		printf("Package %i sent...\r\n", message);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
