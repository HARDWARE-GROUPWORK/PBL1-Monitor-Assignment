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
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "ILI9341_Touchscreen.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include "snow_tiger.h"

#include "am2320.h"

#include "stdbool.h"
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
/* Private variables ---------------------------------------------------------*/
int numberOfRecord = 1;
int lightPercent = 100;
float temp = 99.9;
float humid = 99.9;

char Temp_Buffer_text[40];

//Horizontal Screen
uint16_t maxWidth = 200; //300 //Left 50 - right 50
uint16_t offsetWidth = 60;
uint16_t maxHeight = 240;

uint16_t mode = 0;
uint16_t modeEdit = 1;
uint16_t prevMode = -1;
uint16_t prevModeEdit = -1;

uint16_t secondCounter = 0;
uint16_t prevSecondCounter = 0;
uint32_t millisecondHAL = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint16_t CRC16_2(uint8_t *, uint8_t );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Paint screen black
void setHorizontalScreen(uint16_t color){
	ILI9341_Fill_Screen(color);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
}

// Print text white font with black background
void printText(char arr[],int line,int offset,int size){ // text, line, offset, size
	if (line == 0){ // Start with line 1,2,3,4,5...
		line = 1;
	}else{
		line -=1;
	}
	ILI9341_Draw_Text(arr, 10, 10+(offset*line), WHITE, size, BLACK);
}

void newLine(void){
	char newLine[20] = "\n\r";
	HAL_UART_Transmit(&huart3, (uint8_t*) newLine, strlen(newLine), 1000);
}

char str[50];
uint8_t cmdBuffer[3];
uint8_t dataBuffer[8];


void assignmentOne(){

	setHorizontalScreen(BLACK);

	//Temperature
	cmdBuffer[0] = 0x03;
	cmdBuffer[1] = 0x00;
	cmdBuffer[2] = 0x04;

	//Send Temp & Humid via UART2
	sprintf(str, "Temperature = %4.1f\tHumidity = %4.1f\n\r", temp, humid);
	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
	HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);

	//HAL_Delay(5000); //>3000 ms
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

	//Wake up sensor
	HAL_I2C_Master_Transmit(&hi2c1, 0x5c<<1, cmdBuffer, 3, 200);
	//Send reading command
	HAL_I2C_Master_Transmit(&hi2c1, 0x5c<<1, cmdBuffer, 3, 200);

	//HAL_Delay(1);

	//Receive sensor data
	HAL_I2C_Master_Receive(&hi2c1, 0x5c<<1, dataBuffer, 8, 200);

	uint16_t Rcrc = dataBuffer[7] << 8;
	Rcrc += dataBuffer[6];
	if (Rcrc == CRC16_2(dataBuffer, 6)) {
		uint16_t temperature = ((dataBuffer[4] & 0x7F) << 8) + dataBuffer[5];
		temp = temperature / 10.0;
		temp = (((dataBuffer[4] & 0x80) >> 7)== 1) ? (temp * (-1)) : temp ; // the temperature can be negative

		uint16_t humidity = (dataBuffer[2] << 8) + dataBuffer[3];
		humid = humidity / 10.0;
	}

	//Record
	sprintf(Temp_Buffer_text, "Record %05d", numberOfRecord);
	HAL_UART_Transmit(&huart3, (uint8_t*) Temp_Buffer_text, strlen(Temp_Buffer_text), 1000);
	newLine();
	printText(Temp_Buffer_text,1,20,3);

	//Light
	sprintf(Temp_Buffer_text, "Light %d %%", lightPercent);
	HAL_UART_Transmit(&huart3, (uint8_t*) Temp_Buffer_text, strlen(Temp_Buffer_text), 1000);
	newLine();
	printText(Temp_Buffer_text,2,20,3);

	//Temperature
	sprintf(Temp_Buffer_text, "Temp %0.1f C", temp);
	HAL_UART_Transmit(&huart3, (uint8_t*) Temp_Buffer_text, strlen(Temp_Buffer_text), 1000);
	newLine();
	printText(Temp_Buffer_text,3,20,3);

	//Humidity
	sprintf(Temp_Buffer_text, "Humid  %0.1f %%", humid);
	HAL_UART_Transmit(&huart3, (uint8_t*) Temp_Buffer_text, strlen(Temp_Buffer_text), 1000);
	newLine();
	printText(Temp_Buffer_text,4,20,3);

	newLine();
	HAL_Delay(1000);	// refresh every 1 second
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_USART3_UART_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  sprintf(str, "\n\rAM2320 I2C DEMO Starting . . .\n\r");

  HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);

  //initial driver setup to drive ili9341
  ILI9341_Init();


  //Interrupt millisecond
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);


  //Reset Screen
  setHorizontalScreen(BLACK);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  assignmentOne();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t CRC16_2(uint8_t *ptr, uint8_t length)
{
      uint16_t 	crc = 0xFFFF;
      uint8_t 	s 	= 0x00;

      while(length--) {
        crc ^= *ptr++;
        for(s = 0; s < 8; s++) {
          if((crc & 0x01) != 0) {
            crc >>= 1;
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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
