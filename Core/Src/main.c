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

#include "EEPROM.h"
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

bool initialState = false;

//Values on Screen
uint32_t numberOfRecord = 0; //index
uint8_t lightPercent = 100;
float temp = 99.0;
float humid =  99.0;

uint32_t prevNumberOfRecord = 0;
uint8_t prevLightPercent = 0;
float prevTemp = 0.0;
float prevHumid = 0.0;

//Buffer
uint32_t numberOfRecordBuffer[20] = {0}; //index

//Average Buffer 20 values (1 value/500ms for 10 s)
uint8_t lightPercentBuffer[20] = {0};
float tempBuffer[20] = {0.0};
float humidBuffer[20] = {0.0};

char Temp_Buffer_text[40];

//ADC
volatile uint32_t adc_val = 0;

//Button
bool pressButton1 = 0;
bool pressButton2 = 0;
bool pressButton3 = 0;
bool pressButton4 = 0;

//Mode
uint8_t mode = 0; // default 0, average 1
int8_t previousNum = 1; //avg page, negative index from now
int8_t prevPreviousNum = 0;

//Timer
uint64_t millisecondHAL = 0;
uint64_t prevMillisecondHAL = 0;

//Screen
uint32_t colorScreen = BLACK;
uint32_t prevColorScreen = BLACK;

//EEPROM AddressI2C = 0xa0


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint16_t CRC16_2(uint8_t *, uint8_t );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// EEPROM Delete
void eraseAllData(){
	for (int i=0; i<512; i++)
	{
	  EEPROM_PageErase(i);
	}
}

void saveAllData(){

	// Record 32 bit Page 1,2
	// Light 8 bit Page 3
	// Temp 32 bit Page 4,5
	// Humid 32 bit Page 6,7

	char str[100];
//	sprintf(str, "*******************\n\r");
//	HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);


	// Record 32 bit, #PAGE 1, 0-9, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		uint32_t value = numberOfRecordBuffer[i];
		EEPROM_Write_NUM(1, i*4, value);

//		sprintf(str, "%d WRITE1\n\r", value);
//		HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);
	}
	// Record 32 bit, #PAGE 2, 10-19, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		uint32_t value = numberOfRecordBuffer[i+10];
		EEPROM_Write_NUM(2, i*4, value);
//
//		sprintf(str, "%d WRITE2\n\r",value);
//		HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);
	}



	// Light is Ok
	EEPROM_Write(3, 0, lightPercentBuffer, 20);

	// Temp 32 bit, #PAGE 4, 0-9, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		float value = tempBuffer[i];
		EEPROM_Write_NUM(4, i*4, value);
	}
	// Temp 32 bit, #PAGE 5, 10-19, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		float value = tempBuffer[i+10];
		EEPROM_Write_NUM(5, i*4, value);
	}

	// Humid 32 bit, #PAGE 6, 0-9, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		float value = humidBuffer[i];
		EEPROM_Write_NUM(6, i*4, value);
	}
	// Humid 32 bit, #PAGE 7, 10-19, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		float value = humidBuffer[i+10];
		EEPROM_Write_NUM(7, i*4, value);
	}

}

void readAllData(){

	uint32_t numberOfRecordMax = 0;

	char str[100];
//	sprintf(str, "---------------------\n\r");
//	HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);


	// Record 32 bit, #PAGE 1, 0-9, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		numberOfRecordBuffer[i] = (uint32_t)EEPROM_Read_NUM(1, i*4);
		if(numberOfRecordBuffer[i] > numberOfRecordMax){
			numberOfRecordMax = numberOfRecordBuffer[i];
		}

//		sprintf(str, "%d READ1\n\r",numberOfRecordBuffer[i]);
//		HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);
	}
	// Record 32 bit, #PAGE 2, 10-19, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		numberOfRecordBuffer[i+10] = (uint32_t)EEPROM_Read_NUM(2, i*4);
		if(numberOfRecordBuffer[i+10] > numberOfRecordMax){
			numberOfRecordMax = numberOfRecordBuffer[i+10];
		}

//		sprintf(str, "%d READ2\n\r",numberOfRecordBuffer[i+10]);
//		HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);
	}

	// Update the Lastest
	numberOfRecord = numberOfRecordMax;

	// Light is Ok
	EEPROM_Read(3, 0, lightPercentBuffer, 20);

	// Temp 32 bit, #PAGE 4, 0-9, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		tempBuffer[i] = (uint32_t)EEPROM_Read_NUM(4, i*4);
	}
	// Temp 32 bit, #PAGE 5, 10-19, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		tempBuffer[i+10] = (uint32_t)EEPROM_Read_NUM(5, i*4);
	}
	// Humid 32 bit, #PAGE 6, 0-9, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		humidBuffer[i] = (uint32_t)EEPROM_Read_NUM(6, i*4);
	}
	// Humid 32 bit, #PAGE 7, 10-19, 10 amounts (40 bytes)
	for(uint8_t i = 0; i < 10; i++){
		humidBuffer[i+10] = (uint32_t)EEPROM_Read_NUM(7, i*4);
	}

}

// Paint screen black
void setHorizontalScreen(uint16_t color){
	ILI9341_Fill_Screen(color);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
	ILI9341_Draw_Filled_Rectangle_Coord(15, 15, 305, 225, BLACK);
}

// Print text white font with black background
void printText(char arr[],float line,int offset,int size, uint32_t color){ // text, line, offset, size
	line -=1;
	ILI9341_Draw_Text(arr, 35, 30+(offset*line), color, size, BLACK);
}

void printValue(char arr[],float line,int offset,int size, uint32_t color){
	line -=1;
	ILI9341_Draw_Text(arr, 150, 30+(offset*line), color, size, BLACK);
}

void newLine(void){
	char newLine[20] = "\n\r";
	HAL_UART_Transmit(&huart3, (uint8_t*) newLine, strlen(newLine), 1000);
}

char str[50];
uint8_t cmdBuffer[3];
uint8_t dataBuffer[8];


void tempMonitor(){
	//Temperature
	cmdBuffer[0] = 0x03;
	cmdBuffer[1] = 0x00;
	cmdBuffer[2] = 0x04;

	//Send Temp & Humid via UART3
//	sprintf(str, "Temperature = %4.1f\tHumidity = %4.1f\n\r", temp, humid);
//	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)==RESET){}
//	HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);

	//HAL_Delay(5000); //>3000 ms
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

	//Wake up sensor
	HAL_I2C_Master_Transmit(&hi2c1, 0x5c<<1, cmdBuffer, 3, 200);
	//Send reading command
	HAL_I2C_Master_Transmit(&hi2c1, 0x5c<<1, cmdBuffer, 3, 200);

	HAL_Delay(100); // 50 is too low, 100 is okay

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
}

void averageScreen(){
	uint8_t size = 3;
	uint8_t offset = 30;
	uint8_t n = 0;

	uint16_t averageLightPercent = 0;
	float averageTemp = 0.0;
	float averageHumid = 0.0;

	if(numberOfRecord <= 19){
		n = numberOfRecord;
	}else{
		n = 20;
	}

	for(uint8_t i=0; i<n; i++){
		averageLightPercent += lightPercentBuffer[i];
		averageTemp += tempBuffer[i];
		averageHumid += humidBuffer[i];

		sprintf(str, "%d %f %f / %d %f %f\n\r",lightPercentBuffer[i], tempBuffer[i], humidBuffer[i],averageLightPercent ,averageTemp, averageHumid);
		HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);
	}

	averageLightPercent /= n;
	averageTemp /= n;
	averageHumid /= n;

	//Average
	sprintf(Temp_Buffer_text, "Average 10 s");
	printText(Temp_Buffer_text,1.5,offset,size,GREENYELLOW);

	//Light
	sprintf(Temp_Buffer_text, "Light");
	printText(Temp_Buffer_text,3,offset,size,ORANGE);
	//Temperature
	sprintf(Temp_Buffer_text, "Temp");
	printText(Temp_Buffer_text,4,offset,size,PINK);
	//Humidity
	sprintf(Temp_Buffer_text, "Humid");
	printText(Temp_Buffer_text,5,offset,size,MAROON);

	//Update Light
	sprintf(Temp_Buffer_text, "%02d %%", averageLightPercent);
	printValue(Temp_Buffer_text,3,offset,size,WHITE);
	//Update Temperature
	sprintf(Temp_Buffer_text, "%0.1f C", averageTemp);
	printValue(Temp_Buffer_text,4,offset,size,WHITE);
	//Update Humidity
	sprintf(Temp_Buffer_text, "%0.1f %%", averageHumid);
	printValue(Temp_Buffer_text,5,offset,size,WHITE);
}
void updatePreviousValue(){
	uint8_t size = 3;
	uint8_t offset = 30;
	uint64_t showNum = numberOfRecord + previousNum;
	//Update Record
	sprintf(Temp_Buffer_text, "%05d", numberOfRecordBuffer[showNum % 20]);  // fix here
	printValue(Temp_Buffer_text,1.5,offset,size,WHITE);
	//Update Light
	sprintf(Temp_Buffer_text, "%02d %%", lightPercentBuffer[showNum % 20]);
	printValue(Temp_Buffer_text,3,offset,size,WHITE);
	//Update Temperature
	sprintf(Temp_Buffer_text, "%0.1f C", tempBuffer[showNum % 20]);
	printValue(Temp_Buffer_text,4,offset,size,WHITE);
	//Update Humidity
	sprintf(Temp_Buffer_text, "%0.1f %%", humidBuffer[showNum % 20]);
	printValue(Temp_Buffer_text,5,offset,size,WHITE);
}

void initialValue(){
	uint8_t size = 3;
	uint8_t offset = 30;

	prevNumberOfRecord = 0;
	prevLightPercent = 0;
	prevTemp = 0.0;
	prevHumid = 0.0;

	//Record
	sprintf(Temp_Buffer_text, "Record");
	printText(Temp_Buffer_text,1.5,offset,size,GREENYELLOW);
	//Light
	sprintf(Temp_Buffer_text, "Light");
	printText(Temp_Buffer_text,3,offset,size,ORANGE);
	//Temperature
	sprintf(Temp_Buffer_text, "Temp");
	printText(Temp_Buffer_text,4,offset,size,PINK);
	//Humidity
	sprintf(Temp_Buffer_text, "Humid");
	printText(Temp_Buffer_text,5,offset,size,MAROON);
}

void updateValue(){
	uint8_t size = 3;
	uint8_t offset = 30;

	//Read before Show // fix need to show from buffer or assign buffer to lastest value
	readAllData();

	//Update Record
	if(prevNumberOfRecord != numberOfRecord){
		sprintf(Temp_Buffer_text, "%05d", numberOfRecord+1);
		printValue(Temp_Buffer_text,1.5,offset,size,WHITE);
		prevNumberOfRecord = numberOfRecord;
	}
	//Update Light
	if(prevLightPercent != lightPercent){
		sprintf(Temp_Buffer_text, "%02d %%", lightPercent);
		printValue(Temp_Buffer_text,3,offset,size,WHITE);
		prevLightPercent = lightPercent;
	}
	//Update Temperature
	if(prevTemp != temp){
		sprintf(Temp_Buffer_text, "%0.1f C", temp);
		printValue(Temp_Buffer_text,4,offset,size,WHITE);
		prevTemp = temp;
	}
	//Update Humidity
	if(prevHumid != humid){
		sprintf(Temp_Buffer_text, "%0.1f %%", humid);
		printValue(Temp_Buffer_text,5,offset,size,WHITE);
		prevHumid = humid;
	}

	//Buffer
	numberOfRecordBuffer[numberOfRecord % 20] = numberOfRecord+1;
	lightPercentBuffer[numberOfRecord % 20] = lightPercent;
	tempBuffer[numberOfRecord % 20] = temp;
	humidBuffer[numberOfRecord % 20] = humid;

	//Write After Update
	saveAllData();
}

void resisterMonitor(){

	  float dutyCycleScreen = 0.0;
	  while(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK){}
	  adc_val = HAL_ADC_GetValue(&hadc1);
	  lightPercent = adc_val*100 / 4095;

	  //Change Screen Light Output
	  //PWM
	  dutyCycleScreen = ((adc_val/4095.0) * 0.8) + 0.2;
	  //No. 2
	  htim3.Instance -> CCR1 = (1000-1) * dutyCycleScreen;

	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);


//	  sprintf(str, "%d %d\n\r", lightPercent, adc_val);
//	  HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);
}

void readButton(){
	if(pressButton1 == 1){

		if(mode == 0){
			mode = 1;
		}else{
			mode = 0;
		}
		initialState = false;
		previousNum = 1; //avg page
		prevPreviousNum = 0;
		HAL_Delay(200); // Debounce button
		pressButton1 = 0;
	}

	if(pressButton2 == 1){
		if(previousNum > -19 && numberOfRecord+previousNum > 0){
			previousNum--;
		}
		HAL_Delay(200); // Debounce button
		pressButton2 = 0;
	}

	if(pressButton3 == 1){
		if(previousNum < 0+1){
			previousNum++;
		}
		HAL_Delay(200); // Debounce button
		pressButton3 = 0;
	}

	if(pressButton4 == 1){
		eraseAllData();
		HAL_Delay(200); // Debounce button
		pressButton4 = 0;
	}

}

void calculationTimer(){
	millisecondHAL = HAL_GetTick();
}

void colorCalculation(){
//	float temp = 22;
//	temp += lightPercent*0.1;
	if(temp >= 31.0){
		colorScreen = ORANGE;
	}else if(temp >= 29.0 && temp < 31.0){
		colorScreen = YELLOW;
	}else if(temp >= 27.0 && temp < 29.0){
		colorScreen = GREENYELLOW;
	}else if(temp >= 25.0 && temp < 27.0){
		colorScreen = GREEN;
	}else if(temp >= 23.0 && temp < 25.0){
		colorScreen = CYAN;
	}else if(temp < 23.0){
		colorScreen = BLUE;
	}
}

void assignmentOne(){

	readButton();
	calculationTimer();
	colorCalculation();

	if(mode == 0){ //Normal Mode
		if(prevColorScreen != colorScreen){
			initialState = false;
			prevColorScreen = colorScreen;
		}
		//Print Text Only First time
		if(initialState == false){
			setHorizontalScreen(colorScreen);
			initialValue();
			initialState = true;
		}

		//Read Sensor
		tempMonitor();
		//Read Variable Resister
		resisterMonitor();

		if(millisecondHAL - prevMillisecondHAL >= 500){

			//Increment
			numberOfRecord++;

			//Reset
			if(numberOfRecord > 99999){
				eraseAllData(); // for not max overide number of Record // clean and save new data
				numberOfRecord = 0;
			}

			//Print Value of Sensors
			updateValue();

			prevMillisecondHAL = millisecondHAL;
		}


	}else if(mode == 1){ // Show Average
		if(initialState == false){
			setHorizontalScreen(RED);
			initialValue();
			initialState = true;
		}

		if(prevPreviousNum != previousNum){
			if(previousNum > 0){
				setHorizontalScreen(RED); //set new screen
				averageScreen();
			}else if(previousNum == 0){
				setHorizontalScreen(RED); //set new screen
				initialValue();
				updatePreviousValue();
			}else if(previousNum < 0){
				updatePreviousValue();
			}
			prevPreviousNum = previousNum;
		}

	}

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //Delete All Data EEPROM
  readAllData();

  //Initial driver setup to drive ili9341
  ILI9341_Init();

  //ADC Input variable Resister(Light)
  HAL_ADC_Start(&hadc1);

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Green
	if (GPIO_Pin == GPIO_PIN_7)
	{
		sprintf(str, "pin7 \n\r");
		pressButton1 = 1;
		HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);
	}
	//Red
	else if (GPIO_Pin == GPIO_PIN_6)
	{
		pressButton2 = 1;
		sprintf(str, "pin6 \n\r");
		HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);
	}
	//Blue
	else if (GPIO_Pin == GPIO_PIN_5)
	{
		pressButton3 = 1;
		sprintf(str, "pin5 \n\r");
		HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);
	}
	//On Board
	//Blue
	else if (GPIO_Pin == GPIO_PIN_13)
	{
		pressButton4 = 1;
		sprintf(str, "pin13 \n\r");
		HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),200);
	}

}

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
