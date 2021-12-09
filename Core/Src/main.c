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

uint32_t millisecond = 0;
uint32_t halfsecond = 0;

uint32_t secondNum = 50;
uint32_t minuteNum = 37;
uint32_t hourNum = 12;

uint32_t prevSecondNum = -1;
uint32_t prevMinuteNum = -1;
uint32_t prevHourNum = -1;

bool halfsecondState = true;
bool initialState = false;
char Temp_Buffer_text[40];

//Horizontal Screen
uint16_t maxWidth = 200; //300 //Left 50 - right 50
uint16_t offsetWidth = 60;
uint16_t maxHeight = 240;

uint16_t mode = 0;
uint16_t modeEdit = 1;
uint16_t prevMode = -1;
uint16_t prevModeEdit = -1;

bool pressButton1 = false;
bool pressButton2 = false;
bool pressButton3 = false;
bool pressButton4 = false;

bool isPressButton1 = false;
bool isPressButton2 = false;
bool isPressButton3 = false;
bool isPressButton4 = false;

uint16_t secondCounter = 0;
uint16_t prevSecondCounter = 0;
uint32_t millisecondHAL = 0;

char day[7][3] = {"SUN","MON","TUE","WED","THU","FRI","SAT"};
char month[12][3] = {"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};

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


void calculationClock(uint32_t ms){

	millisecondHAL = HAL_GetTick();

	if(millisecond >= 1000){
		millisecond = 0;
		secondNum++;

	}
	if(secondNum >= 60){
		secondNum = 0;
		if(mode != 100){
			minuteNum++;
		}
	}
	if(minuteNum >= 60){
		minuteNum = 0;
		if(mode != 100){
			hourNum++;
		}
	}
	if(hourNum >= 24){
		hourNum = 0;
	}

	//check for editMode
	if(minuteNum < 0){
		minuteNum = 59;
	}
	if(hourNum < 0){
		hourNum = 23;
	}


}


//void displayClock(uint32_t ms){
//
//	char hexString[30];
//	if(halfsecondState == false){ // colon behaviour
//		sprintf(hexString,"%02d:%02d\r", hourNum, minuteNum);
//		HAL_UART_Transmit(&huart3, (uint8_t*) hexString, strlen(hexString), 1000);
//	}else{
//		sprintf(hexString,"%02d %02d\r", hourNum, minuteNum);
//		HAL_UART_Transmit(&huart3, (uint8_t*) hexString, strlen(hexString), 1000);
//	}
//}

void topBarScreen(){


	sprintf(Temp_Buffer_text, "ON");
	ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0.9 + offsetWidth-5, maxHeight*0.1, BLACK, 2, RED);


}

void resetPrevNum(){
	prevSecondNum = -1;
	prevMinuteNum = -1;
	prevHourNum = -1;
}

void hourScreen(bool status, bool isEdit){
	if(prevHourNum != hourNum || isEdit == true){
		if(status == true){
			sprintf(Temp_Buffer_text, "%02d", hourNum);
			ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0 + offsetWidth-5, maxHeight*0.3, WHITE, 6, BLACK);
		}else{
			sprintf(Temp_Buffer_text, "  ");
			ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0 + offsetWidth-5, maxHeight*0.3, WHITE, 6, BLACK);
		}
		prevHourNum = hourNum;
	}
}

void colonScreen(bool status){
	if(status == true){
		sprintf(Temp_Buffer_text, ":");
		ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0 + offsetWidth+75, maxHeight*0.35, WHITE, 4, BLACK);
	}else{
		sprintf(Temp_Buffer_text, " ");
		ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0 + offsetWidth+75, maxHeight*0.35, WHITE, 4, BLACK);
	}

}
void minuteScreen(bool status, bool isEdit){
	if(prevMinuteNum != minuteNum || isEdit == true){
		if(status == true){

			sprintf(Temp_Buffer_text, "%02d", minuteNum);
			ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0 + offsetWidth+100, maxHeight*0.3, WHITE, 6, BLACK);
		}else{
			sprintf(Temp_Buffer_text, "  ", minuteNum);
			ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0 + offsetWidth+100, maxHeight*0.3, WHITE, 6, BLACK);
		}
		prevMinuteNum = minuteNum;
	}

}
void secondScreen(bool status, bool isEdit){
	if(prevSecondNum != secondNum || isEdit == true){
		if(status == true){
			sprintf(Temp_Buffer_text, "%02d", secondNum);
			ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0.90 + offsetWidth, maxHeight*0.42, WHITE, 2, BLACK);
		}else{
			sprintf(Temp_Buffer_text, "  ");
			ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0.90 + offsetWidth, maxHeight*0.42, WHITE, 2, BLACK);
		}
		prevSecondNum = secondNum;
	}
}

void displayClockScreen(){

	if(halfsecondState == false){ // colon behaviour
		colonScreen(true);
	}else{
		colonScreen(false);
	}

	secondScreen(true,false);
	minuteScreen(true,false);
	hourScreen(true,false);
}

void editHourScreen(){

	colonScreen(true);
	minuteScreen(true,false);
	secondScreen(true,false);

	if(halfsecondState == false){ // hour
		hourScreen(false,true);
	}else{
		hourScreen(true,true);
	}

}
void editMinuteScreen(){

	colonScreen(true);
	hourScreen(true,false);
	secondScreen(true,false);

	if(halfsecondState == false){ //minute
		minuteScreen(false,true);
	}else{
		minuteScreen(true,true);
	}

}
void editSecondScreen(){

	colonScreen(true);
	hourScreen(true,false);
	minuteScreen(true,false);

	if(halfsecondState == false){  //second
		secondScreen(false,true);
	}else{
		secondScreen(true,true);
	}

}

void bottomBarScreen(){

	uint8_t size = 2;
	uint16_t maxWidthFull = 320;
	uint8_t bottomHeight = maxHeight*0.87;
	uint8_t bottomWidth1 = maxWidth*0+51;
	uint8_t bottomWidth2 = maxWidth*0.25+51;
	uint8_t bottomWidth3 = maxWidth*0.50+51;
	uint8_t bottomWidth4 = maxWidth*0.75+51;
	uint8_t bottomWidth = 55;

	ILI9341_Draw_Filled_Rectangle_Coord(bottomWidth1, bottomHeight, bottomWidth1+bottomWidth, maxHeight, RED);
	ILI9341_Draw_Filled_Rectangle_Coord(bottomWidth2, bottomHeight, bottomWidth2+bottomWidth, maxHeight, YELLOW);
	ILI9341_Draw_Filled_Rectangle_Coord(bottomWidth3, bottomHeight, bottomWidth3+bottomWidth, maxHeight, CYAN);
	ILI9341_Draw_Filled_Rectangle_Coord(bottomWidth4, bottomHeight, bottomWidth4+bottomWidth, maxHeight, GREEN);

	sprintf(Temp_Buffer_text, "MOD");
	ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0 + offsetWidth, maxHeight*0.9, BLACK, size, RED);

	sprintf(Temp_Buffer_text, "ADJ");
	ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0.25 + offsetWidth, maxHeight*0.9, BLACK, size, YELLOW);

	sprintf(Temp_Buffer_text, "FWD");
	ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0.5 + offsetWidth, maxHeight*0.9, BLACK, size, CYAN);

	sprintf(Temp_Buffer_text, "BWD");
	ILI9341_Draw_Text(Temp_Buffer_text, maxWidth*0.75 + offsetWidth, maxHeight*0.9, BLACK, size, GREEN);

}

void buzzerSound(){
	  htim3.Instance -> CCR1 = (1000-1) * 0.5;
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_Delay(70);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}




char str[50];
uint8_t cmdBuffer[3];
uint8_t dataBuffer[8];


void assignmentOne(){

	setHorizontalScreen(BLACK);






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
	//HAL_Delay(1000);	// refresh every 1 second
}

void assignmentTwo(){

	calculationClock(millisecond);

	if(prevMode != mode || prevModeEdit != modeEdit){
		prevModeEdit = modeEdit;
		resetPrevNum();
	}
	if(prevMode != mode){
		prevMode = mode;
		setHorizontalScreen(BLACK);
		initialState = false;
		bottomBarScreen();
	}


	if(mode == 0){
		topBarScreen();
		displayClockScreen();
	}else if(mode == 100){ // Adjust time

		if(modeEdit == 1){
			editHourScreen();
		}else if(modeEdit == 2){
			editMinuteScreen();
		}else if(modeEdit == 3){
			editSecondScreen();
		}
	}else if(mode == 1){

	}


	//Test huart1 UART PB6 TX - PB15 RX
//	sprintf(Temp_Buffer_text, "AA");
//	HAL_UART_Transmit(&huart1, (uint8_t*) Temp_Buffer_text, strlen(Temp_Buffer_text), 1000);


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

  cmdBuffer[0] = 0x03;
  cmdBuffer[1] = 0x00;
  cmdBuffer[2] = 0x04;

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



	  // REAL CODE BEGIN


	  //TEST WITH TEMP SENSOR
	  //I2C1_ADDR_Chk();		//LOOP CHECK ADDR
	  //getAM2320(env);			//P'AU CODE
	  //tempMonitor();			//AJARN CODE


//	  char stringBuffer[30];
//	  sprintf(stringBuffer, "%d\r\n" , millisecond);
//	  HAL_UART_Transmit(&huart3, (uint8_t*) stringBuffer, strlen(stringBuffer), 200);


	  if(halfsecond == 1){  // interupt every 500 ms
		  halfsecondState = !halfsecondState; // check appearing of colon (:) in clock
		  //displayClock(millisecond);
		  halfsecond = 0;
		  //assignmentTwo();
	  }

	  assignmentOne();

	  pressButton1 = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7); // pressButton1 is "true" when press, is "false" when not press
	  pressButton2 = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6); // pressButton1 is "true" when press, is "false" when not press
	  pressButton3 = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5); // pressButton1 is "true" when press, is "false" when not press
	  pressButton4 = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4); // pressButton1 is "true" when press, is "false" when not press

	  //Buzzer
	  if((pressButton1 == true && isPressButton1 == false) ||
		  (pressButton2 == true && isPressButton2 == false) ||
		  (pressButton3 == true && isPressButton3 == false) ||
		  (pressButton4 == true && isPressButton4 == false)){
		  buzzerSound();
	  }


	  //General Mode
	  if(pressButton1 == true && isPressButton1 == false && mode == 0){ // increase mode only once
		  mode++;
		  isPressButton1 = true;
	  }


	  //Adjust Time Mode
  	  if(pressButton2 == true && isPressButton2 == false && mode == 0){ // initial time when pressButton2
  		  isPressButton2 = true;
  		  prevSecondCounter = millisecondHAL;
  	  }else if(pressButton2 == true && isPressButton2 == true && mode == 0 && millisecondHAL - prevSecondCounter >= 3000){ // hold for 3 seconds
		  buzzerSound();
  		  mode = 100;
		  prevSecondCounter = millisecondHAL;
	  }

  	  //Exit Adjust Time Mode
  	  if(pressButton2 == true && isPressButton2 == false && millisecondHAL - prevSecondCounter >= 1000 && mode == 100){
  		  isPressButton2 = true;
  		  mode = 0;
  		  prevSecondCounter = millisecondHAL;
  	  }

  	  //Edit Mode
	  if(pressButton1 == true && isPressButton1 == false && mode == 100){ // increase mode only once
		  modeEdit++;
		  isPressButton1 = true;
		  if(modeEdit == 4){ // finish loop edit
			  modeEdit = 1; // Reset to hour
			  mode = 0;	// Back to General Mode
		  }
	  }

	  //Forward
	  if(pressButton3 == true && isPressButton3 == false && mode == 100){ // increase value
		  if(modeEdit == 1){
			  hourNum--;
		  }else if(modeEdit == 2){
			  minuteNum--;
		  }else if(modeEdit == 3){
			  secondNum = 0;
		  }
		  halfsecondState = false;
		  resetPrevNum();
		  isPressButton3 = true;
	  }

	  //Backward
	  if(pressButton4 == true && isPressButton4 == false && mode == 100){ // decrease value
		  if(modeEdit == 1){
			  hourNum++;
		  }else if(modeEdit == 2){
			  minuteNum++;
		  }else if(modeEdit == 3){
			  secondNum = 0;
		  }
		  halfsecondState = false;
		  resetPrevNum();
		  isPressButton4 = true;
	  }


  	  //Reset isPressButton
	  if(pressButton1 == false){
		  isPressButton1 = false;
	  }
  	  if(pressButton2 == false){
		  isPressButton2 = false;
	  }
	  if(pressButton3 == false){
		  isPressButton3 = false;
	  }
	  if(pressButton4 == false){
		  isPressButton4 = false;
	  }

	  // REAL CODE END

	  //----------------------------------------------------------PERFORMANCE TEST
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("FPS TEST, 40 loop 2 screens", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		uint32_t Timer_Counter = 0;
//	  		for(uint32_t j = 0; j < 2; j++)
//	  		{
//	  			HAL_TIM_Base_Start(&htim1);
//	  			for(uint16_t i = 0; i < 10; i++)
//	  			{
//	  				ILI9341_Fill_Screen(WHITE);
//	  				ILI9341_Fill_Screen(BLACK);
//	  			}
//
//	  			//20.000 per second!
//	  			HAL_TIM_Base_Stop(&htim1);
//	  			Timer_Counter += __HAL_TIM_GET_COUNTER(&htim1);
//	  			__HAL_TIM_SET_COUNTER(&htim1, 0);
//	  		}
//	  		Timer_Counter /= 2;
//
//	  		char counter_buff[30];
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		sprintf(counter_buff, "Timer counter value: %ld", Timer_Counter*2);
//	  		ILI9341_Draw_Text(counter_buff, 10, 10, BLACK, 1, WHITE);
//
//	  		double seconds_passed = 2*((float)Timer_Counter / 20000);
//	  		sprintf(counter_buff, "Time: %.3f Sec", seconds_passed);
//	  		ILI9341_Draw_Text(counter_buff, 10, 30, BLACK, 2, WHITE);
//
//	  		double timer_float = 20/(((float)Timer_Counter)/20000);	//Frames per sec
//
//	  		sprintf(counter_buff, "FPS:  %.2f", timer_float);
//	  		ILI9341_Draw_Text(counter_buff, 10, 50, BLACK, 2, WHITE);
//	  		double MB_PS = timer_float*240*320*2/1000000;
//	  		sprintf(counter_buff, "MB/S: %.2f", MB_PS);
//	  		ILI9341_Draw_Text(counter_buff, 10, 70, BLACK, 2, WHITE);
//	  		double SPI_utilized_percentage = (MB_PS/(6.25 ))*100;		//50mbits / 8 bits
//	  		sprintf(counter_buff, "SPI Utilized: %.2f", SPI_utilized_percentage);
//	  		ILI9341_Draw_Text(counter_buff, 10, 90, BLACK, 2, WHITE);
//	  		HAL_Delay(10000);
//
//
//	  		static uint16_t x = 0;
//	  		static uint16_t y = 0;
//
//	  		char Temp_Buffer_text[40];

	  //----------------------------------------------------------COUNTING MULTIPLE SEGMENTS
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Counting multiple segments at once", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		for(uint16_t i = 0; i <= 10; i++)
//	  		{
//	  		sprintf(Temp_Buffer_text, "Counting: %d", i);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 30, BLUE, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 50, RED, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 70, GREEN, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 90, BLACK, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 110, BLUE, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 130, RED, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 150, GREEN, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 170, WHITE, 2, BLACK);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 190, BLUE, 2, BLACK);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 210, RED, 2, BLACK);
//	  		}
//
//	  		HAL_Delay(1000);

	  //----------------------------------------------------------COUNTING SINGLE SEGMENT
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Counting single segment", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint16_t i = 0; i <= 100; i++)
//	  		{
//	  		sprintf(Temp_Buffer_text, "Counting: %d", i);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 3, WHITE);
//	  		}
//
//	  		HAL_Delay(1000);

	  //----------------------------------------------------------ALIGNMENT TEST
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Rectangle alignment check", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		ILI9341_Draw_Hollow_Rectangle_Coord(50, 50, 100, 100, BLACK);
//	  		ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 50, 50, BLACK);
//	  		ILI9341_Draw_Hollow_Rectangle_Coord(10, 10, 19, 19, BLACK);
//	  		HAL_Delay(1000);

	  //----------------------------------------------------------LINES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Horizontal and Vertical lines", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 30000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Horizontal_Line(xr, yr, radiusr, colourr);
//	  			ILI9341_Draw_Vertical_Line(xr, yr, radiusr, colourr);
//	  		}
//
//	  		HAL_Delay(1000);

	  //----------------------------------------------------------HOLLOW CIRCLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Circles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		for(uint32_t i = 0; i < 3000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Hollow_Circle(xr, yr, radiusr*2, colourr);
//	  		}
//	  		HAL_Delay(1000);

	  //----------------------------------------------------------FILLED CIRCLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Filled Circles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 1000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Filled_Circle(xr, yr, radiusr/2, colourr);
//	  		}
//	  		HAL_Delay(1000);

	  //----------------------------------------------------------HOLLOW RECTANGLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Rectangles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 20000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Hollow_Rectangle_Coord(xr, yr, xr+radiusr, yr+radiusr, colourr);
//	  		}
//	  		HAL_Delay(1000);

	  //----------------------------------------------------------FILLED RECTANGLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Filled Rectangles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 20000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Rectangle(xr, yr, radiusr, radiusr, colourr);
//	  		}
//	  		HAL_Delay(1000);

	  //----------------------------------------------------------INDIVIDUAL PIXEL EXAMPLE

//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Slow draw by selecting", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("and adressing pixels", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		x = 0;
//	  		y = 0;
//	  		while (y < 240)
//	  		{
//	  		while ((x < 320) && (y < 240))
//	  		{
//
//	  			if(x % 2)
//	  			{
//	  				ILI9341_Draw_Pixel(x, y, BLACK);
//	  			}
//
//	  			x++;
//	  		}
//
//	  			y++;
//	  			x = 0;
//	  		}
//
//	  		x = 0;
//	  		y = 0;
//
//
//	  		while (y < 240)
//	  		{
//				while ((x < 320) && (y < 240))
//				{
//
//					if(y % 2)
//					{
//						ILI9341_Draw_Pixel(x, y, BLACK);
//					}
//
//					x++;
//				}
//
//	  			y++;
//	  			x = 0;
//	  		}
//	  		HAL_Delay(2000);

	  //----------------------------------------------------------INDIVIDUAL PIXEL EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Random position and colour", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("500000 pixels", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		for(uint32_t i = 0; i < 500000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			uint16_t color = HAL_RNG_GetRandomNumber(&hrng);
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			ILI9341_Draw_Pixel(xr, yr, color);
//	  		}
//	  		HAL_Delay(2000);

	  //----------------------------------------------------------565 COLOUR EXAMPLE, Grayscale
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Colour gradient", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Grayscale", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//
//
//	  		for(uint16_t i = 0; i <= (320); i++)
//	  		{
//	  			uint16_t Red = 0;
//	  			uint16_t Green = 0;
//	  			uint16_t Blue = 0;
//
//	  			Red = i/(10);
//	  			Red <<= 11;
//	  			Green = i/(5);
//	  			Green <<= 5;
//	  			Blue = i/(10);
//
//
//
//	  			uint16_t RGB_color = Red + Green + Blue;
//	  			ILI9341_Draw_Rectangle(i, x, 1, 240, RGB_color);
//
//	  		}
//	  		HAL_Delay(2000);

	  //----------------------------------------------------------IMAGE EXAMPLE, Snow Tiger
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("RGB Picture", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("TIGER", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Draw_Image((const char*)snow_tiger, SCREEN_VERTICAL_2);
//	  		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
//	  		HAL_Delay(5000);


	  //----------------------------------------------------------TOUCHSCREEN EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Touchscreen", 10, 10, BLACK, 2, WHITE);
//	  		ILI9341_Draw_Text("Touch to draw", 10, 30, BLACK, 2, WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
//
//
//	  		while(1)
//	  		{
//	  			HAL_Delay(20);
//
//	  			if(TP_Touchpad_Pressed())
//	          {
//
//	  					uint16_t x_pos = 0;
//	  					uint16_t y_pos = 0;
//
//
//	  					HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_SET);
//
//	            uint16_t position_array[2];
//
//	  					if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK)
//	  					{
//	  					x_pos = position_array[0];
//	  					y_pos = position_array[1];
//	  					ILI9341_Draw_Filled_Circle(x_pos, y_pos, 2, BLACK);
//
//	  					ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  					char counter_buff[30];
//	  					sprintf(counter_buff, "POS X: %.3d", x_pos);
//	  					ILI9341_Draw_Text(counter_buff, 10, 80, BLACK, 2, WHITE);
//	  					sprintf(counter_buff, "POS Y: %.3d", y_pos);
//	  					ILI9341_Draw_Text(counter_buff, 10, 120, BLACK, 2, WHITE);
//	  					ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
//	  					}
//
//	  					ILI9341_Draw_Pixel(x_pos, y_pos, BLACK);
//
//	          }
//	  			else
//	  			{
//	  				HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);
//	  			}
//
//	  		}

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
