
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "math.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void magConfig(I2C_HandleTypeDef hi2c, uint16_t addMAG);
static void magRead(I2C_HandleTypeDef hi2c, uint16_t addMAG, uint16_t regMAG);
static void accConfig(I2C_HandleTypeDef hi2c, uint16_t addACC);
static void accRead(I2C_HandleTypeDef hi2c, uint16_t addACC, uint16_t regACC);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
 int i2cFreq=600000; //600000,100000,400000
 int uartBaudRate = 921600; //115200;921600;
 double del1,del2,del3,del4;
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_RTC_Init();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  HAL_Delay(30);
	/* USER CODE BEGIN 2 */
	uint16_t regACC=0x20; // ACCELERATION register
	uint16_t regGIR=0xF7; // GIROSCOPE    register
	uint16_t regMAG=0x28; // MAGNETOMETER register
	uint16_t addACC=0x6A<<1; // ACCELERATION address
	uint16_t addGIR=0x6A<<1; // GIROSCOPE    address
	uint16_t addMAG=0x1E<<1; // MAGNETOMETER address
	uint8_t mag1[] = {0,0,0,0,0,0,0,0};
	uint8_t mag2[] = {0,0,0,0,0,0,0,0};
	uint8_t mag3[] = {0,0,0,0,0,0,0,0};

  /* USER CODE END 2 */
		/* magnetometer configuration*/
	magConfig(hi2c1, addMAG);
  magConfig(hi2c2, addMAG);
  magConfig(hi2c3, addMAG);
		/* accelerometer + giro configuration*/
	accConfig(hi2c1, addACC);
	accConfig(hi2c2, addACC);
	accConfig(hi2c3, addACC);

	/* Infinite loop */
  /* USER CODE BEGIN WHILE */
	double i=0;
 
		
  while (1)
  {
  /* I2C   */
		
  magRead(hi2c1, addMAG, regMAG);
  magRead(hi2c2, addMAG, regMAG);
  magRead(hi2c3, addMAG, regMAG);
		
	accRead(hi2c1, addACC, regACC);
	accRead(hi2c2, addACC, regACC);
	accRead(hi2c3, addACC, regACC);

	uint8_t who[] = {0};
	while(HAL_I2C_IsDeviceReady(&hi2c1, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK);
	HAL_I2C_Mem_Read(&hi2c1, addMAG, 0xF, I2C_MEMADD_SIZE_8BIT, who, sizeof(who), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,who,sizeof(who),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n",sizeof("\r\n"),HAL_MAX_DELAY);
	who[0] = 0;
	while(HAL_I2C_IsDeviceReady(&hi2c2, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK);
	HAL_I2C_Mem_Read(&hi2c2, addMAG, 0xF, I2C_MEMADD_SIZE_8BIT, who, sizeof(who), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,who,sizeof(who),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n",sizeof("\r\n"),HAL_MAX_DELAY);
	who[0] = 0;
	while(HAL_I2C_IsDeviceReady(&hi2c3, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK);
	HAL_I2C_Mem_Read(&hi2c3, addMAG, 0xF, I2C_MEMADD_SIZE_8BIT, who, sizeof(who), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,who,sizeof(who),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n",sizeof("\r\n"),HAL_MAX_DELAY);
	//HAL_Delay(93);
	
	who[0] = 0;
	while(HAL_I2C_IsDeviceReady(&hi2c1, addACC, 1, HAL_MAX_DELAY)!=HAL_OK);
	HAL_I2C_Mem_Read(&hi2c1, addACC, 0xF, I2C_MEMADD_SIZE_8BIT, who, sizeof(who), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,who,sizeof(who),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n",sizeof("\r\n"),HAL_MAX_DELAY);
	who[0] = 0;
	while(HAL_I2C_IsDeviceReady(&hi2c2, addACC, 1, HAL_MAX_DELAY)!=HAL_OK);
	HAL_I2C_Mem_Read(&hi2c2, addACC, 0xF, I2C_MEMADD_SIZE_8BIT, who, sizeof(who), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,who,sizeof(who),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n",sizeof("\r\n"),HAL_MAX_DELAY);
	who[0] = 0;
	while(HAL_I2C_IsDeviceReady(&hi2c3, addACC, 1, HAL_MAX_DELAY)!=HAL_OK);
	HAL_I2C_Mem_Read(&hi2c3, addACC, 0xF, I2C_MEMADD_SIZE_8BIT, who, sizeof(who), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,who,sizeof(who),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n",sizeof("\r\n"),HAL_MAX_DELAY);
	//HAL_Delay(9);
/**/		

		/* UART  */
	i++;
	//HAL_UART_Transmit(&huart2,(uint8_t *)(i),sizeof(i),100);
  //HAL_UART_Transmit(&huart2,(uint8_t *)("\r\n"),sizeof("\r\n"),100);
	//HAL_UART_Transmit(&huart2,(uint8_t *)("os4tias\r\n"),sizeof("os4tias\r\n"),100);
		
  /* GPIO  */
	if(i>del1)
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);//LED on
	else
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);//LED off
	if(i>del2)
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);//LED on
	else
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);//LED off
	if(i>del3)
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);//LED on
	else
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);//LED off
	if(i>del4)
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);//LED on
	else
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);//LED off

	//while(HAL_I2C_IsDeviceReady(&hi2c1, DevAddress, 1, HAL_MAX_DELAY)!=HAL_OK);
		
	if (i> 255)
		i=0;
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = i2cFreq;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = i2cFreq;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = i2cFreq;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = uartBaudRate; //921600; //115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct0;
  GPIO_InitTypeDef GPIO_InitStruct1;
  GPIO_InitTypeDef GPIO_InitStruct2;
  GPIO_InitTypeDef GPIO_InitStruct3;
	
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
	
	  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct0.Pin = GPIO_PIN_9;
  GPIO_InitStruct0.Pin = GPIO_PIN_9;
  GPIO_InitStruct0.Pin = GPIO_PIN_9;
  GPIO_InitStruct0.Pin = GPIO_PIN_9;
  GPIO_InitStruct0.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct0.Pull = GPIO_NOPULL;
  GPIO_InitStruct0.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct0);
  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct1.Pin = GPIO_PIN_10;
  GPIO_InitStruct1.Pin = GPIO_PIN_10;
  GPIO_InitStruct1.Pin = GPIO_PIN_10;
  GPIO_InitStruct1.Pin = GPIO_PIN_10;
  GPIO_InitStruct1.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct1.Pull = GPIO_NOPULL;
  GPIO_InitStruct1.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct1);
  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct2.Pin = GPIO_PIN_7;
  GPIO_InitStruct2.Pin = GPIO_PIN_7;
  GPIO_InitStruct2.Pin = GPIO_PIN_7;
  GPIO_InitStruct2.Pin = GPIO_PIN_7;
  GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct2.Pull = GPIO_NOPULL;
  GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct2);
  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct3.Pin = GPIO_PIN_8;
  GPIO_InitStruct3.Pin = GPIO_PIN_8;
  GPIO_InitStruct3.Pin = GPIO_PIN_8;
  GPIO_InitStruct3.Pin = GPIO_PIN_8;
  GPIO_InitStruct3.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct3.Pull = GPIO_NOPULL;
  GPIO_InitStruct3.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct3);

}

/* USER CODE BEGIN 4 */
void magConfig(I2C_HandleTypeDef hi2c, uint16_t addMAG)
{
	uint8_t reg = 0x20; //  1. CTR_REG1
	uint8_t write[] = {0xFE};
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
	HAL_I2C_Mem_Write(&hi2c, addMAG, reg, I2C_MEMADD_SIZE_8BIT, write, 1, HAL_MAX_DELAY);
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
	reg = 0x21;        //  2. CTR_REG2
	write[0] = 0x6C; // reset + reboot
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
	HAL_I2C_Mem_Write(&hi2c, addMAG, reg, I2C_MEMADD_SIZE_8BIT, write, 1, HAL_MAX_DELAY);
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
	HAL_Delay(100);
	reg = 0x21;        //  2. CTR_REG2
	write[0] = 0x00;
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
	HAL_I2C_Mem_Write(&hi2c, addMAG, reg, I2C_MEMADD_SIZE_8BIT, write, 1, HAL_MAX_DELAY);
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
	
	reg = 0x22;        //  3. CTR_REG3
	write[0] = 0x00;
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
	HAL_I2C_Mem_Write(&hi2c, addMAG, reg, I2C_MEMADD_SIZE_8BIT, write, 1, HAL_MAX_DELAY);
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
	reg = 0x23;        //  4. CTR_REG4
	write[0] = 0x0C;
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
	HAL_I2C_Mem_Write(&hi2c, addMAG, reg, I2C_MEMADD_SIZE_8BIT, write, 1, HAL_MAX_DELAY);
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
	reg = 0x24;        //  5. CTR_REG5
	write[0] = 0x80;
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
	HAL_I2C_Mem_Write(&hi2c, addMAG, reg, I2C_MEMADD_SIZE_8BIT, write, 1, HAL_MAX_DELAY);
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK){};
}
void magRead(I2C_HandleTypeDef hi2c, uint16_t addMAG, uint16_t regMAG)
{
	uint8_t mag1[] = {0,0,0,0,0,0,0,0};
	while(HAL_I2C_IsDeviceReady(&hi2c, addMAG, 1, HAL_MAX_DELAY)!=HAL_OK);
	HAL_I2C_Mem_Read(&hi2c, addMAG, regMAG, I2C_MEMADD_SIZE_8BIT, mag1, sizeof(mag1), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,mag1,sizeof(mag1),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n",sizeof("\r\n"),HAL_MAX_DELAY);
	del1=mag1[7];
	del2=mag1[0];
}
void accConfig(I2C_HandleTypeDef hi2c, uint16_t addACC)
{
	uint8_t reg = 0x10; //  1. CTR_REG1
	uint8_t write[] = {0xA8,0xA8,0x04,0x82};
	while(HAL_I2C_IsDeviceReady(&hi2c, addACC, 1, HAL_MAX_DELAY)!=HAL_OK){};
	HAL_I2C_Mem_Write(&hi2c, addACC, reg, I2C_MEMADD_SIZE_8BIT, write, sizeof(write), HAL_MAX_DELAY);
	while(HAL_I2C_IsDeviceReady(&hi2c, addACC, 1, HAL_MAX_DELAY)!=HAL_OK){};
}
void accRead(I2C_HandleTypeDef hi2c, uint16_t addACC, uint16_t regACC)
{
	uint8_t mag1[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	while(HAL_I2C_IsDeviceReady(&hi2c, addACC, 1, HAL_MAX_DELAY)!=HAL_OK);
	HAL_I2C_Mem_Read(&hi2c, addACC, regACC, I2C_MEMADD_SIZE_8BIT, mag1, sizeof(mag1), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,mag1,sizeof(mag1),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n",sizeof("\r\n"),HAL_MAX_DELAY);
	del3=mag1[3];
	del4=mag1[4];
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
