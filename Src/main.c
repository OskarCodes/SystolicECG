/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

uint8_t spiData;
uint8_t spiReg;

uint8_t error_lod;
uint8_t error_status;
uint8_t error_range1;
uint8_t error_misc;
uint8_t revid;
uint8_t config;

uint8_t drdypin;


uint8_t upper_ch1;
uint8_t middle_ch1;
uint8_t lower_ch1;
unsigned int ch1_data;
uint8_t ch1_data2[3];
float ch1_voltage;

uint8_t tick = 0;

uint8_t upper_ch2;
uint8_t middle_ch2;
uint8_t lower_ch2;
unsigned int ch2_data;
uint8_t ch2_data2[3];
float ch2_voltage;

uint8_t upper_ch3;
uint8_t middle_ch3;
uint8_t lower_ch3;
unsigned int ch3_data;
uint8_t ch3_data2[3];
float ch3_voltage;


//uint8_t ch1_pace[2];
//float ch1_pace_voltage;
//float ch1_pace_old = 0;


char data[100];
uint8_t buffer3[1024];

float LSB;

HAL_StatusTypeDef result;

uint8_t testing;
uint8_t runstat = 0;

char usbregister[5];
char usbdata[5];
uint8_t usbreg_int;
uint8_t usbdata_int;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void set_RGB(uint8_t red, uint8_t green, uint8_t blue) {
	htim1.Instance->CCR1 = red;
	htim1.Instance->CCR2 = green;
	htim1.Instance->CCR3 = blue;
}

void start_RGB() {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	set_RGB(0,0,0);
}

//FUNCTION TO READ ADS1293 REGISTERS WITH READ BIT
uint8_t read_reg(uint8_t reg) {
	//DEFINE REGISTER DATA VARIABLE
	uint8_t reg_data;
	//CS LOW - START SPI OP
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	//TRANSMIT REGISTER WITH READ BIT
	reg = 0x80 | reg; 
	HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
	//READ REGISTER DAT
	HAL_SPI_Receive(&hspi1, &reg_data, 1, 10);
	//CS HIGH - DONE SPI OP
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	//RETURN REGISTER DATA =)
	return reg_data;
}

//FUNCTION TO WRITE TO ADS1293 REGISTERS
HAL_StatusTypeDef write_reg(uint8_t reg, uint8_t regdata) {
	uint8_t payload[2];
	HAL_StatusTypeDef writeresult;
	payload[0] = reg;
	payload[1] = regdata;
	//CS LOW - START SPI OP
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	//TRANSMIT REGISTER WITH DATA
	writeresult = HAL_SPI_Transmit(&hspi1, payload, 2, 10);
	//CS HIGH - DONE SPI OP
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	//RETURN RESULT, e.g. HAL_OK or HAL_NOTOK
	return writeresult;
}

int fputc(int c, FILE *stream) {
	return(ITM_SendChar(c));
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
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	//CS DEFAULT HIGH
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	
	start_RGB();
	
	//STOP CONVERSION -> 00000000
	spiData = 0x00;
	spiReg = 0x00;
	result = write_reg(spiReg, spiData);
	HAL_Delay(250);
	

	//SET CH1 PROPERTIES -> 00001010 -> NO TEST, IN1 to POS, and IN2 to NEG
	spiData = 0x0A; //LEAD I
	spiReg = 0x01;
	result = write_reg(spiReg, spiData);
	
	//SET CH2 PROPERTIES -> 00011010 -> NO TEST, IN3 to POS, and IN2 to NEG
	spiData = 0x1A; //LEAD II
	spiReg = 0x02;
	result = write_reg(spiReg, spiData);
	
	//SET CH3 PROPERTIES -> 00011001 -> NO TEST, IN3 to POS, and IN1 to NEG
	spiData = 0x19; //LEAD III
	spiReg = 0x03;
	result = write_reg(spiReg, spiData);
	
	//SETUP CLK
	spiData = 0x04;
	spiReg = 0x12;
	result = write_reg(spiReg, spiData);

	//Setup R2 DEC RATE
	//spiData = 0x01; //4
	spiData = 0x08; //8
	spiReg = 0x21;
	result = write_reg(spiReg, spiData);
	
	//Setup R3-1, R3-2 & R3-3
	//spiData = 0x08; //12
	spiData = 0x10; //16
	//spiData = 0x20; //32
	//spiData = 0x40; //64
	//spiData = 0x80; //128
	
	spiReg = 0x22;
	result = write_reg(spiReg, spiData);
	spiReg = 0x23;
	result = write_reg(spiReg, spiData);
	spiReg = 0x24;
	result = write_reg(spiReg, spiData);

	//DRDY TO CHANNEL 1 ECG
	spiData = 0x08;
	spiReg = 0x27;
	result = write_reg(spiReg, spiData);
	
	//RLDOUT SETUP & TO IN4
	//spiData = 0x04; //LOW BAND
	spiData = 0x44; //HIGH BAND
	spiReg = 0x0C;
	result = write_reg(spiReg, spiData);
	
	//COMMONMODE USING IN1, IN2 & IN3
	spiData = 0x07;
	spiReg = 0x0A;
	result = write_reg(spiReg, spiData);
	
	//COMMONMODE SETUP
	spiData = 0x04; //HIGH BAND LOW CAP DRV
	//spiData = 0x05; //HIGH BAND MED CAP DRV
	spiReg = 0x0B;
	result = write_reg(spiReg, spiData);
	
	//LOOP BACK SETUP 
	//spiData = 0x10; //CH1 ONLY
	spiData = 0x30; //CH1 & CH2 ONLY
	spiReg = 0x2F;
	result = write_reg(spiReg, spiData);
	
	//SHUT DOWN CHANNELS
	//spiData = 0x36; //CH3 AND CH2 OFF 
	//spiData = 0x24; //CH3 OFF
	spiData = 0x00; //ALL ON
	spiReg = 0x14;
	result = write_reg(spiReg, spiData);
	
	//FRONT END RESOLUTION
	//spiData = 0x03; //HIGH RES CH1 & CH2
	//spiData = 0x07; //HIGH RES ALL 102.4 KHz
	spiData = 0x3F; //HIGH RES ALL 204.8 KHz
	spiReg = 0x13;
	result = write_reg(spiReg, spiData);
	
	//START CONVERSION -> 00000001
	spiData = 0x01;
	spiReg = 0x00;
	result = write_reg(spiReg, spiData);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		//READ DRDY STATUS
		drdypin = HAL_GPIO_ReadPin(DRDYB_GPIO_Port, DRDYB_Pin);
		if (drdypin == 0 && runstat == 1) {
		//if (runstat == 1) {
			set_RGB(0,0,20);
			
			//READ CH1 ECG
			upper_ch1 = read_reg(0x37);
			middle_ch1 = read_reg(0x38);
			lower_ch1 = read_reg(0x39);
			
			ch1_data = upper_ch1 << 16 | middle_ch1 << 8 | lower_ch1;
			ch1_voltage = ch1_data; //(((ch1_data / 0x800000) - (1.00/2.00)) * 4.8) / 3.5;
		
			//READ CH2 ECG
			upper_ch2 = read_reg(0x3A);
			middle_ch2 = read_reg(0x3B);
			lower_ch2 = read_reg(0x3C);
			
			ch2_data = upper_ch2 << 16 | middle_ch2 << 8 | lower_ch2;
			ch2_voltage = ch2_data;
			
			//READ CH3 ECG
			upper_ch3 = read_reg(0x3D);
			middle_ch3 = read_reg(0x3E);
			lower_ch3 = read_reg(0x3F);
			
			ch3_data = upper_ch3 << 16 | middle_ch3 << 8 | lower_ch3;
			ch3_voltage = ch3_data;
			
			//mV conversion
			//ch1_voltage = ch1_voltage * pow(10.0,3.0);
			//ch2_voltage = ch2_voltage * pow(10.0,3.0);
			//ch3_voltage = ch3_voltage * pow(10.0,3.0);

			
			/*
			ch1_pace[0] = read_reg(0x31);
			ch1_pace[1] = read_reg(0x32);
			ch1_pace_voltage = ch1_pace[0] << 8 | ch1_pace[1];
			if ((ch1_pace_voltage - ch1_pace_old) > 0 && ch1_pace_voltage > )
			*/
			
			//CALCULATE LEAD III
			
			/*
			//READ CH1 PACE
			upper_ch1 = read_reg(0x31);
			lower_ch1 = read_reg(0x32);			
			ch1_data = upper_ch1 << 8 | lower_ch1;
			ch1_voltage = ch1_data;
			*/
			/*
			//READ ERROR REGISTERS
			error_lod = read_reg(0x18);
			error_status = read_reg(0x19);
			error_range1 = read_reg(00x1A);
			error_misc = read_reg(0x1E);
			revid = read_reg(0x40);
			config = read_reg(0x00);
			*/
			
			// sprintf(data, "%f\n", ch2_voltage);
			sprintf(data, "%f,%f,%f\n", ch1_voltage, ch2_voltage, ch3_voltage);
			strcpy (buffer3, data);
			CDC_Transmit_FS(buffer3, strlen(buffer3));
			//printf("Data read!");
			set_RGB(0,20,0);
		}
		//HAL_Delay(1000);
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1230-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ALARMB_Pin */
  GPIO_InitStruct.Pin = ALARMB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ALARMB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDYB_Pin */
  GPIO_InitStruct.Pin = DRDYB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDYB_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void CDC_RecieveCallBack(uint8_t *buf, uint32_t len) {
	
	//STOP CONVERSION -> 00000000

	spiData = 0x00;
	spiReg = 0x00;
	result = write_reg(spiReg, spiData);
	printf("Stopped Conversion!\n");
	runstat = 0;
	
	testing = read_reg(0x00);
	char usbregister[5];
	memcpy( usbregister, &buf[0], 4 );
	usbregister[4] = '\0';
	
	char usbdata[5];
	memcpy( usbdata, &buf[5], 4 );
	usbdata[4] = '\0';

	
	/*
	printf((char*)usbregister);
	printf("\n");
	printf((char*)usbdata);
	*/
	
	usbreg_int = strtoul(usbregister, NULL, 16);
	usbdata_int = strtoul(usbdata, NULL, 16);
	
	write_reg(usbreg_int, usbdata_int);
	sprintf(data, "Register: %i, Data: %i \n", usbreg_int, usbdata_int);
	
	if (strncmp(usbregister, "0x00", 2) == 0 && strncmp(usbdata, "0x01", 4) == 0) {
		runstat = 1;
		// printf((uint8_t*)usbregister);
		// printf((uint8_t*)usbdata);
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
