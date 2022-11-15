/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "BMP280.h"
#include "GPS.h"
//#include "bmp180.h"
#include "BMP280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//float Roll = 45, Pitch = 30, Yaw = 127; //BMX160 and BMP388 variables
double temperature, press, alti;
 int  i  = 0;
float temperature2, press2, alti2; //MS5611 variables
float altitudeDif = 0;

uint8_t address[16];
//gps controller
uint8_t RAW_GPS[512];

//Xbee değişkenleri
char Xbee_Buffer[256];

uint8_t xbee_rx_buffer[256];
uint8_t rxBuffer;
uint8_t rxBufferCounter = 0;
uint8_t dataArrived = 0;



// Görev yükü data
uint8_t gHour, gSec, gMin;
uint16_t gPacketCount = 0;
double gPressure, gTemperature, gAltitude;
float gVelocity;
float gVoltage;
double gGPSLongitude, gGPSAltitude, gGPSLatitude;
char gCURRENT_STATE_STRING[20];
double gRoll, gPitch, gYaw;
int gRotationCount;
char gVIDEO_STATE[16];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
long lastUpdate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart);
void sw_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//extern DS3231 rtc;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

 /*int k = 0;
   for( i = 0; i<255; i++)
   {
 	  if(!HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 100))
 	  {
 		  address[k] = i;
 		  k++;
 		  i++;
 	  }
   }*/
  //sprintf((char *)RAW_GPS,"$GPGGA,124843.00,3952.22921,N,03244.06415,E,1,08,1.15,1055.8,M,36.0,M,,*65");
  sw_init();    //software initilazing
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//BMP388 ve BMX160 kodları
	//MPU9250_getEulerAngles(&Roll, &Pitch, &Yaw);

	//alti = BMP388_readAltitude(press, 1013.25);
	//BMP388_readData(&temperature, &press);

	//BMP280 kodları
	//press2 = BMP280_ReadPressure();
	//temperature2 = BMP280_ReadTemperature();
	//alti2= BMP280_ReadAltitude(101325);


	 //GPS verileri


	//RTC verileri
	//DS3231_getTime();

	//veri aktarımı ve veriler
	//sprintf(Xbee_Buffer, "395669,%.1f,%.1f,%.1f, %.1f\r\n", press2,hgps.GPGGA.LatitudeDecimal,hgps.GPGGA.LongitudeDecimal,hgps.GPGGA.MSL_Altitude);
	if(HAL_GetTick() - lastUpdate >= 950) // saniyede bir yeni veri gönderir
	{
		GPS_Process((char)*RAW_GPS);
		//press = BMP180_GetPress(1);
		BMP280Temp_Pres_Calc(&temperature, &press);
		sprintf(Xbee_Buffer, "395669,%.1f,%f,%f,%f\r\n", press, hgps
				.GPGGA.LatitudeDecimal,hgps.GPGGA.LongitudeDecimal,hgps.GPGGA.MSL_Altitude);
		HAL_UART_Transmit(&huart2, (uint8_t*)Xbee_Buffer, strlen(Xbee_Buffer),100);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		lastUpdate = HAL_GetTick();
	}


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void sw_init(void){


//HAL_UART_RxCpltCallback(&huart1);
	// GPS Start
	HAL_UART_Receive_DMA(&huart1, RAW_GPS, 512);

	// XBee receive start
	HAL_UART_Receive_IT(&huart2, &rxBuffer, 1);


	//BMP180_Start();
	BMP280_Init(BMP280_MODE_NORMAL, BMP280_SAMPLING_X1, BMP280_SAMPLING_X1, BMP280_FILTER_OFF, BMP280_STANDBY_MS_1);



}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == huart1.Instance)
	{
		HAL_UART_Receive_DMA(&huart1, RAW_GPS, 512);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
