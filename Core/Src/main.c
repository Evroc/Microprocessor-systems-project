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
#include "eth.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_DATA_INDEX 1
#define BMP280_ADDRESS_INDEX 2
const int SPI_BUFFER_LEN= 28;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t Received[4];
double TempRef=30;
char buffer[100];
uint8_t size;
uint8_t error;
double TempDif=0;
static unsigned short heater_state = 0;
static unsigned short fan_state = 0;
double TempRecived;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/**Do opisania, odbirór z terminala plus kontrola*/

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	TempRecived=atof(&Received);
	if (TempRecived < 20.0)
	{
		error = sprintf(buffer,"Nieprawidlowa wartosc zadana\n\r ");
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, error, 200);
	}
	else if (TempRecived > 40.0)
	{
		error = sprintf(buffer,"Nieprawidlowa wartosc zadana\n\r ");
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, error, 200);
	}
	else
	{
		TempRef=(atof(&Received));
	}

}

/** Do opisania, algorytm sterowania*/
void Control_Algorithm (double TempRef, double temp)
{
	 TempDif=TempRef-temp;  //obliczanie różnicy temepratury zadanej i mierzonej

	 if (TempDif > 1)
	 {
		 heater_state = 1;
		 fan_state = 0;
	 }
	 else if (TempDif < -0.8)
	 {
			 heater_state = 0;
			 fan_state = 1;
	 }
	 else if (TempDif < -0.2)
	 {
		 heater_state = 0;
		 fan_state = 0;
	 }

}

int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the SPI write routine according to the target machine . */
	HAL_StatusTypeDef status = HAL_OK ;
	int32_t iError = BMP280_OK ;
	uint8_t txarray [ SPI_BUFFER_LEN * BMP280_ADDRESS_INDEX ];
	uint8_t stringpos ;


	txarray [0] = reg_addr ;
	for ( stringpos = 0; stringpos < length ; stringpos ++) {
	txarray [ stringpos + BMP280_DATA_INDEX ] = reg_data [ stringpos ];
	 }

	HAL_GPIO_WritePin ( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_RESET );
	status = HAL_SPI_Transmit ( &hspi4 , ( uint8_t *)(& txarray ), length *2, 100);
	while ( hspi4 . State == HAL_SPI_STATE_BUSY ) {};
	HAL_GPIO_WritePin ( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_SET );

	if ( status != HAL_OK )
	{
	iError = ( -1);
	}
	return ( int8_t ) iError ;
}

int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the SPI read routine according to the target machine . */
	HAL_StatusTypeDef status = HAL_OK ;
	int32_t iError = BMP280_OK ;
	uint8_t txarray [ 28 ] = {0 ,};
	uint8_t rxarray [ 28 ] = {0 ,};
	uint8_t stringpos ;

	txarray [0] = reg_addr ;

	HAL_GPIO_WritePin ( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_RESET );
	status = HAL_SPI_TransmitReceive ( &hspi4 , ( uint8_t *)(& txarray ),( uint8_t *)(& rxarray ), length +1, 5);
	while ( hspi4 . State == HAL_SPI_STATE_BUSY ) {};
	HAL_GPIO_WritePin ( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_SET );

	for ( stringpos = 0; stringpos < length ; stringpos ++)
	{
		*( reg_data + stringpos ) = rxarray [ stringpos + BMP280_DATA_INDEX ];
	}

	if ( status != HAL_OK )
	{
	iError = ( -1);
	}
	return ( int8_t ) iError ;
}
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
	int8_t rslt;
	struct bmp280_dev bmp;
	struct bmp280_config conf;
	struct bmp280_uncomp_data ucomp_data;
	int32_t temp32;
	double temp;

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */



  bmp.delay_ms=HAL_Delay;
  bmp.dev_id = 0;
  bmp.read = spi_reg_read;
  bmp.write = spi_reg_write;
  bmp.intf = BMP280_SPI_INTF;

 rslt = bmp280_init(&bmp);


 /* Always read the current settings before writing, especially when
  * all the configuration is not modified
  */
 rslt = bmp280_get_config(&conf, &bmp);


 /* configuring the temperature oversampling, filter coefficient and output data rate */
 /* Overwrite the desired settings */
 conf.filter = BMP280_FILTER_COEFF_2;

 /* Temperature oversampling set at 4x */
 conf.os_temp = BMP280_OS_4X;

 /* Pressure over sampling none (disabling pressure measurement) */
 conf.os_pres = BMP280_OS_NONE;

 /* Setting the output data rate as 1HZ(1000ms) */
 conf.odr = BMP280_ODR_1000_MS;
 rslt = bmp280_set_config(&conf, &bmp);


 /* Always set the power mode after setting the configuration */
 rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /** Reading the raw data from sensor */
     rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

     /** Getting the compensated temperature as floating point value */
     rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
     /**Transmitting the temperature via UART*/
     size = sprintf(buffer,"Temperatura %f\n\r ", temp);
     HAL_UART_Transmit(&huart3, (uint8_t*)buffer, size, 200);

     /** Sleep time between measurements = BMP280_ODR_2000_MS */
     bmp.delay_ms(2000);

     HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, heater_state);  // Wysterowywanie Pinu grzałki
     HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, fan_state);			 // Wysterowanie Pinu wentylatora

     /**Receiving Referenced temperature*/

     HAL_UART_Receive_IT(&huart3, Received, 4);
     Control_Algorithm(TempRef, temp);								// Wywołąnie funckji algorytmu sterowania

     HAL_Delay(100);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
