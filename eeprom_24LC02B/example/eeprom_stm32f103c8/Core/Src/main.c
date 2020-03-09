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
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom_24LC02B.h"

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
uint8_t data_addr = 0;	// first address to write below data to

uint8_t uint8_out = 189;
uint8_t uint8_in;
uint8_t uint8_status;

int8_t int8_out = -89;
int8_t int8_in;
uint8_t int8_status;

uint16_t uint16_out = 35987;
uint16_t uint16_in;
uint8_t uint16_status;

int16_t int16_out = -15976;
int16_t int16_in;
uint8_t int16_status;

uint32_t uint32_out = 789456123;
uint32_t uint32_in;
uint8_t uint32_status;

int32_t int32_out = -789123;
int32_t int32_in;
uint8_t int32_status;

float float_out = 69.1254;
float float_in;
uint8_t float_status;

double double_out = 7.648216478936315;
double double_in;
uint8_t double_status;

uint8_t str_out[] = "sdperry.com";
uint8_t str_in[sizeof(str_out)];
uint8_t str_status;

uint8_t buffer[255];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	// write uint8_t to eeprom, get the return status as uint8_status
	uint8_status = EEPROM_24LC02B_write_uint8(data_addr, &uint8_out);
	HAL_Delay(5);	// wait for eeprom to finish writing data
	uint8_in = EEPROM_24LC02B_read_uint8(data_addr); // read back data

	data_addr += sizeof(uint8_t);

	// write int8_t to eeprom, get the return status as int8_status
	int8_status = EEPROM_24LC02B_write_int8(data_addr, &int8_out);
	HAL_Delay(5);	// wait for eeprom to finish writing data
	int8_in = EEPROM_24LC02B_read_int8(data_addr); // read back data

	data_addr += sizeof(int8_t);

	// write uint16_t to eeprom, get the return status as uint16_status
	uint16_status = EEPROM_24LC02B_write_uint16(data_addr, &uint16_out);
	HAL_Delay(5);	// wait for eeprom to finish writing data
	uint16_in = EEPROM_24LC02B_read_uint16(data_addr);

	data_addr += sizeof(uint16_t);

	// write int16_t to eeprom, get the return status as int16_status
	int16_status = EEPROM_24LC02B_write_int16(data_addr, &int16_out);
	HAL_Delay(5);	// wait for eeprom to finish writing data
	int16_in = EEPROM_24LC02B_read_int16(data_addr);

	data_addr += sizeof(int16_t);

	// write uint32_t to eeprom, get the return status as uint32_status
	uint32_status = EEPROM_24LC02B_write_uint32(data_addr, &uint32_out);
	HAL_Delay(5);	// wait for eeprom to finish writing data
	uint32_in = EEPROM_24LC02B_read_uint32(data_addr);

	data_addr += sizeof(uint32_t);

	// write int32_t to eeprom, get the return status as int32_status
	int32_status = EEPROM_24LC02B_write_int32(data_addr, &int32_out);
	HAL_Delay(5);	// wait for eeprom to finish writing data
	int32_in = EEPROM_24LC02B_read_int32(data_addr);

	data_addr += sizeof(int32_t);

	// write float to eeprom, get the return status as float_status
	float_status = EEPROM_24LC02B_write_float(data_addr, &float_out);
	HAL_Delay(5);	// wait for eeprom to finish writing data
	float_in = EEPROM_24LC02B_read_float(data_addr);

	data_addr += sizeof(float);

	// write double to eeprom, get the return status as double_status
	double_status = EEPROM_24LC02B_write_double(data_addr, &double_out);
	HAL_Delay(5);	// wait for eeprom to finish writing data
	double_in = EEPROM_24LC02B_read_double(data_addr);

	data_addr += sizeof(double);

	// write char string to eeprom, get the return status as str_status
	str_status = EEPROM_24LC02B_write_str(data_addr, str_out, sizeof(str_out));
	HAL_Delay(5);	// wait for eeprom to finish writing data
	EEPROM_24LC02B_read_str(data_addr, str_in, sizeof(str_in));

	// read the entire eeprom
	EEPROM_24LC02B_memdump(buffer, sizeof(buffer));

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
