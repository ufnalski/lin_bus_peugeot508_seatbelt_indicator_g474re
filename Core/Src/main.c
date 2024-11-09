/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "rng.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ------------------------------ choose your hardware here (BEGIN) ------------
#define LIN_V1_x // Peugeot 207
//#define LIN_V2_x // Peugeot 508
// ------------------------------ choose your hardware here (END) --------------

#define FRONT_LEFT_SEATBELT_WARNING 1
#define FRONT_LEFT_SEATBELT_BLINKING 0
#define FRONT_RIGHT_SEATBELT_WARNING 0
#define FRONT_RIGHT_SEATBELT_BLINKING 0

#define REAR_LEFT_SEATBELT_WARNING 1
#define REAR_LEFT_SEATBELT_BLINKING 0
#define REAR_MIDDLE_SEATBELT_WARNING 0
#define REAR_MIDDLE_SEATBELT_BLINKING 0
#define REAR_RIGHT_SEATBELT_WARNING 1
#define REAR_RIGHT_SEATBELT_BLINKING 0

#define PASSENGER_AIRBAG_ON 0
#define PASSENGER_AIRBAG_OFF 1

#define RANDOMIZE_INDICATOR
#define RANDOM_CHANGE_PERIOD 3000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t txData[20];
uint32_t rng_number;
volatile uint8_t rng_number_flag = 0;
uint32_t RngSoftTimer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t ProtectedIdCalc(uint8_t msg_id);
uint8_t LinChecksumCalc(uint8_t pid, uint8_t *data, uint8_t size);
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
	MX_LPUART1_UART_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	MX_RNG_Init();
	/* USER CODE BEGIN 2 */
	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(20, 2);
	ssd1306_WriteString("ufnalski.edu.pl", Font_6x8, White);
	ssd1306_SetCursor(18, 12);
	ssd1306_WriteString("LIN 1.0 bus demo", Font_6x8, White);
	ssd1306_SetCursor(28, 22);
	ssd1306_WriteString("Peugeot 207", Font_6x8, White);
	ssd1306_SetCursor(18, 32);
	ssd1306_WriteString("LIN 2.0 bus demo", Font_6x8, White);
	ssd1306_SetCursor(28, 42);
	ssd1306_WriteString("Peugeot 508", Font_6x8, White);
	ssd1306_SetCursor(15, 52);
	ssd1306_WriteString("seatbelt indicator", Font_6x8, White);
	ssd1306_UpdateScreen();

	HAL_RNG_GenerateRandomNumber_IT(&hrng);
	RngSoftTimer = HAL_GetTick();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

#ifdef RANDOMIZE_INDICATOR
		if ((rng_number_flag == 1)
				&& (HAL_GetTick() - RngSoftTimer > RANDOM_CHANGE_PERIOD))
		{
			RngSoftTimer = HAL_GetTick();
			rng_number_flag = 0;
			rng_number = HAL_RNG_ReadLastRandomNumber(&hrng);
			HAL_RNG_GenerateRandomNumber_IT(&hrng);
		}
#endif

		txData[0] = 0x55;  // sync field
		txData[1] = ProtectedIdCalc(0x30);

		if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
		{
			txData[2] = ((FRONT_LEFT_SEATBELT_WARNING << 0)
					| (FRONT_LEFT_SEATBELT_BLINKING << 1)
					| (FRONT_RIGHT_SEATBELT_WARNING << 2)
					| (FRONT_RIGHT_SEATBELT_BLINKING << 3)
					| (REAR_LEFT_SEATBELT_WARNING << 4)
					| (REAR_LEFT_SEATBELT_BLINKING << 5)
					| (REAR_MIDDLE_SEATBELT_WARNING << 6)
					| (REAR_MIDDLE_SEATBELT_BLINKING << 7)) ^ (rng_number); // data byte 0
			txData[3] = ((REAR_RIGHT_SEATBELT_WARNING << 0)
					| (REAR_RIGHT_SEATBELT_BLINKING << 1)
					| (PASSENGER_AIRBAG_OFF << 6)) ^ (rng_number >> 8);
			txData[4] = 0xAA;
			txData[5] = 0xAA;
			txData[6] = 0xAA;
			txData[7] = 0xAA;
			txData[8] = 0xAA;
			txData[9] = (PASSENGER_AIRBAG_ON << 0) ^ (rng_number >> 16); // data byte 7
		}
		else
		{
			memset(txData + 2, 0x00, 8);
		}

#ifdef LIN_V1_x
	  			txData[10] = LinChecksumCalc(0 /* no PID included */, txData + 2, 8); //Lin 1.x checksum does not include PID
	  	#endif
#ifdef LIN_V2_x
		txData[10] = LinChecksumCalc(txData[1], TxData + 2, 8); //Lin 2.x checksum includes PID
#endif

		HAL_LIN_SendBreak(&huart3);
		HAL_UART_Transmit(&huart3, txData, 11, 500);

		HAL_Delay(500);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 12;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_RNG_ReadyDataCallback(RNG_HandleTypeDef *hrng, uint32_t random32bit)
{
	rng_number_flag = 1;
}

// https://controllerstech.com/stm32-uart-8-lin-protocol-part-1/
uint8_t ProtectedIdCalc(uint8_t msg_id)
{
	if (msg_id > 0x3F)
		Error_Handler();
	uint8_t IDBuf[6];
	for (int i = 0; i < 6; i++)
	{
		IDBuf[i] = (msg_id >> i) & 0x01;
	}

	uint8_t P0 = (IDBuf[0] ^ IDBuf[1] ^ IDBuf[2] ^ IDBuf[4]) & 0x01;
	uint8_t P1 = ~((IDBuf[1] ^ IDBuf[3] ^ IDBuf[4] ^ IDBuf[5]) & 0x01);

	msg_id = msg_id | (P0 << 6) | (P1 << 7);
	return msg_id;
}

// https://controllerstech.com/stm32-uart-8-lin-protocol-part-1/
uint8_t LinChecksumCalc(uint8_t pid, uint8_t *data, uint8_t size)
{
	uint8_t buffer[size + 2];
	uint16_t sum = 0;
	buffer[0] = pid;
	for (int i = 0; i < size; i++)
	{
		buffer[i + 1] = data[i];
	}

	for (int i = 0; i < size + 1; i++)
	{
		sum = sum + buffer[i];
		if (sum > 0xFF)
			sum = sum - 0xFF;
	}

	sum = 0xFF - sum;
	return sum;
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
