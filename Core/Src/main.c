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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_hid.h"

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
extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t convertKeys[132], convertE0Keys[126];
GPIO_PinState dataPinState;
unsigned char PS2Data = 0, bitCounter = 11, isDeviceReady = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void defineconvertKeys(void);
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
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	// Toggle onboard LED if no device connected
	while (!isDeviceReady) {
		HAL_GPIO_TogglePin(onBoardLED_GPIO_Port, onBoardLED_Pin);
		HAL_Delay(500);
	}

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(onBoardLED_GPIO_Port, onBoardLED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PS2Clock_Pin */
	GPIO_InitStruct.Pin = PS2Clock_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PS2Clock_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PS2Data_Pin */
	GPIO_InitStruct.Pin = PS2Data_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PS2Data_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : onBoardLED_Pin */
	GPIO_InitStruct.Pin = onBoardLED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(onBoardLED_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/**
 * @brief This is used by puts and printf to send data to SWV ITM console.
 */
int _write(int file, char *ptr, int len) {
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

/**
 * @brief This function defines convert key codes from PS/2 (array key) to USB HID (array value).
 * @param None
 * @retval None
 */
void defineconvertKeys() {
	// Convert keys from PS/2 (https://wiki.osdev.org/PS/2_Keyboard) to HID (https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf)
	convertKeys[1] = 66;
	convertKeys[3] = 62;
	convertKeys[4] = 60;
	convertKeys[5] = 58;
	convertKeys[6] = 59;
	convertKeys[7] = 69;
	convertKeys[9] = 67;
	convertKeys[10] = 65;
	convertKeys[11] = 63;
	convertKeys[12] = 61;
	convertKeys[13] = 43;
	convertKeys[14] = 53;
	convertKeys[17] = 226;
	convertKeys[18] = 225;
	convertKeys[20] = 224;
	convertKeys[21] = 20;
	convertKeys[22] = 30;
	convertKeys[26] = 29;
	convertKeys[27] = 22;
	convertKeys[28] = 4;
	convertKeys[29] = 26;
	convertKeys[30] = 31;
	convertKeys[33] = 6;
	convertKeys[34] = 27;
	convertKeys[35] = 7;
	convertKeys[36] = 8;
	convertKeys[37] = 33;
	convertKeys[38] = 32;
	convertKeys[41] = 44;
	convertKeys[42] = 25;
	convertKeys[43] = 9;
	convertKeys[44] = 23;
	convertKeys[45] = 21;
	convertKeys[46] = 34;
	convertKeys[49] = 17;
	convertKeys[50] = 5;
	convertKeys[51] = 11;
	convertKeys[52] = 10;
	convertKeys[53] = 28;
	convertKeys[54] = 35;
	convertKeys[58] = 16;
	convertKeys[59] = 13;
	convertKeys[60] = 24;
	convertKeys[61] = 36;
	convertKeys[62] = 37;
	convertKeys[65] = 54;
	convertKeys[66] = 14;
	convertKeys[67] = 12;
	convertKeys[68] = 18;
	convertKeys[69] = 39;
	convertKeys[70] = 38;
	convertKeys[73] = 55;
	convertKeys[74] = 56;
	convertKeys[75] = 15;
	convertKeys[76] = 51;
	convertKeys[77] = 19;
	convertKeys[78] = 45;
	convertKeys[82] = 52;
	convertKeys[84] = 47;
	convertKeys[85] = 46;
	convertKeys[88] = 57;
	convertKeys[89] = 229;
	convertKeys[90] = 40;
	convertKeys[91] = 48;
	convertKeys[93] = 49;
	convertKeys[97] = 100;
	convertKeys[102] = 42;
	convertKeys[105] = 89;
	convertKeys[107] = 92;
	convertKeys[108] = 95;
	convertKeys[112] = 98;
	convertKeys[113] = 99;
	convertKeys[114] = 90;
	convertKeys[115] = 93;
	convertKeys[116] = 94;
	convertKeys[117] = 96;
	convertKeys[118] = 41;
	convertKeys[119] = 83;
	convertKeys[120] = 68;
	convertKeys[121] = 87;
	convertKeys[122] = 91;
	convertKeys[123] = 86;
	convertKeys[124] = 85;
	convertKeys[125] = 97;
	convertKeys[126] = 71;
	convertKeys[131] = 64;

	convertE0Keys[17] = 230;
	convertE0Keys[20] = 228;
	convertE0Keys[31] = 227;
	convertE0Keys[39] = 231;
	convertE0Keys[47] = 101;
	convertE0Keys[74] = 84;
	convertE0Keys[90] = 88;
	convertE0Keys[105] = 77;
	convertE0Keys[107] = 80;
	convertE0Keys[108] = 74;
	convertE0Keys[112] = 73;
	convertE0Keys[113] = 76;
	convertE0Keys[114] = 81;
	convertE0Keys[116] = 79;
	convertE0Keys[117] = 82;
	convertE0Keys[122] = 78;
	convertE0Keys[124] = 70;
	convertE0Keys[125] = 75;
}

/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// Check if it is from line 0
	if (GPIO_Pin == GPIO_PIN_0) {
		// First bits are for keyboard identifier, no need to send them over USB
		if (!isDeviceReady) {
			isDeviceReady = 1; // Now keyboard is connected and ready
			return;
		}
		// Read data pin state
		dataPinState = HAL_GPIO_ReadPin(PS2Data_GPIO_Port, PS2Data_Pin);
		// Write pin state on onboard led
		HAL_GPIO_WritePin(onBoardLED_GPIO_Port, onBoardLED_Pin, dataPinState);
		// Fill PS/2 data variable (LSB first) also ignore start bit, stop bit and parity bits
		if ((bitCounter < 11) && (bitCounter > 2)) {
			PS2Data >>= 1;
			if (dataPinState == GPIO_PIN_SET)
				PS2Data = PS2Data | 0x80; //
			else
				PS2Data = PS2Data & 0x7f; // else store a '0'
		}
		// Check if all bits received
		if (--bitCounter == 0) {
			// Print data for debugging proposes
			printf("%x\n----------------\n", PS2Data);
			// Reset PS2Data and bitCounter
			PS2Data = 0;
			bitCounter = 11;
		}
	}
}

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
