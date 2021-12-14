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
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TC_ADDR 0x1E	//0x0F << 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t aTxBuffer[6] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * formats the register addresses and data how the Toshiba chip needs them
 * and then calls HAL_I2C_Master_Transmit()
 * TODO: optimize this better, possibly with premade arrays
 */
void TCFormatI2C(uint16_t regAddr, uint32_t data, uint8_t dsize)
{
	aTxBuffer[0] = (uint8_t)(regAddr >> 8);
	aTxBuffer[1] = (uint8_t)regAddr;
	aTxBuffer[2] = (uint8_t)data;
	aTxBuffer[3] = (uint8_t)(data >> 8);
	aTxBuffer[4] = (uint8_t)(data >> 16);
	aTxBuffer[5] = (uint8_t)(data >> 24);
	if ( HAL_I2C_Master_Transmit(&hi2c1, TC_ADDR, aTxBuffer, dsize+2, 10) != HAL_OK)
		Error_Handler();
	HAL_Delay(1);
}

/**
 * wrapper for reading a byte from the Toshiba chip over I2C
 */
uint8_t TCFormatReadI2C(uint16_t regAddr)
{
	aTxBuffer[0] = (uint8_t)(regAddr >> 8);
	aTxBuffer[1] = (uint8_t)regAddr;
	if ( HAL_I2C_Master_Transmit(&hi2c1, TC_ADDR, aTxBuffer, 2, 10) == HAL_OK)
		if( HAL_I2C_Master_Receive(&hi2c1, TC_ADDR, aTxBuffer, 1, 10) == HAL_OK)
			return aTxBuffer[0];
	Error_Handler();
	return 0x00;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  union {
		  float quat[4];
		  uint8_t bytes[16];
	  } buffer;
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  //Currently unknown usage:
  //GPIOA, PIN 3
  //GPIOB, PINS 5, 9
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//GPIO B9 hi prevents Toshiba from receiving
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	//GPIO B5 goes to light sensor, might be reset or input
  //HAL_Delay(10);

  //turn on LCD panel power (+/- 5.5V rail)
  //must come before bridge chip so bridge chip can configure it
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(10);

  //turn on bridge chip power rail
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_Delay(10);

  //disable bridge chip reset line
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(10);

  //start writing to bridge chip registers
  TCFormatI2C(0x0004, 0x0004, 2);
  TCFormatI2C(0x0002, 0x3F00, 2);
  TCFormatI2C(0x0002, 0x0000, 2);
  TCFormatI2C(0x0006, 0x0000, 2);
  TCFormatI2C(0x0016, 0x0F3F, 2);
  TCFormatI2C(0x8502, 0xFF, 1);
  TCFormatI2C(0x850B, 0xFF, 1);
  TCFormatI2C(0x0014, 0x0F3F, 2);
  TCFormatI2C(0x8512, 0xFE, 1);
  TCFormatI2C(0x851B, 0xFD, 1);
  TCFormatI2C(0x8410, 0x03, 1);
  TCFormatI2C(0x8413, 0x3F, 1);
  TCFormatI2C(0x8420, 0x06, 1);
  TCFormatI2C(0x84F0, 0x31, 1);
  TCFormatI2C(0x84F4, 0x01, 1);
  TCFormatI2C(0x8540, 0x12C0, 2);
  TCFormatI2C(0x8630, 0x00, 1);
  TCFormatI2C(0x8631, 0x0753, 2);
  TCFormatI2C(0x8670, 0x02, 1);
  TCFormatI2C(0x8A0C, 0x12C0, 2);
  TCFormatI2C(0x85E0, 0x01, 1);
  TCFormatI2C(0x85E3, 0x0100, 2);

  // EDID Data
  //edid needs to have a correct checksum or else it breaks the status register
  uint8_t edid[] = {0x8C, 0x00,	//edid address hardcoded as part of the array to make sending it easier
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x4C, 0xAE, 0x19, 0x10, 0x1A, 0x46, 0xDA, 0x01,	//128 bytes edid
	0x23, 0x1B, 0x01, 0x03, 0x80, 0x00, 0x00, 0x78, 0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
	0x12, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3A, 0x38, 0xC0, 0x40, 0x80, 0x16, 0x70, 0x0C, 0x28,
	0xE2, 0x00, 0xC0, 0x78, 0x00, 0x00, 0x00, 0x18, 0x02, 0x3A, 0x38, 0xC0, 0x40, 0x80, 0x16, 0x70,
	0x0C, 0x28, 0xE2, 0x00, 0xC0, 0x78, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x57,
	0x56, 0x52, 0x31, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0xFD,
	0x00, 0x14, 0x78, 0x01, 0xFF, 0x1D, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x24,
	0x02, 0x03, 0x1A, 0x71, 0x47, 0xC6, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0x23, 0x09, 0x07, 0x01,	//128 bytes cea
	0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x02, 0x3A, 0x38, 0xC0, 0x40, 0x80,
	0x16, 0x70, 0x0C, 0x28, 0xE2, 0x00, 0xC0, 0x78, 0x00, 0x00, 0x00, 0x18, 0x02, 0x3A, 0x38, 0xC0,
	0x40, 0x80, 0x16, 0x70, 0x0C, 0x28, 0xE2, 0x00, 0xC0, 0x78, 0x00, 0x00, 0x00, 0x18, 0x02, 0x3A,
	0x38, 0xC0, 0x40, 0x80, 0x16, 0x70, 0x0C, 0x28, 0xE2, 0x00, 0xC0, 0x78, 0x00, 0x00, 0x00, 0x18,
	0x02, 0x3A, 0x38, 0xC0, 0x40, 0x80, 0x16, 0x70, 0x0C, 0x28, 0xE2, 0x00, 0xC0, 0x78, 0x00, 0x00,
	0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};

  if ( HAL_I2C_Master_Transmit(&hi2c1, TC_ADDR, edid, sizeof(edid), 50) != HAL_OK)	//10ms isn't quite enough for 257 bytes, increased to 50ms
	  Error_Handler();

  HAL_Delay(1);
  TCFormatI2C(0x85EC, 0x01, 1);
  TCFormatI2C(0x8560, 0x24, 1);
  TCFormatI2C(0x8563, 0x11, 1);
  TCFormatI2C(0x8564, 0x0F, 1);
  TCFormatI2C(0x8543, 0x02, 1);
  TCFormatI2C(0x8544, 0x10, 1);
  TCFormatI2C(0x8600, 0x00, 1);
  TCFormatI2C(0x8602, 0xF3, 1);
  TCFormatI2C(0x8603, 0x02, 1);
  TCFormatI2C(0x8604, 0x0C, 1);
  TCFormatI2C(0x8606, 0x05, 1);
  TCFormatI2C(0x8607, 0x00, 1);
  TCFormatI2C(0x8652, 0x02, 1);
  TCFormatI2C(0x8671, 0x020C49BA, 4);
  TCFormatI2C(0x8675, 0x01E1B089, 4);
  TCFormatI2C(0x8680, 0x00, 1);
  TCFormatI2C(0x0016, 0x0F1F, 2);
  TCFormatI2C(0x0002, 0x0001, 2);
  TCFormatI2C(0x0002, 0x0000, 2);
  TCFormatI2C(0x0016, 0x0F3F, 2);
  TCFormatI2C(0x8502, 0xFF, 1);
  TCFormatI2C(0x850B, 0xFF, 1);
  TCFormatI2C(0x0014, 0x0F3F, 2);
  TCFormatI2C(0x0016, 0x0D3F, 2);
  TCFormatI2C(0x854A, 0x01, 1);
  TCFormatI2C(0x0002, 0x0000, 2);
  TCFormatI2C(0x0016, 0x0F3F, 2);
  TCFormatI2C(0x8502, 0xFF, 1);
  TCFormatI2C(0x850B, 0xFF, 1);
  TCFormatI2C(0x0014, 0x0F3F, 2);

  //poll for hdmi plugged in and ready
  while(TCFormatReadI2C(0x8520) != 0x9F)
	  HAL_Delay(500);	//takes about 1300 to 1400 ms for it to register if hdmi already plugged in, 0.5s delay is still fairly responsive

  TCFormatI2C(0x0004, 0x0C32, 2);
  TCFormatI2C(0x0002, 0x1200, 2);
  TCFormatI2C(0x0002, 0x0000, 2);
  TCFormatI2C(0x8A02, 0x42, 1);
  TCFormatI2C(0x0108, 0x00000001, 4);
  TCFormatI2C(0x010C, 0x00000001, 4);
  TCFormatI2C(0x02A0, 0x00000001, 4);
  TCFormatI2C(0x02AC, 0x000090B5, 4);
  TCFormatI2C(0x02A0, 0x00000003, 4);
  TCFormatI2C(0x0118, 0x00000014, 4);
  TCFormatI2C(0x0120, 0x00001388, 4);
  TCFormatI2C(0x0124, 0x00000000, 4);
  TCFormatI2C(0x0128, 0x00000101, 4);
  TCFormatI2C(0x0130, 0x00010000, 4);
  TCFormatI2C(0x0134, 0x00005000, 4);
  TCFormatI2C(0x0138, 0x00010000, 4);
  TCFormatI2C(0x013C, 0x00010000, 4);
  TCFormatI2C(0x0140, 0x00010000, 4);
  TCFormatI2C(0x0144, 0x00010000, 4);
  TCFormatI2C(0x0148, 0x00001000, 4);
  TCFormatI2C(0x014C, 0x00010000, 4);
  TCFormatI2C(0x0150, 0x00000160, 4);
  TCFormatI2C(0x0154, 0x00000001, 4);
  TCFormatI2C(0x0158, 0x000000C8, 4);
  TCFormatI2C(0x0168, 0x0000002A, 4);
  TCFormatI2C(0x0170, 0x000003A3, 4);
  TCFormatI2C(0x017C, 0x00000081, 4);
  TCFormatI2C(0x018C, 0x00000001, 4);
  TCFormatI2C(0x0190, 0x00000064, 4);
  TCFormatI2C(0x01A4, 0x00000000, 4);
  TCFormatI2C(0x01C0, 0x00000015, 4);
  TCFormatI2C(0x0214, 0x00000000, 4);
  TCFormatI2C(0x021C, 0x00000080, 4);
  TCFormatI2C(0x0224, 0x00000000, 4);
  TCFormatI2C(0x0254, 0x00000006, 4);
  TCFormatI2C(0x0258, 0x00200005, 4);
  TCFormatI2C(0x025C, 0x000B0005, 4);
  TCFormatI2C(0x0260, 0x000C0006, 4);
  TCFormatI2C(0x0264, 0x00003E80, 4);
  TCFormatI2C(0x0268, 0x0000000B, 4);
  TCFormatI2C(0x026C, 0x000B0008, 4);
  TCFormatI2C(0x0270, 0x00000020, 4);
  TCFormatI2C(0x0274, 0x0000001F, 4);
  TCFormatI2C(0x0278, 0x00060005, 4);
  TCFormatI2C(0x027C, 0x00000002, 4);
  TCFormatI2C(0x011C, 0x00000001, 4);
  TCFormatI2C(0x0500, 0x0000, 2);
  TCFormatI2C(0x0110, 0x00000016, 4);
  TCFormatI2C(0x0310, 0x00000016, 4);
  TCFormatI2C(0x0504, 0x0005, 2);
  TCFormatI2C(0x0504, 0x0011, 2);
  TCFormatI2C(0x0504, 0x0005, 2);
  TCFormatI2C(0x0504, 0x0029, 2);
  TCFormatI2C(0x5000, 0x0000, 2);
  TCFormatI2C(0x500C, 0x0000, 2);
  TCFormatI2C(0x500E, 0x0437, 2);
  TCFormatI2C(0x5080, 0x0000, 2);
  TCFormatI2C(0x508C, 0x0000, 2);
  TCFormatI2C(0x508E, 0x0437, 2);
  TCFormatI2C(0x7080, 0x0080, 2);
  TCFormatI2C(0x5008, 0x0CA8, 2);
  TCFormatI2C(0x5088, 0x0CA8, 2);
  TCFormatI2C(0x8502, 0xFF, 1);
  TCFormatI2C(0x8503, 0xFF, 1);
  TCFormatI2C(0x8504, 0xFF, 1);
  TCFormatI2C(0x8505, 0xFF, 1);
  TCFormatI2C(0x8506, 0xFF, 1);
  TCFormatI2C(0x850B, 0xFF, 1);
  TCFormatI2C(0x0014, 0x0F3F, 2);
  TCFormatI2C(0x0016, 0x0D3F, 2);
  TCFormatI2C(0x0004, 0x0C35, 2);
  TCFormatI2C(0x0006, 0x0000, 2);
  TCFormatI2C(0x0110, 0x00000006, 4);
  TCFormatI2C(0x0310, 0x00000006, 4);

  //clear interrupts
  TCFormatI2C(0x850B, 0xFF, 1);
  TCFormatI2C(0x0014, 0x0FBF, 2);


  //turn on panel backlight power
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(100);
  //PB1 seems to be an input related to back light power
  //  because using it as an output prevents backlight from turning on while driven low


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    // Send HID report
	    buffer.quat[0] = 1;	//w
	    buffer.quat[1] = 0;	//y
	    buffer.quat[2] = 0;	//z
	    buffer.quat[3] = 0; //x
	    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &(buffer.bytes), sizeof(buffer));
	    //HAL_Delay(1000);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
