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
#include "stdbool.h"
#include "hidfix.h"
#include "exmath.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TC_ADDR 0x1E	//Toshiba TC358870XBG	I2C Address: 0x0F << 1
#define AK_ADDR 0x1A	//AsahiKASEI AK09915c	I2C Address: 0x0D << 1
//#define PROX_ADDR 0x90	//Proximity Sensor 	I2C Address: 0x48 << 1	//not really necessary since it's an LCD
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buffer[15] = {0};
static quat gQ_gyroFrame = {1,0,0,0};
static vec3 gV_world_down = {0,0,1};
static vec3 gV_world_north = {1,0,0};
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
	buffer[0] = (uint8_t)(regAddr >> 8);
	buffer[1] = (uint8_t)regAddr;
	buffer[2] = (uint8_t)data;
	buffer[3] = (uint8_t)(data >> 8);
	buffer[4] = (uint8_t)(data >> 16);
	buffer[5] = (uint8_t)(data >> 24);
	if ( HAL_I2C_Master_Transmit(&hi2c1, TC_ADDR, buffer, dsize+2, 10) != HAL_OK)
		Error_Handler();
	HAL_Delay(1);	//needs some delay to properly ingest data
}

/**
 * wrapper for reading a byte from the Toshiba chip over I2C
 */
uint8_t TCFormatReadI2C(uint16_t regAddr)
{
	buffer[0] = (uint8_t)(regAddr >> 8);
	buffer[1] = (uint8_t)regAddr;
	if ( HAL_I2C_Master_Transmit(&hi2c1, TC_ADDR, buffer, 2, 10) == HAL_OK)
		if( HAL_I2C_Master_Receive(&hi2c1, TC_ADDR, buffer, 1, 10) == HAL_OK)
			return buffer[0];
	Error_Handler();
	return 0x00;
}

struct Measure
{
	int16_t x;
	int16_t y;
	int16_t z;
};

//returns true if data is valid	//TODO: clean this up a bit
bool AKGetMeasurement(struct Measure* data)
{
	buffer[0] = 0x10;	//read data
	if ( HAL_I2C_Master_Transmit(&hi2c2, AK_ADDR, buffer, 1, 10) == HAL_OK)
	{
		if( HAL_I2C_Master_Receive(&hi2c2, AK_ADDR, buffer, 9, 10) == HAL_OK)
		{
			if ((buffer[0] & 1) && !(buffer[8] & 8))	//data ready and no overflow
			{
				//magnetometer axes (x, y, z) are (right, down, front)
				//SteamVR expects (right, up, back)
				//units are 0.15 uT
				data->y = (int16_t)(buffer[1] | buffer[2] << 8);
				data->z = (int16_t)(buffer[3] | buffer[4] << 8);
				data->x = (int16_t)(buffer[5] | buffer[6] << 8);
				return true;
			}
		}
	}
	return false;
}

bool ICMGetMeasurement(vec3* accelData, vec3* gyroData, int16_t* tempData)
{
	buffer[0] = 0xBB;	//read data
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_Delay(1);
	if(HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, 15, 10)!=HAL_OK)
		return false;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(1);

	//accelerometer axes (x, y, z) are (right, down, back)
	//SteamVR expects (right, up, back)
	accelData->y = (int16_t)(buffer[ 1] << 8 | buffer[ 2]);
	accelData->z = (int16_t)(buffer[ 3] << 8 | buffer[ 4]);
	accelData->x = (int16_t)(buffer[ 5] << 8 | buffer[ 6]) * -1.0;
	*tempData    = (int16_t)(buffer[ 7] << 8 | buffer[ 8]);
	gyroData->y  = (int16_t)(buffer[ 9] << 8 | buffer[10]);
	gyroData->z  = (int16_t)(buffer[11] << 8 | buffer[12]);
	gyroData->x  = (int16_t)(buffer[14] << 8 | buffer[13]) * -1.0;

	return true;
}

union UsbBuffer
{
	float items[7];
	uint8_t bytes[28];
};

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  USBD_CUSTOM_HID_Init(&hUsbDeviceFS);
  //Currently unknown usage:
  //GPIOA, PIN 3
  //GPIOB, PINS 5, 9
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//GPIO B9 hi prevents Toshiba from receiving
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	//GPIO B5 goes to light sensor, might be reset or input
  HAL_Delay(10);
/*
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
*/

  //Initialising the imu components while we wait for hdmi to be ready


  //Mag: AsahiKASEI AK09915c (on I2C2)

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_Delay(1);
  //soft reset
  buffer[0] = 0x32;
  buffer[1] = 0x01;
  HAL_I2C_Master_Transmit(&hi2c2, AK_ADDR, buffer, 2, 100);
  HAL_Delay(1);

  //test read WhoAmI
  buffer[0] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c2, AK_ADDR, buffer, 1, 10);
  HAL_I2C_Master_Receive(&hi2c2, AK_ADDR, buffer, 2, 10);

  //control settings
  buffer[0] = 0x30;
  buffer[1] = 0x20; //enable noise filter
  buffer[2] = 0x4A;	//low noise drive & continuous measurement 200Hz

  if ( HAL_I2C_Master_Transmit(&hi2c2, AK_ADDR, buffer, 3, 10) != HAL_OK)
	  Error_Handler();

  //Gryro/Accel: InvenSense ICM-20602 (on SPI1)

  //soft reset
  buffer[0] = 0x6B;	//write power management 1
  buffer[1] = 0x80;	//set reset bit
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(1);
  if(HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, 2, 10)!=HAL_OK)
	  Error_Handler();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(1);

  //set sampling rate
  buffer[0] = 0x19;	//write sample rate divider
  buffer[1] = 0x04;	//200Hz
  buffer[2] = 0x06;	//unexplained bit, maxed gyro and temp filtering
  buffer[3] = 0x00;	//gyro scale +/- 250dps, filter bypass disabled
  buffer[4] = 0x00;	//accel scale +/- 2g
  buffer[5] = 0x36;	//32 accel sample averaging (in LP mode?), filter bypass disabled, max accel filtering
  buffer[6] = 0x70;	//128 gyro sample averaging (in LP mode?)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(1);
  if(HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, 7, 10)!=HAL_OK)
	  Error_Handler();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(1);

  //wake device
  buffer[0] = 0x6B;	//write power management 1
  buffer[1] = 0x01;	//wake device
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(1);
  if(HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, 2, 10)!=HAL_OK)
	  Error_Handler();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(1);

  /*
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
*/

  HAL_Delay(5);	//wait for accel/gyro filter to be populated
  vec3 accelVec, gyroVec;
  struct Measure magMeas;
  int16_t tempData;
  union UsbBuffer buf;
  quat q;
  //vec3 axis = {0,1,0};
  //float angle = 0;

	vec3 v_acc_meas, v_mag_meas, v_meas_world_down, v_meas_world_north, v_cross;
	quat q_correct;
	float mag_g, angle;
	float heading, bank, attitude;

	//variables related to keeping running sum
	struct Measure magBuffer[256] = {{0,0,0}};
	int32_t magSumX = 0;
	int32_t magSumY = 0;
	int32_t magSumZ = 0;
	uint8_t sumIndex = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	HAL_Delay(5);

	//keep a running sum to smooth magnetometer readings b/c AK09915 has ~10 bits worth of data for Earth's magnetic field strength
	if(AKGetMeasurement(&magMeas) == true)
	{
		magSumX -= magBuffer[sumIndex].x;
		magSumY -= magBuffer[sumIndex].y;
		magSumZ -= magBuffer[sumIndex].z;

		magBuffer[sumIndex++] = magMeas;

		magSumX += magMeas.x;
		magSumY += magMeas.y;
		magSumZ += magMeas.z;

		v_mag_meas.x = magSumX;
		v_mag_meas.y = magSumY;
		v_mag_meas.z = magSumZ;
	}
	else
		while(1);

	if(ICMGetMeasurement(&accelVec, &gyroVec, &tempData) != true)
		while(1);


	if(v3normalize( &v_acc_meas, &accelVec) == 0)
		v_acc_meas = gV_world_down;

	if(v3normalize( &v_mag_meas, &v_mag_meas) == 0)
		v_mag_meas = gV_world_north;

	//v3normalize( &v_gyro_meas, &gyroVec);

//  ---------------   measurement taken,normalized and stored in vectors: acc, mag, v_mag_meas, v_acc_meas  --------------------
#define INTEGRATING
#ifdef INTEGRATING

	// used when we incrementally maintain the sensor body attitude in the loop and apply correction to the last measured attitude each pass
	//                                 out
	Quat_RotateVec3( &gQ_gyroFrame, &v_meas_world_down, &v_acc_meas);
#else
 // 	this is to directly calculate the attitude of the body once per loop -- Comment out if incremental correction is wanted.  --
	Quat_Set( &gQ_gyroFrame, 1.0f, 0.0f, 0.0f, 0.0f );	// North and Level reference attitude --
	v_meas_world_down.x = v_acc_meas.x;
	v_meas_world_down.y = v_acc_meas.y;
	v_meas_world_down.z = v_acc_meas.z;

	v_meas_world_north.x = v_mag_meas.x;
	v_meas_world_north.y = v_mag_meas.y;
	v_meas_world_north.z = v_mag_meas.z;
#endif
	//      v_out   ,v_in1             , vin_2
	v3cross(&v_cross,&v_meas_world_down, &gV_world_down);	// create the 90 deg vector to rotate around from the measured G vector and the world view G vector

	mag_g =  v3normalize(&v_cross,&v_cross); // The magnitude can not be used as the angle to rotate-- it can only provides  0-90 degrees, not 180

	// calculate the angle to rotate, this gives the 0-180 degrees we need
	angle = fabs( clamped_acos( v3dot( &v_meas_world_down, &gV_world_down) ) ); // Angles are always positive since the rotation angle flips appropriately.

//	if(angle > 0.001)
//		angle = 0.001;
//	angle = 0;

	// With the angle to rotate and the axis to rotate around, form a rotation quaternion (q_correct) - Then apply that to the quaternion representing the
	// the sensor body( Q_gyroFrame).

	Quat_SetAxisAndAngle( &q_correct, &v_cross, angle);
	Quat_Multiply( &gQ_gyroFrame, &q_correct, &gQ_gyroFrame );

	// now the same for the mag ---------------------------------------------------
#define do_mag
#ifdef do_mag
					// in 			// out 				/in
	Quat_RotateVec3( &gQ_gyroFrame, &v_meas_world_north, &v_mag_meas);
	v_meas_world_north.z = 0.0f;	// take Z component out -- flatten to x & y only because of inclination ??
	v3normalize(&v_meas_world_north,&v_meas_world_north);

	v3cross(&v_cross,&v_meas_world_north, &gV_world_north);
	v3normalize(&v_cross,&v_cross);

	angle = fabs( clamped_acos( v3dot( &v_meas_world_north, &gV_world_north) ) ); // Angles are always positive since the rotation angle flips appropriately.

//	if(angle > 0.001)
//		angle = 0.001;
//	angle = 0;
	// Get the quat that rotates our sensor toward the world vector by the specified amount
	Quat_SetAxisAndAngle( &q_correct, &v_cross, angle);
	Quat_Multiply( &gQ_gyroFrame, &q_correct, &gQ_gyroFrame );
#endif
	Quat_ToEuler( &gQ_gyroFrame, &heading, &bank, &attitude);

	// Send HID report
	buf.items[0] = gQ_gyroFrame.w;	//w
	buf.items[1] = gQ_gyroFrame.y;	//x
	buf.items[2] = -gQ_gyroFrame.z;	//y
	buf.items[3] = -gQ_gyroFrame.x;	//z
	buf.items[4] = 0;	//vx
	buf.items[5] = 0;	//vy
	buf.items[6] = 0;	//vz

	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, buf.bytes, sizeof(buf));

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
