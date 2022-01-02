/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ICM_INT_Pin GPIO_PIN_0
#define ICM_INT_GPIO_Port GPIOA
#define ICM_RESV_Pin GPIO_PIN_1
#define ICM_RESV_GPIO_Port GPIOA
#define ICM_FSYNC_Pin GPIO_PIN_2
#define ICM_FSYNC_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define BKLT_EN_Pin GPIO_PIN_0
#define BKLT_EN_GPIO_Port GPIOB
#define BKLT_FEEDBACK_Pin GPIO_PIN_1
#define BKLT_FEEDBACK_GPIO_Port GPIOB
#define AK_RSTN_Pin GPIO_PIN_2
#define AK_RSTN_GPIO_Port GPIOB
#define AK_DRDY_Pin GPIO_PIN_12
#define AK_DRDY_GPIO_Port GPIOB
#define TC_VEN_Pin GPIO_PIN_14
#define TC_VEN_GPIO_Port GPIOB
#define LCD_5_5V_EN_Pin GPIO_PIN_8
#define LCD_5_5V_EN_GPIO_Port GPIOA
#define TC_RESETN_Pin GPIO_PIN_8
#define TC_RESETN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
