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
#define AIR_PLUS_Pin GPIO_PIN_0
#define AIR_PLUS_GPIO_Port GPIOC
#define AIR_MINUS_Pin GPIO_PIN_1
#define AIR_MINUS_GPIO_Port GPIOC
#define AIR_PRE_Pin GPIO_PIN_2
#define AIR_PRE_GPIO_Port GPIOC
#define SAFETY_DETECTION_Pin GPIO_PIN_3
#define SAFETY_DETECTION_GPIO_Port GPIOC
#define AMS_FAULT_MCU_Pin GPIO_PIN_4
#define AMS_FAULT_MCU_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOB
#define SPI2_CS1_Pin GPIO_PIN_6
#define SPI2_CS1_GPIO_Port GPIOC
#define SPI2_CS2_Pin GPIO_PIN_7
#define SPI2_CS2_GPIO_Port GPIOC
#define CARD_DETECT_Pin GPIO_PIN_8
#define CARD_DETECT_GPIO_Port GPIOA
#define DIN_1_Pin GPIO_PIN_3
#define DIN_1_GPIO_Port GPIOB
#define DIN_2_Pin GPIO_PIN_4
#define DIN_2_GPIO_Port GPIOB
#define DIN_3_Pin GPIO_PIN_5
#define DIN_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
