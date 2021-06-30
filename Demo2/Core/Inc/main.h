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
#include "stm32f0xx_hal.h"

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
#define DI3_Pin GPIO_PIN_0
#define DI3_GPIO_Port GPIOA
#define DO3_Pin GPIO_PIN_1
#define DO3_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOA
#define DO2_Pin GPIO_PIN_11
#define DO2_GPIO_Port GPIOA
#define DI2_Pin GPIO_PIN_12
#define DI2_GPIO_Port GPIOA
#define CS2_Pin GPIO_PIN_13
#define CS2_GPIO_Port GPIOA
#define SCK2_Pin GPIO_PIN_14
#define SCK2_GPIO_Port GPIOA
#define SCK1_Pin GPIO_PIN_15
#define SCK1_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_3
#define CS1_GPIO_Port GPIOB
#define DO1_Pin GPIO_PIN_4
#define DO1_GPIO_Port GPIOB
#define DI1_Pin GPIO_PIN_5
#define DI1_GPIO_Port GPIOB
#define SCK3_Pin GPIO_PIN_6
#define SCK3_GPIO_Port GPIOB
#define CS3_Pin GPIO_PIN_7
#define CS3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
