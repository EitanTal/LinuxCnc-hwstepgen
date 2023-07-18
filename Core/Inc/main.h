/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32u5xx_hal.h"

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

void HAL_LPTIM_MspPostInit(LPTIM_HandleTypeDef *hlptim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOC
#define AD_Pin GPIO_PIN_3
#define AD_GPIO_Port GPIOA
#define AP_Pin GPIO_PIN_5
#define AP_GPIO_Port GPIOA
#define ZD_Pin GPIO_PIN_6
#define ZD_GPIO_Port GPIOA
#define ZP_Pin GPIO_PIN_7
#define ZP_GPIO_Port GPIOA
#define YD_Pin GPIO_PIN_4
#define YD_GPIO_Port GPIOC
#define YP_Pin GPIO_PIN_5
#define YP_GPIO_Port GPIOC
#define XD_Pin GPIO_PIN_0
#define XD_GPIO_Port GPIOB
#define XP_Pin GPIO_PIN_1
#define XP_GPIO_Port GPIOB
#define OE1_Pin GPIO_PIN_10
#define OE1_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_12
#define IN1_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_13
#define IN2_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_14
#define IN3_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_15
#define IN4_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_6
#define OUT1_GPIO_Port GPIOC
#define OUT2_Pin GPIO_PIN_7
#define OUT2_GPIO_Port GPIOC
#define OUT3_Pin GPIO_PIN_8
#define OUT3_GPIO_Port GPIOC
#define OUT4_Pin GPIO_PIN_9
#define OUT4_GPIO_Port GPIOC
#define SPINDLE_PWM_Pin GPIO_PIN_8
#define SPINDLE_PWM_GPIO_Port GPIOA
#define DATA_REQUEST_Pin GPIO_PIN_12
#define DATA_REQUEST_GPIO_Port GPIOC
#define DATA_READY_Pin GPIO_PIN_2
#define DATA_READY_GPIO_Port GPIOD
#define SPI1_MOSI_ALIAS_Pin GPIO_PIN_8
#define SPI1_MOSI_ALIAS_GPIO_Port GPIOB
#define EXT_RESET_Pin GPIO_PIN_9
#define EXT_RESET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef __uint32_t uint32_t; // stupid VSCODE
#define AXES 4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
