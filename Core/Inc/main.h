/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define IN4_Pin GPIO_PIN_0
#define IN4_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_1
#define IN3_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_10
#define IN2_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_11
#define IN1_GPIO_Port GPIOB
#define BTN_Pin GPIO_PIN_12
#define BTN_GPIO_Port GPIOB
#define LINE_3_Pin GPIO_PIN_13
#define LINE_3_GPIO_Port GPIOB
#define LINE_2_Pin GPIO_PIN_14
#define LINE_2_GPIO_Port GPIOB
#define LINE_1_Pin GPIO_PIN_15
#define LINE_1_GPIO_Port GPIOB
#define EN_B_Pin GPIO_PIN_8
#define EN_B_GPIO_Port GPIOA
#define EN_A_Pin GPIO_PIN_9
#define EN_A_GPIO_Port GPIOA
#define ECHO_Pin GPIO_PIN_11
#define ECHO_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_12
#define TRIG_GPIO_Port GPIOA
#define LEFT_Pin GPIO_PIN_3
#define LEFT_GPIO_Port GPIOB
#define RIGHT_Pin GPIO_PIN_4
#define RIGHT_GPIO_Port GPIOB
#define FORWARD_Pin GPIO_PIN_6
#define FORWARD_GPIO_Port GPIOB
#define BACKWARD_Pin GPIO_PIN_7
#define BACKWARD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
