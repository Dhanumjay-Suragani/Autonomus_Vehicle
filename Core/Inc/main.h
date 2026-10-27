/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

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
#define US_ECHO_Pin GPIO_PIN_5
#define US_ECHO_GPIO_Port GPIOE
#define M_IN3_Pin GPIO_PIN_0
#define M_IN3_GPIO_Port GPIOC
#define M_IN4_Pin GPIO_PIN_1
#define M_IN4_GPIO_Port GPIOC
#define M_IN1_Pin GPIO_PIN_0
#define M_IN1_GPIO_Port GPIOB
#define M_IN2_Pin GPIO_PIN_1
#define M_IN2_GPIO_Port GPIOB
#define Left_IR_Pin GPIO_PIN_11
#define Left_IR_GPIO_Port GPIOF
#define Center_IR_Pin GPIO_PIN_12
#define Center_IR_GPIO_Port GPIOF
#define Right_IR_Pin GPIO_PIN_13
#define Right_IR_GPIO_Port GPIOF
#define RFID_RST_Pin GPIO_PIN_14
#define RFID_RST_GPIO_Port GPIOF
#define US_TRIG_Pin GPIO_PIN_12
#define US_TRIG_GPIO_Port GPIOC
#define RFID_CS_Pin GPIO_PIN_9
#define RFID_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
