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
#include "stm32g4xx_hal.h"
#include "cmsis_os.h"
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
#define RTS_B2_Pin GPIO_PIN_13
#define RTS_B2_GPIO_Port GPIOC
#define CTS_B2_Pin GPIO_PIN_14
#define CTS_B2_GPIO_Port GPIOC
#define ENC_MOT1_VERDE_Pin GPIO_PIN_0
#define ENC_MOT1_VERDE_GPIO_Port GPIOC
#define ENC_MOT1_BLU_Pin GPIO_PIN_1
#define ENC_MOT1_BLU_GPIO_Port GPIOC
#define ENC_MOT2_VERDE_Pin GPIO_PIN_1
#define ENC_MOT2_VERDE_GPIO_Port GPIOA
#define ENC_MOT3_VERDE_Pin GPIO_PIN_4
#define ENC_MOT3_VERDE_GPIO_Port GPIOA
#define ENC_MOT2_BLU_Pin GPIO_PIN_5
#define ENC_MOT2_BLU_GPIO_Port GPIOA
#define ENC_MOT3_BLU_Pin GPIO_PIN_6
#define ENC_MOT3_BLU_GPIO_Port GPIOA
#define ADC_BATTERY_Pin GPIO_PIN_1
#define ADC_BATTERY_GPIO_Port GPIOB
#define ENC_MOT5_VERDE_Pin GPIO_PIN_2
#define ENC_MOT5_VERDE_GPIO_Port GPIOB
#define LED_RED_RIGHT_Pin GPIO_PIN_12
#define LED_RED_RIGHT_GPIO_Port GPIOB
#define LED_RED_LEFT_Pin GPIO_PIN_13
#define LED_RED_LEFT_GPIO_Port GPIOB
#define LED_WHITE_LEFT_Pin GPIO_PIN_14
#define LED_WHITE_LEFT_GPIO_Port GPIOB
#define LED_WHITE_RIGHT_Pin GPIO_PIN_15
#define LED_WHITE_RIGHT_GPIO_Port GPIOB
#define MOTOR_RL_Pin GPIO_PIN_6
#define MOTOR_RL_GPIO_Port GPIOC
#define MOTOR_FR_Pin GPIO_PIN_7
#define MOTOR_FR_GPIO_Port GPIOC
#define ADC_TEMPERATURE_Pin GPIO_PIN_8
#define ADC_TEMPERATURE_GPIO_Port GPIOA
#define MOTOR_RR_Pin GPIO_PIN_11
#define MOTOR_RR_GPIO_Port GPIOA
#define MOTOR_FL_Pin GPIO_PIN_12
#define MOTOR_FL_GPIO_Port GPIOA
#define BOARD_ALIVE_B2_Pin GPIO_PIN_13
#define BOARD_ALIVE_B2_GPIO_Port GPIOA
#define BOARD_ALIVE_B1_Pin GPIO_PIN_14
#define BOARD_ALIVE_B1_GPIO_Port GPIOA
#define ENC_MOT5_BLU_Pin GPIO_PIN_12
#define ENC_MOT5_BLU_GPIO_Port GPIOC
#define CTS_B1_Pin GPIO_PIN_6
#define CTS_B1_GPIO_Port GPIOB
#define RTS_B1_Pin GPIO_PIN_7
#define RTS_B1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
