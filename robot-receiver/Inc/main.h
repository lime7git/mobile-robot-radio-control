/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define USER_LED_Pin GPIO_PIN_13
#define USER_LED_GPIO_Port GPIOC
#define MOTR_PWM_Pin GPIO_PIN_1
#define MOTR_PWM_GPIO_Port GPIOA
#define SENSOR_SIDE_LEFT_Pin GPIO_PIN_3
#define SENSOR_SIDE_LEFT_GPIO_Port GPIOA
#define SENSOR_SIDE_RIGHT_Pin GPIO_PIN_4
#define SENSOR_SIDE_RIGHT_GPIO_Port GPIOA
#define SENSOR_FRONT_Pin GPIO_PIN_5
#define SENSOR_FRONT_GPIO_Port GPIOA
#define MOTL_ENC_CH1_Pin GPIO_PIN_6
#define MOTL_ENC_CH1_GPIO_Port GPIOA
#define MOTL_ENC_CH2_Pin GPIO_PIN_7
#define MOTL_ENC_CH2_GPIO_Port GPIOA
#define MOTL_IN4_Pin GPIO_PIN_12
#define MOTL_IN4_GPIO_Port GPIOB
#define MOTL_IN3_Pin GPIO_PIN_13
#define MOTL_IN3_GPIO_Port GPIOB
#define MOTR_IN2_Pin GPIO_PIN_14
#define MOTR_IN2_GPIO_Port GPIOB
#define MOTR_IN1_Pin GPIO_PIN_15
#define MOTR_IN1_GPIO_Port GPIOB
#define MOTL_PWM_Pin GPIO_PIN_8
#define MOTL_PWM_GPIO_Port GPIOA
#define nRF_CE_Pin GPIO_PIN_12
#define nRF_CE_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define MOTL_ENC_CH1B6_Pin GPIO_PIN_6
#define MOTL_ENC_CH1B6_GPIO_Port GPIOB
#define MOTR_ENC_CH2_Pin GPIO_PIN_7
#define MOTR_ENC_CH2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
