/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "microphone_constants.h"
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
#define TMR3_RES_FREQ_KHZ 1
#define TMR3_PERIOD ((TMR3_RES_FREQ_KHZ*MIC_SETTLING_PERIOD_MS)-1)
#define TMR3_PRESCALER ((SYSCLK_FREQ_HZ/(1000*TMR3_RES_FREQ_KHZ))-1)
#define SYSCLK_FREQ_HZ 32000000U
#define USER_B1_Pin GPIO_PIN_13
#define USER_B1_GPIO_Port GPIOC
#define LD4_GREEN_Pin GPIO_PIN_5
#define LD4_GREEN_GPIO_Port GPIOA
#define DB45_Pin GPIO_PIN_0
#define DB45_GPIO_Port GPIOB
#define DB55_Pin GPIO_PIN_1
#define DB55_GPIO_Port GPIOB
#define DB65_Pin GPIO_PIN_2
#define DB65_GPIO_Port GPIOB
#define DB75_Pin GPIO_PIN_3
#define DB75_GPIO_Port GPIOB
#define DB82_Pin GPIO_PIN_4
#define DB82_GPIO_Port GPIOB
#define DB87_Pin GPIO_PIN_5
#define DB87_GPIO_Port GPIOB
#define DB92_Pin GPIO_PIN_6
#define DB92_GPIO_Port GPIOB
#define DB97_Pin GPIO_PIN_7
#define DB97_GPIO_Port GPIOB
#define DB102_Pin GPIO_PIN_8
#define DB102_GPIO_Port GPIOB
#define DBMAX_Pin GPIO_PIN_9
#define DBMAX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
