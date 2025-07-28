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
#define SEG2_F_Pin GPIO_PIN_13
#define SEG2_F_GPIO_Port GPIOC
#define SEG2_G_Pin GPIO_PIN_14
#define SEG2_G_GPIO_Port GPIOC
#define SEG2_DP_Pin GPIO_PIN_15
#define SEG2_DP_GPIO_Port GPIOC
#define SEG1_A_Pin GPIO_PIN_0
#define SEG1_A_GPIO_Port GPIOC
#define SEG1_B_Pin GPIO_PIN_1
#define SEG1_B_GPIO_Port GPIOC
#define SEG1_C_Pin GPIO_PIN_2
#define SEG1_C_GPIO_Port GPIOC
#define SEG1_D_Pin GPIO_PIN_3
#define SEG1_D_GPIO_Port GPIOC
#define POT_Pin GPIO_PIN_0
#define POT_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_2
#define SW1_GPIO_Port GPIOA
#define SW1_EXTI_IRQn EXTI2_IRQn
#define SW2_Pin GPIO_PIN_3
#define SW2_GPIO_Port GPIOA
#define SW2_EXTI_IRQn EXTI3_IRQn
#define SEG1_E_Pin GPIO_PIN_4
#define SEG1_E_GPIO_Port GPIOC
#define SEG1_F_Pin GPIO_PIN_5
#define SEG1_F_GPIO_Port GPIOC
#define SW3_Pin GPIO_PIN_0
#define SW3_GPIO_Port GPIOB
#define SW3_EXTI_IRQn EXTI0_IRQn
#define SW4_Pin GPIO_PIN_1
#define SW4_GPIO_Port GPIOB
#define SW4_EXTI_IRQn EXTI1_IRQn
#define D1_Pin GPIO_PIN_12
#define D1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_13
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_14
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_15
#define D4_GPIO_Port GPIOB
#define SEG1_G_Pin GPIO_PIN_6
#define SEG1_G_GPIO_Port GPIOC
#define SEG1_DP_Pin GPIO_PIN_7
#define SEG1_DP_GPIO_Port GPIOC
#define SEG2_A_Pin GPIO_PIN_8
#define SEG2_A_GPIO_Port GPIOC
#define SEG2_B_Pin GPIO_PIN_9
#define SEG2_B_GPIO_Port GPIOC
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOA
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_10
#define D7_GPIO_Port GPIOA
#define D8_Pin GPIO_PIN_11
#define D8_GPIO_Port GPIOA
#define SEG2_C_Pin GPIO_PIN_10
#define SEG2_C_GPIO_Port GPIOC
#define SEG2_D_Pin GPIO_PIN_11
#define SEG2_D_GPIO_Port GPIOC
#define SEG2_E_Pin GPIO_PIN_12
#define SEG2_E_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
