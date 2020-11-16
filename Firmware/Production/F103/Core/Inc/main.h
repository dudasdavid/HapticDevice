/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define B_AUX_1_Pin GPIO_PIN_2
#define B_AUX_1_GPIO_Port GPIOA
#define B_AUX_1_EXTI_IRQn EXTI2_IRQn
#define B_AUX_2_Pin GPIO_PIN_3
#define B_AUX_2_GPIO_Port GPIOA
#define B_AUX_2_EXTI_IRQn EXTI3_IRQn
#define B_AUX_3_Pin GPIO_PIN_4
#define B_AUX_3_GPIO_Port GPIOA
#define B_AUX_3_EXTI_IRQn EXTI4_IRQn
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOB
#define B1_EXTI_IRQn EXTI0_IRQn
#define B2_Pin GPIO_PIN_1
#define B2_GPIO_Port GPIOB
#define B2_EXTI_IRQn EXTI1_IRQn
#define CS1_Pin GPIO_PIN_12
#define CS1_GPIO_Port GPIOB
#define CS3_Pin GPIO_PIN_13
#define CS3_GPIO_Port GPIOB
#define CS4_Pin GPIO_PIN_14
#define CS4_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_15
#define CS2_GPIO_Port GPIOB
#define B_AUX_4_Pin GPIO_PIN_5
#define B_AUX_4_GPIO_Port GPIOB
#define B_AUX_4_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
