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
#define USER_LED_Pin GPIO_PIN_13
#define USER_LED_GPIO_Port GPIOC
#define USER_BUTTON_EXTI0_Pin GPIO_PIN_0
#define USER_BUTTON_EXTI0_GPIO_Port GPIOA
#define USER_BUTTON_EXTI0_EXTI_IRQn EXTI0_IRQn
#define TLE5012B_CS1_Pin GPIO_PIN_1
#define TLE5012B_CS1_GPIO_Port GPIOA
#define TLE5012B_CS2_Pin GPIO_PIN_2
#define TLE5012B_CS2_GPIO_Port GPIOA
#define TLE5012B_CS3_Pin GPIO_PIN_3
#define TLE5012B_CS3_GPIO_Port GPIOA
#define SPI1_FLASH_CS_Pin GPIO_PIN_4
#define SPI1_FLASH_CS_GPIO_Port GPIOA
#define SPI1_FLASH_SCK_Pin GPIO_PIN_5
#define SPI1_FLASH_SCK_GPIO_Port GPIOA
#define SPI1_FLASH_MISO_Pin GPIO_PIN_6
#define SPI1_FLASH_MISO_GPIO_Port GPIOA
#define SPI1_FLASH_MOSI_Pin GPIO_PIN_7
#define SPI1_FLASH_MOSI_GPIO_Port GPIOA
#define SPI2_TLE5012B_SCK_Pin GPIO_PIN_10
#define SPI2_TLE5012B_SCK_GPIO_Port GPIOB
#define SPI3_WS2812_SCK_Not_used_Pin GPIO_PIN_12
#define SPI3_WS2812_SCK_Not_used_GPIO_Port GPIOB
#define SPI2_TLE5012B_MOSI_Pin GPIO_PIN_15
#define SPI2_TLE5012B_MOSI_GPIO_Port GPIOB
#define TIM1_ENCODER_CH1_Pin GPIO_PIN_8
#define TIM1_ENCODER_CH1_GPIO_Port GPIOA
#define TIM1_ENCODER_CH2_Pin GPIO_PIN_9
#define TIM1_ENCODER_CH2_GPIO_Port GPIOA
#define TLE5012B_CS4_Pin GPIO_PIN_10
#define TLE5012B_CS4_GPIO_Port GPIOA
#define SPI3_WS2812_MOSI_Pin GPIO_PIN_5
#define SPI3_WS2812_MOSI_GPIO_Port GPIOB
#define I2C1_HAPTIC_SCL_Pin GPIO_PIN_6
#define I2C1_HAPTIC_SCL_GPIO_Port GPIOB
#define I2C1_HAPTIC_SDA_Pin GPIO_PIN_7
#define I2C1_HAPTIC_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
