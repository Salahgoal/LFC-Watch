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
#define POWER_EN_Pin GPIO_PIN_3
#define POWER_EN_GPIO_Port GPIOA
#define KEY_WAKE_Pin GPIO_PIN_4
#define KEY_WAKE_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_5
#define KEY1_GPIO_Port GPIOA
#define KEY1_EXTI_IRQn EXTI9_5_IRQn
#define LCD_BLK_Pin GPIO_PIN_0
#define LCD_BLK_GPIO_Port GPIOB
#define WDOG_EN_Pin GPIO_PIN_1
#define WDOG_EN_GPIO_Port GPIOB
#define WDOG_WDI_Pin GPIO_PIN_2
#define WDOG_WDI_GPIO_Port GPIOB
#define SENSOR_SDA_Pin GPIO_PIN_13
#define SENSOR_SDA_GPIO_Port GPIOB
#define SENSOR_SCL_Pin GPIO_PIN_14
#define SENSOR_SCL_GPIO_Port GPIOB
#define LED_EN_Pin GPIO_PIN_15
#define LED_EN_GPIO_Port GPIOB
#define BLE_EN_Pin GPIO_PIN_8
#define BLE_EN_GPIO_Port GPIOA
#define EEPROM_SDA_Pin GPIO_PIN_11
#define EEPROM_SDA_GPIO_Port GPIOA
#define EEPROM_SCL_Pin GPIO_PIN_12
#define EEPROM_SCL_GPIO_Port GPIOA
#define TP_RST_Pin GPIO_PIN_15
#define TP_RST_GPIO_Port GPIOA
#define TP_SDA_Pin GPIO_PIN_4
#define TP_SDA_GPIO_Port GPIOB
#define TP_SCL_Pin GPIO_PIN_6
#define TP_SCL_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_7
#define LCD_RST_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_8
#define LCD_CS_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_9
#define LCD_DC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* 低功耗固件的三个构建档位。 */
#define LOW_POWER_PROFILE_DEBUG_NO_STOP 0U
#define LOW_POWER_PROFILE_STOP_DEBUG    1U
#define LOW_POWER_PROFILE_RELEASE       2U

/* 当前使用真实低功耗发布档。 */
#define LOW_POWER_BUILD_PROFILE LOW_POWER_PROFILE_RELEASE

#if LOW_POWER_BUILD_PROFILE == LOW_POWER_PROFILE_DEBUG_NO_STOP
#define STOP_MODE_TEST_ENABLED 0U
#define LOW_POWER_DEBUG_ENABLED 0U
#elif LOW_POWER_BUILD_PROFILE == LOW_POWER_PROFILE_STOP_DEBUG
#define STOP_MODE_TEST_ENABLED 1U
#define LOW_POWER_DEBUG_ENABLED 1U
#elif LOW_POWER_BUILD_PROFILE == LOW_POWER_PROFILE_RELEASE
#define STOP_MODE_TEST_ENABLED 1U
#define LOW_POWER_DEBUG_ENABLED 0U
#else
#error "Invalid LOW_POWER_BUILD_PROFILE"
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
