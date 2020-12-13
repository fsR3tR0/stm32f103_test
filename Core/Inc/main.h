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
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef unsigned char uch;

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
#define led_0_Pin GPIO_PIN_13
#define led_0_GPIO_Port GPIOC
#define led_1_Pin GPIO_PIN_1
#define led_1_GPIO_Port GPIOA
#define led_2_Pin GPIO_PIN_2
#define led_2_GPIO_Port GPIOA
#define led_3_Pin GPIO_PIN_3
#define led_3_GPIO_Port GPIOA
#define led_4_Pin GPIO_PIN_4
#define led_4_GPIO_Port GPIOA
#define led_5_Pin GPIO_PIN_5
#define led_5_GPIO_Port GPIOA
#define led_6_Pin GPIO_PIN_6
#define led_6_GPIO_Port GPIOA
#define led_IT_Pin GPIO_PIN_8
#define led_IT_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_15
#define LCD_RS_GPIO_Port GPIOA
#define LCD_E_Pin GPIO_PIN_3
#define LCD_E_GPIO_Port GPIOB
#define LCD_DATA_4_Pin GPIO_PIN_4
#define LCD_DATA_4_GPIO_Port GPIOB
#define LCD_DATA_5_Pin GPIO_PIN_5
#define LCD_DATA_5_GPIO_Port GPIOB
#define LCD_DATA_6_Pin GPIO_PIN_6
#define LCD_DATA_6_GPIO_Port GPIOB
#define LCD_DATA_7_Pin GPIO_PIN_7
#define LCD_DATA_7_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
