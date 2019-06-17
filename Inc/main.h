/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Mikro1_Pin GPIO_PIN_1
#define Mikro1_GPIO_Port GPIOA
#define Mikro2_Pin GPIO_PIN_2
#define Mikro2_GPIO_Port GPIOA
#define Mikro3_Pin GPIO_PIN_3
#define Mikro3_GPIO_Port GPIOA
#define Krancowka1_Pin GPIO_PIN_4
#define Krancowka1_GPIO_Port GPIOA
#define Krancowka1_EXTI_IRQn EXTI4_IRQn
#define PoziomHalasu_Pin GPIO_PIN_12
#define PoziomHalasu_GPIO_Port GPIOB
#define PoziomHalasu_EXTI_IRQn EXTI15_10_IRQn
#define A3_Pin GPIO_PIN_13
#define A3_GPIO_Port GPIOB
#define A4_Pin GPIO_PIN_14
#define A4_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_15
#define A1_GPIO_Port GPIOB
#define EN12_Pin GPIO_PIN_8
#define EN12_GPIO_Port GPIOA
#define EN34_Pin GPIO_PIN_9
#define EN34_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_10
#define A2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
