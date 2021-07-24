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
#define LED_0_Pin GPIO_PIN_13
#define LED_0_GPIO_Port GPIOC
#define EX0_START_Pin GPIO_PIN_0
#define EX0_START_GPIO_Port GPIOA
#define EX0_START_EXTI_IRQn EXTI0_IRQn
#define EX1_UP_Pin GPIO_PIN_1
#define EX1_UP_GPIO_Port GPIOA
#define EX1_UP_EXTI_IRQn EXTI1_IRQn
#define EX2_DOWN_Pin GPIO_PIN_2
#define EX2_DOWN_GPIO_Port GPIOA
#define EX2_DOWN_EXTI_IRQn EXTI2_IRQn
#define EX3_SEL_Pin GPIO_PIN_3
#define EX3_SEL_GPIO_Port GPIOA
#define EX3_SEL_EXTI_IRQn EXTI3_IRQn
#define DACA_0_Pin GPIO_PIN_4
#define DACA_0_GPIO_Port GPIOA
#define DACA_1_Pin GPIO_PIN_5
#define DACA_1_GPIO_Port GPIOA
#define DACA_2_Pin GPIO_PIN_6
#define DACA_2_GPIO_Port GPIOA
#define DACA_3_Pin GPIO_PIN_7
#define DACA_3_GPIO_Port GPIOA
#define DACB_0_Pin GPIO_PIN_0
#define DACB_0_GPIO_Port GPIOB
#define DACB_1_Pin GPIO_PIN_1
#define DACB_1_GPIO_Port GPIOB
#define DACB_2_Pin GPIO_PIN_2
#define DACB_2_GPIO_Port GPIOB
#define DACB_10_Pin GPIO_PIN_10
#define DACB_10_GPIO_Port GPIOB
#define DACB_11_Pin GPIO_PIN_11
#define DACB_11_GPIO_Port GPIOB
#define DACB_12_Pin GPIO_PIN_12
#define DACB_12_GPIO_Port GPIOB
#define DACB_13_Pin GPIO_PIN_13
#define DACB_13_GPIO_Port GPIOB
#define DACB_14_Pin GPIO_PIN_14
#define DACB_14_GPIO_Port GPIOB
#define DACB_15_Pin GPIO_PIN_15
#define DACB_15_GPIO_Port GPIOB
#define DACA_4_Pin GPIO_PIN_8
#define DACA_4_GPIO_Port GPIOA
#define DACA_5_Pin GPIO_PIN_9
#define DACA_5_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_10
#define LED_2_GPIO_Port GPIOA
#define DACB_3_Pin GPIO_PIN_3
#define DACB_3_GPIO_Port GPIOB
#define DACB_4_Pin GPIO_PIN_4
#define DACB_4_GPIO_Port GPIOB
#define DACB_5_Pin GPIO_PIN_5
#define DACB_5_GPIO_Port GPIOB
#define DACB_6_Pin GPIO_PIN_6
#define DACB_6_GPIO_Port GPIOB
#define DACB_7_Pin GPIO_PIN_7
#define DACB_7_GPIO_Port GPIOB
#define DACB_8_Pin GPIO_PIN_8
#define DACB_8_GPIO_Port GPIOB
#define DACB_9_Pin GPIO_PIN_9
#define DACB_9_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define BUFFER_SIZE 360 //buffer size for DMA Transfer
#define USB_DEVICE_START	0x55
#define USB_DEVICE_STOP		0xAA
#define USB_DEVICE_TYPE		0x00
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
