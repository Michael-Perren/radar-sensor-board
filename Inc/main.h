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
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct __attribute__((packed)) uart_data {
    uint16_t  address;
    uint16_t msg;
} uart_data;
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
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define Translator_OE_Pin GPIO_PIN_1
#define Translator_OE_GPIO_Port GPIOB
#define en_ldo_radar_Pin GPIO_PIN_14
#define en_ldo_radar_GPIO_Port GPIOB
#define osc_en_Pin GPIO_PIN_15
#define osc_en_GPIO_Port GPIOB
#define led_select0_Pin GPIO_PIN_8
#define led_select0_GPIO_Port GPIOA
#define led_select1_Pin GPIO_PIN_9
#define led_select1_GPIO_Port GPIOA
#define IRQ_R_M_Pin GPIO_PIN_6
#define IRQ_R_M_GPIO_Port GPIOB
#define RST_M_R_Pin GPIO_PIN_7
#define RST_M_R_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
