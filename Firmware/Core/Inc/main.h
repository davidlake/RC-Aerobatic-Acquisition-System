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
void UART_IdleLineCallback(UART_HandleTypeDef *huart);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define IMU_SPI3_CS_Pin GPIO_PIN_15
#define IMU_SPI3_CS_GPIO_Port GPIOC
#define UserButton_Pin GPIO_PIN_0
#define UserButton_GPIO_Port GPIOA
#define UserButton_EXTI_IRQn EXTI0_IRQn
#define GPS_USART2_TX_Pin GPIO_PIN_2
#define GPS_USART2_TX_GPIO_Port GPIOA
#define GPS_USART2_RX_Pin GPIO_PIN_3
#define GPS_USART2_RX_GPIO_Port GPIOA
#define FLASH_SPI1_SCK_Pin GPIO_PIN_5
#define FLASH_SPI1_SCK_GPIO_Port GPIOA
#define FLASH_SPI1_MISO_Pin GPIO_PIN_6
#define FLASH_SPI1_MISO_GPIO_Port GPIOA
#define FLASH_SPI1_MOSI_Pin GPIO_PIN_7
#define FLASH_SPI1_MOSI_GPIO_Port GPIOA
#define FLASH_SPI1_CS_Pin GPIO_PIN_0
#define FLASH_SPI1_CS_GPIO_Port GPIOB
#define MS5611_SPI2_CS_Pin GPIO_PIN_1
#define MS5611_SPI2_CS_GPIO_Port GPIOB
#define MS5611_SPI2_SCK_Pin GPIO_PIN_10
#define MS5611_SPI2_SCK_GPIO_Port GPIOB
#define IMU_SPI3_SCK_Pin GPIO_PIN_12
#define IMU_SPI3_SCK_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_13
#define Buzzer_GPIO_Port GPIOB
#define MS5611_SPI2_MISO_Pin GPIO_PIN_14
#define MS5611_SPI2_MISO_GPIO_Port GPIOB
#define MS5611_SPI2_MOSI_Pin GPIO_PIN_15
#define MS5611_SPI2_MOSI_GPIO_Port GPIOB
#define BLE_USART1_TX_Pin GPIO_PIN_9
#define BLE_USART1_TX_GPIO_Port GPIOA
#define BLE_USART1_RX_Pin GPIO_PIN_10
#define BLE_USART1_RX_GPIO_Port GPIOA
#define Test_Pin GPIO_PIN_12
#define Test_GPIO_Port GPIOA
#define BLE_Power_Pin GPIO_PIN_15
#define BLE_Power_GPIO_Port GPIOA
#define IMU_SPI3_MISO_Pin GPIO_PIN_4
#define IMU_SPI3_MISO_GPIO_Port GPIOB
#define IMU_SPI3_MOSI_Pin GPIO_PIN_5
#define IMU_SPI3_MOSI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/