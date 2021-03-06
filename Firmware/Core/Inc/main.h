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
#define ICM20948_SPI3_CS_Pin GPIO_PIN_15
#define ICM20948_SPI3_CS_GPIO_Port GPIOC
#define UBLOX_USART2_TX_Pin GPIO_PIN_2
#define UBLOX_USART2_TX_GPIO_Port GPIOA
#define UBLOX_USART2_RX_Pin GPIO_PIN_3
#define UBLOX_USART2_RX_GPIO_Port GPIOA
#define W25Q64_SPI1_SCK_Pin GPIO_PIN_5
#define W25Q64_SPI1_SCK_GPIO_Port GPIOA
#define W25Q64_SPI1_MISO_Pin GPIO_PIN_6
#define W25Q64_SPI1_MISO_GPIO_Port GPIOA
#define W25Q64_SPI1_MOSI_Pin GPIO_PIN_7
#define W25Q64_SPI1_MOSI_GPIO_Port GPIOA
#define W25Q64_SPI1_CS_Pin GPIO_PIN_0
#define W25Q64_SPI1_CS_GPIO_Port GPIOB
#define MS5611_SPI2_CS_Pin GPIO_PIN_1
#define MS5611_SPI2_CS_GPIO_Port GPIOB
#define MS5611_SPI2_SCK_Pin GPIO_PIN_10
#define MS5611_SPI2_SCK_GPIO_Port GPIOB
#define ICM20948_SPI3_SCK_Pin GPIO_PIN_12
#define ICM20948_SPI3_SCK_GPIO_Port GPIOB
#define Buzzer_PWR_CTRL_Pin GPIO_PIN_13
#define Buzzer_PWR_CTRL_GPIO_Port GPIOB
#define MS5611_SPI2_MISO_Pin GPIO_PIN_14
#define MS5611_SPI2_MISO_GPIO_Port GPIOB
#define MS5611_SPI2_MOSI_Pin GPIO_PIN_15
#define MS5611_SPI2_MOSI_GPIO_Port GPIOB
#define HC06_USART1_TX_Pin GPIO_PIN_9
#define HC06_USART1_TX_GPIO_Port GPIOA
#define HC06_USART1_RX_Pin GPIO_PIN_10
#define HC06_USART1_RX_GPIO_Port GPIOA
#define Test_IMU_Pin GPIO_PIN_12
#define Test_IMU_GPIO_Port GPIOA
#define HC06_PWR_CTRL_Pin GPIO_PIN_15
#define HC06_PWR_CTRL_GPIO_Port GPIOA
#define ICM20948_SPI3_MISO_Pin GPIO_PIN_4
#define ICM20948_SPI3_MISO_GPIO_Port GPIOB
#define ICM20948_SPI3_MOSI_Pin GPIO_PIN_5
#define ICM20948_SPI3_MOSI_GPIO_Port GPIOB
#define RC_PWM_TIM4_CH1_Pin GPIO_PIN_6
#define RC_PWM_TIM4_CH1_GPIO_Port GPIOB
#define Test_Baro_Pin GPIO_PIN_7
#define Test_Baro_GPIO_Port GPIOB
#define Test_GPS_Pin GPIO_PIN_8
#define Test_GPS_GPIO_Port GPIOB
#define Test_Flash_Pin GPIO_PIN_9
#define Test_Flash_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
