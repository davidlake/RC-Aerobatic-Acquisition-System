/*
 * HC06.h
 *
 *  Created on: Feb 23, 2021
 *      Author: dlago
 */

#ifndef HC06_INC_HC06_H_
#define HC06_INC_HC06_H_

#include "main.h"

typedef enum _HC06_BAUDRATE
{
	BR_9600,
	BR_115200,
	BR_921600,
}HC06_BAUDRATE;
typedef enum _HC06_CONNECTION
{
	Connected,
	Disconnected,
}HC06_CONNECTION;
typedef enum _HC06_AT_CMD_RESULT
{
	OK = 1,
	NOK = 2,
}HC06_AT_CMD_RESULT;
typedef struct _HC06_HAL_SERIF
{
	UART_HandleTypeDef *UART_HANDLE;

}HC06_HAL_SERIF;
typedef struct _HC06_DEVICE
{
	uint8_t id;
	HC06_HAL_SERIF serif;
	GPIO_TypeDef *Power_PIN_PORT;
	uint16_t Power_PIN_NUMBER;
	uint8_t power;
	HC06_CONNECTION connectionState;
}HC06_DEVICE;

void HC06_Init_Device(HC06_DEVICE *pdev, UART_HandleTypeDef *UART_HANDLE, GPIO_TypeDef *Power_PIN_PORT, uint16_t Power_PIN_NUMBER);
HC06_AT_CMD_RESULT HC06_Send_AT_Test_Command(HC06_DEVICE *pdev);
void HC06_CheckConnectionState(HC06_DEVICE *pdev);
void HC06_PowerON(HC06_DEVICE *pdev);
void HC06_PowerOFF(HC06_DEVICE *pdev);
void HC06_ConfigBaudRate(HC06_DEVICE *pdev, HC06_BAUDRATE br);

#endif /* HC06_INC_HC06_H_ */
