/*
 * HC08.h
 *
 *  Created on: Feb 23, 2021
 *      Author: dlago
 */

#ifndef HC08_INC_HC08_H_
#define HC08_INC_HC08_H_

#include "main.h"

typedef enum _HC08_POWER_MODE
{
	Mode0 = '0', //full power mode
	Mode1 = '1',
	Mode2 = '2', //low power mode --> goes to sleep after x seconds of inactivity
}HC08_POWER_MODE;
typedef enum _HC08_CONNECTION
{
	Connected,
	Disconnected,
}HC08_CONNECTION;
typedef enum _HC08_AT_CMD_RESULT
{
	OK = 1,
	NOK = 2,
}HC08_AT_CMD_RESULT;
typedef struct _HC08_HAL_SERIF
{
	UART_HandleTypeDef *UART_HANDLE;

}HC08_HAL_SERIF;
typedef struct _HC08_DEVICE
{
	uint8_t id;
	HC08_HAL_SERIF serif;
	GPIO_TypeDef *Power_PIN_PORT;
	uint16_t Power_PIN_NUMBER;
	uint8_t power;
	HC08_POWER_MODE powerMode;
	HC08_CONNECTION connectionState;
}HC08_DEVICE;

void HC08_Init_Device(HC08_DEVICE *pdev, UART_HandleTypeDef *UART_HANDLE, GPIO_TypeDef *Power_PIN_PORT, uint16_t Power_PIN_NUMBER);
void HC08_Send_WakeUp_String(HC08_DEVICE *pdev);
HC08_AT_CMD_RESULT HC08_Send_AT_Test_Command(HC08_DEVICE *pdev);
HC08_AT_CMD_RESULT HC08_Reset_Settings(HC08_DEVICE *pdev);
HC08_AT_CMD_RESULT HC08_Reset_Module(HC08_DEVICE *pdev);
HC08_AT_CMD_RESULT HC08_Set_FullPowerMode(HC08_DEVICE *pdev);
HC08_AT_CMD_RESULT HC08_Set_SleepMode(HC08_DEVICE *pdev);
HC08_POWER_MODE HC08_Get_PowerMode(HC08_DEVICE *pdev);
void HC08_CheckConnectionState(HC08_DEVICE *pdev);
void HC08_PowerON(HC08_DEVICE *pdev);
void HC08_PowerOFF(HC08_DEVICE *pdev);

#endif /* HC08_INC_HC08_H_ */
