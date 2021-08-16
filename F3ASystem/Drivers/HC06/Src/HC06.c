/*
 * HC08.c
 *
 *  Created on: Feb 23, 2021
 *      Author: dlago
 */
#include "HC06.h"
#include <string.h>

#define HC06_WAKE_DUMB_STR	{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}

void HC06_Init_Device(HC06_DEVICE *pdev, UART_HandleTypeDef *UART_HANDLE, GPIO_TypeDef *Power_PIN_PORT, uint16_t Power_PIN_NUMBER)
{
	pdev->serif.UART_HANDLE = UART_HANDLE;
	pdev->Power_PIN_PORT = Power_PIN_PORT;
	pdev->Power_PIN_NUMBER = Power_PIN_NUMBER;

	HC06_PowerON(pdev);
	HC06_CheckConnectionState(pdev);
	HC06_PowerOFF(pdev);
}
HC06_AT_CMD_RESULT HC06_Send_AT_Test_Command(HC06_DEVICE *pdev)
{
	HC06_AT_CMD_RESULT result;
	unsigned char CMD[2] = {'A','T'};
	unsigned const char okResult[2] = {'O','K'};
	unsigned char rxBuff[2];
	HAL_UART_Transmit(pdev->serif.UART_HANDLE, &CMD[0], 2, 10);
	HAL_UART_Receive(pdev->serif.UART_HANDLE, &rxBuff[0], 2, 10);
	if (strcmp((char*)&rxBuff[0],(char*)&okResult[0]) == 0) //not equal
	{
		result = NOK;
	}
	else //equal
	{
		result = OK;
	}
	return result;
}
void HC06_CheckConnectionState(HC06_DEVICE *pdev)
{
	HC06_AT_CMD_RESULT result = HC06_Send_AT_Test_Command(pdev);
	if (result == OK)
	{
		pdev->connectionState = Disconnected;
	}
	else
	{
		pdev->connectionState = Connected;
	}
}
void HC06_PowerON(HC06_DEVICE *pdev)
{
	HAL_GPIO_WritePin(pdev->Power_PIN_PORT, pdev->Power_PIN_NUMBER, GPIO_PIN_SET); //NPN transistor used
	pdev->power = 1;
}
void HC06_PowerOFF(HC06_DEVICE *pdev)
{
	HAL_GPIO_WritePin(pdev->Power_PIN_PORT, pdev->Power_PIN_NUMBER, GPIO_PIN_RESET); //NPN transistor used
	pdev->power = 0;
	pdev->connectionState = Disconnected;
}
void HC06_ConfigBaudRate(HC06_DEVICE *pdev, HC06_BAUDRATE br)
{
	char cmd[9];
	switch (br)
	{
		case BR_9600:
			strcpy(cmd,"AT+BAUD4");
			break;
		case BR_115200:
			strcpy(cmd,"AT+BAUD8");
			break;
		case BR_921600:
			strcpy(cmd,"AT+BAUDB");
			break;
		default:
			break;
	}
	HAL_UART_Transmit(pdev->serif.UART_HANDLE, (uint8_t *) &cmd[0], 8, 100);
}
