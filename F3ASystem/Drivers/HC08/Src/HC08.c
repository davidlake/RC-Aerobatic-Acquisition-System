/*
 * HC08.c
 *
 *  Created on: Feb 23, 2021
 *      Author: dlago
 */
#include "HC08.h"
#include <string.h>

#define HC08_WAKE_DUMB_STR	{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}

void HC08_Init_Device(HC08_DEVICE *pdev, UART_HandleTypeDef *UART_HANDLE, GPIO_TypeDef *Power_PIN_PORT, uint16_t Power_PIN_NUMBER)
{
	pdev->serif.UART_HANDLE = UART_HANDLE;
	pdev->Power_PIN_PORT = Power_PIN_PORT;
	pdev->Power_PIN_NUMBER = Power_PIN_NUMBER;

	HC08_Send_WakeUp_String(pdev); //string to wake up if has started on power mode 2
	HC08_AT_CMD_RESULT result = HC08_Send_AT_Test_Command(pdev);
	if (result == OK)
	{
		pdev->connectionState = Disconnected;
		HC08_Reset_Module(pdev);
		HAL_Delay(200);
//		HC08_Reset_Settings(pdev);
//		HAL_Delay(200);
		pdev->powerMode = HC08_Get_PowerMode(pdev);
	}
	else
	{
		pdev->connectionState = Connected;
	}
	HC08_PowerOFF(pdev);
}
void HC08_Send_WakeUp_String(HC08_DEVICE *pdev)
{
	uint8_t CMD[10] = HC08_WAKE_DUMB_STR;
	HAL_UART_Transmit(pdev->serif.UART_HANDLE, &CMD[0], 10, 10);
}
HC08_AT_CMD_RESULT HC08_Send_AT_Test_Command(HC08_DEVICE *pdev)
{
	HC08_AT_CMD_RESULT result;
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
HC08_AT_CMD_RESULT HC08_Reset_Settings(HC08_DEVICE *pdev)
{
	HC08_AT_CMD_RESULT result;
	unsigned char CMD[10] = {'A','T','+','D','E','F','A','U','L','T'};
	unsigned const char okResult[2] = {'O','K'};
	unsigned char rxBuff[2];
	HAL_UART_Transmit(pdev->serif.UART_HANDLE, &CMD[0], 10, 10);
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
HC08_AT_CMD_RESULT HC08_Reset_Module(HC08_DEVICE *pdev)
{
	HC08_AT_CMD_RESULT result;
	unsigned char CMD[8] = {'A','T','+','R','E','S','E','T'};
	unsigned const char okResult[2] = {'O','K'};
	unsigned char rxBuff[2];
	HAL_UART_Transmit(pdev->serif.UART_HANDLE, &CMD[0], 8, 10);
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
HC08_AT_CMD_RESULT HC08_Set_FullPowerMode(HC08_DEVICE *pdev)
{
	HC08_AT_CMD_RESULT result;
	unsigned char CMD[9] = {'A','T','+','M','O','D','E','=','0'};
	unsigned const char okResult[2] = {'O','K'};
	unsigned char rxBuff[2];
	HAL_UART_Transmit(pdev->serif.UART_HANDLE, &CMD[0], 9, 10);
	HAL_UART_Receive(pdev->serif.UART_HANDLE, &rxBuff[0], 2, 10);
	if (strcmp((char*)&rxBuff[0],(char*)&okResult[0]) == 0) //not equal
	{
		result = NOK;
	}
	else //equal
	{
		result = OK;
		pdev->powerMode = Mode0;
	}
	return result;

}
HC08_AT_CMD_RESULT HC08_Set_SleepMode(HC08_DEVICE *pdev)
{
	HC08_AT_CMD_RESULT result;
	unsigned char CMD[9] = {'A','T','+','M','O','D','E','=','2'};
	unsigned const char okResult[2] = {'O','K'};
	unsigned char rxBuff[2];
	HAL_UART_Transmit(pdev->serif.UART_HANDLE, &CMD[0], 9, 10);
	HAL_UART_Receive(pdev->serif.UART_HANDLE, &rxBuff[0], 2, 10);
	if (strcmp((char*)&rxBuff[0],(char*)&okResult[0]) == 0) //not equal
	{
		result = NOK;
	}
	else //equal
	{
		result = OK;
		pdev->powerMode = Mode2;
	}
	return result;
}
HC08_POWER_MODE HC08_Get_PowerMode(HC08_DEVICE *pdev)
{
	unsigned char CMD[9] = {'A','T','+','M','O','D','E','=','?'};
	HC08_POWER_MODE powerMode;
	HAL_UART_Transmit(pdev->serif.UART_HANDLE, &CMD[0], 9, 10);
	HAL_UART_Receive(pdev->serif.UART_HANDLE, &powerMode, 1, 10);
	return powerMode;
}
void HC08_CheckConnectionState(HC08_DEVICE *pdev)
{
	HC08_Send_WakeUp_String(pdev);
	HC08_AT_CMD_RESULT result = HC08_Send_AT_Test_Command(pdev);
	if (result == OK)
	{
		pdev->connectionState = Disconnected;
	}
	else
	{
		pdev->connectionState = Connected;
	}
}
void HC08_PowerON(HC08_DEVICE *pdev)
{
	HAL_GPIO_WritePin(pdev->Power_PIN_PORT, pdev->Power_PIN_NUMBER, GPIO_PIN_SET); //NPN transistor used
	pdev->power = 1;
}
void HC08_PowerOFF(HC08_DEVICE *pdev)
{
	HAL_GPIO_WritePin(pdev->Power_PIN_PORT, pdev->Power_PIN_NUMBER, GPIO_PIN_RESET); //NPN transistor used
	pdev->power = 0;
	pdev->connectionState = Disconnected;
}
