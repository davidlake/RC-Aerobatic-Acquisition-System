/*
 * UBLOXM8N.c
 *
 *  Created on: Dec 29, 2020
 *      Author: dlago
 */

#include "UBLOXM8N.h"
#include <string.h>

#define UBX_NAV_PVT	{0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19}

//Definitions for UBLOXM8N UBX protocol
#define UBX_PORT_ID_UART1 0x01

uint8_t UBLOXM8N_checkSum(uint8_t buff[],uint8_t CKA, uint8_t CKB);

void UBLOXM8N_Init_Device(UBLOXM8N_DEVICE *pdev, UART_HandleTypeDef *UART_HANDLE)
{
	pdev->serif.UART_HANDLE = UART_HANDLE;
	//UBX device always start at 9600 baud when powered on and NMEA protocol running
	//UBX_CFG_PRT config: NMEA disabled (only UBX protocol) and 115200 baudrate
	uint8_t buffer_Tx1[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 0x42};
	HAL_UART_Transmit(pdev->serif.UART_HANDLE, &buffer_Tx1[0], 28, 100);
	HAL_Delay(100);
	//Change MCU UART baudrate
	pdev->serif.UART_HANDLE->Init.BaudRate = 115200;
	HAL_UART_Init(pdev->serif.UART_HANDLE);
	//Set nav rate to 5Hz
	uint8_t buffer_Tx2[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
	HAL_UART_Transmit(pdev->serif.UART_HANDLE, &buffer_Tx2[0], 14, 100);
	HAL_Delay(100);
	//Activate PVT periodic message
	uint8_t buffer_Tx4[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};
	HAL_UART_Transmit(pdev->serif.UART_HANDLE, &buffer_Tx4[0], 11, 1000);
	//Enable idle line uart interrupt in the MCU
	//__HAL_UART_ENABLE_IT(pdev->serif.UART_HANDLE, UART_IT_IDLE);

}
uint8_t UBLOXM8N_checkSum(uint8_t buff[],uint8_t CKA, uint8_t CKB)
{
	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	uint8_t n = 96;
	uint8_t i;
	uint8_t result;
	for (i = 0; i < n; ++i)
	{
		CK_A = CK_A + buff[i];
		CK_B = CK_B + CK_A;
	}
	if ((CKA == CK_A) & (CKB == CK_B))
	{
		result = 1;
	}
	else
	{
		result = 0;
	}
	return result;
}
UBLOXM8N_navResult UBLOXM8N_processPVTmsg(UBLOXM8N_DEVICE *pdev)
{
	uint8_t bufferChk[96];
	memcpy(&(pdev->navData),&(pdev->pvtMsg[6]),92);
	memcpy(&bufferChk[0],&(pdev->pvtMsg[2]),96);
	uint8_t chkSumResult = UBLOXM8N_checkSum(bufferChk, pdev->pvtMsg[98],pdev->pvtMsg[99]);
	UBLOXM8N_navResult navResult;
	if (chkSumResult)
	{
		if (pdev->pvtMsg[27] & 1) // checking gnssFixOk flag (byte 21+6 overhead)
		{
			navResult = Fix;
		}
		else
		{
			navResult = NoFix;
		}
	}
	else
	{
		navResult = CHKerror;
	}
	pdev->navResult = navResult;
	return navResult;
}
