/*
 * UBLOXM8N.h
 *
 *  Created on: Dec 29, 2020
 *      Author: dlago
 */

#ifndef UBLOXM8N_INC_UBLOXM8N_H_
#define UBLOXM8N_INC_UBLOXM8N_H_

#include "main.h"

typedef enum _UBLOXM8N_navResult
{
	Fix,
	NoFix,
	CHKerror
} UBLOXM8N_navResult;
typedef struct _UBLOXM8N_HAL_SERIF
{
	UART_HandleTypeDef *UART_HANDLE;

}UBLOXM8N_HAL_SERIF;
typedef struct _UBLOXM8N_DEVICE
{
	UBLOXM8N_HAL_SERIF serif;
	UBLOXM8N_navResult navResult;
	uint8_t pvtMsg[100];
	uint8_t navData[92];
} UBLOXM8N_DEVICE;

void UBLOXM8N_Init_Device(UBLOXM8N_DEVICE *pdev, UART_HandleTypeDef *UART_HANDLE);
UBLOXM8N_navResult UBLOXM8N_processPVTmsg(UBLOXM8N_DEVICE *pdev);

#endif /* UBLOXM8N_INC_UBLOXM8N_H_ */
