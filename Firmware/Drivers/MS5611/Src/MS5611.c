/*
 * MS5611.c
 *
 *  Created on: Nov 29, 2020
 *      Author: David
 */
#include "MS5611.h"

#include <math.h>

//MS5611 commands
#define MS5611_RESET_CMD		0x1E
#define MS5611_CONVD1_256		0x40
#define MS5611_CONVD1_512		0x42
#define MS5611_CONVD1_1024		0x44
#define MS5611_CONVD1_2048		0x46
#define MS5611_CONVD1_4096		0x48
#define MS5611_CONVD2_256		0x50
#define MS5611_CONVD2_512		0x52
#define MS5611_CONVD2_1024		0x54
#define MS5611_CONVD2_2048		0x56
#define MS5611_CONVD2_4096		0x58
#define MS5611_ADC_READ			0x00
#define MS5611_PROM_AD0_READ	0xA0
#define MS5611_PROM_AD1_READ 	0xA2
#define MS5611_PROM_AD2_READ 	0xA4
#define MS5611_PROM_AD3_READ 	0xA6
#define MS5611_PROM_AD4_READ 	0xA8
#define MS5611_PROM_AD5_READ 	0xAA
#define MS5611_PROM_AD6_READ	0xAC
#define MS5611_PROM_AD7_READ 	0xAE

//Private prototype functions-----------------------------------------------------------------
void MS5611_Reset(MS5611_DEVICE *pdev);
void MS5611_PROMread(MS5611_DEVICE *pdev);

//Private function definitions----------------------------------------------------------------
void MS5611_Reset(MS5611_DEVICE *pdev)
{
	uint8_t CMD = MS5611_RESET_CMD;
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_RESET);
	HAL_SPI_Transmit(pdev->serif.SPI_HANDLE, &CMD, 1, 10);
	HAL_Delay(10);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_SET);
}
void MS5611_PROMread(MS5611_DEVICE *pdev)
{
	uint8_t PROM_buffer_Tx[8] = {
			MS5611_PROM_AD0_READ,
			MS5611_PROM_AD1_READ,
			MS5611_PROM_AD2_READ,
			MS5611_PROM_AD3_READ,
			MS5611_PROM_AD4_READ,
			MS5611_PROM_AD5_READ,
			MS5611_PROM_AD6_READ,
			MS5611_PROM_AD7_READ};		//buffer that stores the PROM read commmands
	uint8_t PROM_buffer_Rx[16];	//buffer that stores the PROM read data
	uint8_t i;
	for (i=0; i<8; i++)
	{
		HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_RESET);
		HAL_SPI_Transmit(pdev->serif.SPI_HANDLE, &PROM_buffer_Tx[i], 1, 10);
		HAL_SPI_Receive(pdev->serif.SPI_HANDLE, &PROM_buffer_Rx[2*i], 2, 10);
		HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_SET);
	}
	pdev->FactoryData = (PROM_buffer_Rx[0] << 8) | PROM_buffer_Rx[1];
	for (i=0; i<=5; i++)
	{
		pdev->C[i] = (PROM_buffer_Rx[2+2*i] << 8) | PROM_buffer_Rx[3+2*i];
	}
	pdev->CRCData = (PROM_buffer_Rx[14] << 8) | PROM_buffer_Rx[15];
}

//Public functions----------------------------------------------------------------------------
void MS5611_Init_Device(MS5611_DEVICE *pdev, SPI_HandleTypeDef *SPI_HANDLE, GPIO_TypeDef *CS_PIN_PORT, uint16_t CS_PIN_NUMBER)
{
	pdev->serif.SPI_HANDLE = SPI_HANDLE;
	pdev->serif.CS_PIN_PORT = CS_PIN_PORT;
	pdev->serif.CS_PIN_NUMBER = CS_PIN_NUMBER;
	MS5611_Reset(pdev);
	MS5611_PROMread(pdev);
}
void MS5611_Set_Config(MS5611_DEVICE *pdev, MS5611_OSR_mode pressure_OSR, MS5611_OSR_mode temperature_OSR)
{
	switch(pressure_OSR)
	{
		case OSR_4096:
			pdev->Conv1_CMD = MS5611_CONVD1_4096;
			break;
		case OSR_2048:
			pdev->Conv1_CMD = MS5611_CONVD1_2048;
			break;
		case OSR_1024:
			pdev->Conv1_CMD = MS5611_CONVD1_1024;
			break;
		case OSR_512:
			pdev->Conv1_CMD = MS5611_CONVD1_512;
			break;
		case OSR_256:
			pdev->Conv1_CMD = MS5611_CONVD1_256;
			break;
	}
	switch(temperature_OSR)
	{
		case OSR_4096:
			pdev->Conv2_CMD = MS5611_CONVD2_4096;
			break;
		case OSR_2048:
			pdev->Conv2_CMD = MS5611_CONVD2_2048;
			break;
		case OSR_1024:
			pdev->Conv2_CMD = MS5611_CONVD2_1024;
			break;
		case OSR_512:
			pdev->Conv2_CMD = MS5611_CONVD2_512;
			break;
		case OSR_256:
			pdev->Conv2_CMD = MS5611_CONVD2_256;
			break;
	}
}
void MS5611_Init_ADC_Conv(MS5611_DEVICE *pdev, MS5611_VARIABLE variable)
{
	uint8_t CMD;
	if (variable == Pressure) //if starting pressure conversion
	{
		CMD = pdev->Conv1_CMD;
	}
	if (variable == Temperature) //if starting temperature conversion
	{
		CMD = pdev->Conv2_CMD;
	}
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_RESET);
	HAL_SPI_Transmit(pdev->serif.SPI_HANDLE, &CMD, 1, 10);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_SET);
	pdev->current_ADC_Var = variable;
}
float MS5611_Read_ADC(MS5611_DEVICE *pdev)
{
	uint8_t CMD = MS5611_ADC_READ;
	uint8_t ADCResult_buffer[3];
	float result;
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_RESET);
	HAL_SPI_Transmit(pdev->serif.SPI_HANDLE, &CMD, 1, 100);
	HAL_SPI_Receive(pdev->serif.SPI_HANDLE, &ADCResult_buffer[0], 3, 10);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT, pdev->serif.CS_PIN_NUMBER, GPIO_PIN_SET);
	switch(pdev->current_ADC_Var)
	{
		case Temperature:
			pdev->D[1] = (ADCResult_buffer[0] << 16) | (ADCResult_buffer[1] << 8) | ADCResult_buffer[2];
			pdev->dT = pdev->D[1]-(uint32_t)pdev->C[4]*256;
			pdev->T = 2000+(int64_t)pdev->dT*pdev->C[5]/8388608;
			pdev->OFF = (int64_t)pdev->C[1]*65536+((int64_t)pdev->C[3]*pdev->dT)/128;
			pdev->SENS = (int64_t)pdev->C[0]*32768+((int64_t)pdev->C[2]*pdev->dT)/256;
			if (pdev->T < 2000)
			{
				pdev->T2 = (int64_t)pdev->dT*pdev->dT/2147483648;
				pdev->OFF2 = 5*(((int64_t)pdev->T-2000)*(pdev->T-2000))/2;
				pdev->SENS2 = 5*(((int64_t)pdev->T-2000)*(pdev->T-2000))/4;
				if (pdev->T < -1500)
				{
					pdev->OFF2 = pdev->OFF2+7*((int64_t)pdev->T+1500)*(pdev->T+1500);
					pdev->SENS2 = pdev->SENS2+11*((int64_t)pdev->T+1500)*(pdev->T+1500)/2;
				}
			}
			else
			{
				pdev->T2 = 0;
				pdev->OFF2 = 0;
				pdev->SENS2 = 0;
			}
			pdev->OFF = pdev->OFF-pdev->OFF2;
			pdev->SENS = pdev->SENS-pdev->SENS2;
			pdev->T = pdev->T-pdev->T2;
			pdev->T_Result = (float)pdev->T/100;
			result = pdev->T_Result;
			break;
		case Pressure:
			pdev->D[0] = (ADCResult_buffer[0] << 16) | (ADCResult_buffer[1] << 8) | ADCResult_buffer[2];
			pdev->P = (pdev->D[0]*pdev->SENS/2097152-pdev->OFF)/32768;
			pdev->P_Result = (float)pdev->P/100;
			result = pdev->P_Result;
			break;
	}
	return result;
}
void MS5611_Calibrate_Altitude(MS5611_DEVICE *pdev, uint8_t calcTime)
{
	uint16_t i = 0;
	uint16_t n = 5*calcTime; //5 measurements per second
	float Pvector[n];
	float Tvector[n];

	for (i = 0; i < n; i++) //5 measurements per second
	{
		MS5611_Init_ADC_Conv(pdev, Temperature);
		HAL_Delay(10);
		Tvector[i] = MS5611_Read_ADC(pdev);
		HAL_Delay(10);
		MS5611_Init_ADC_Conv(pdev, Pressure);
		HAL_Delay(10);
		Pvector[i] = MS5611_Read_ADC(pdev);
		HAL_Delay(10);
		HAL_Delay(150);
	}

	float sumP = 0;
	float sumT = 0;

	for (i = 0; i < n; i++)
	{
		sumP = sumP + Pvector[i];
		sumT = sumT + Tvector[i];
	}

	pdev->T0 = sumT/n;
	pdev->P0 = sumP/n;
}
void MS5611_Get_DeltaAltitude(MS5611_DEVICE *pdev)
{
	pdev->deltaH = (1-pow((pdev->P_Result/pdev->P0),19.0293552021604))*(pdev->T0+273.15)*153.846153846154; //in centimeters
}
void MS5611_Get_Measurements(MS5611_DEVICE *pdev)
{
	MS5611_Init_ADC_Conv(pdev, Temperature);
	HAL_Delay(10);
	MS5611_Read_ADC(pdev);
	MS5611_Init_ADC_Conv(pdev, Pressure);
	HAL_Delay(10);
	MS5611_Read_ADC(pdev);
	MS5611_Get_DeltaAltitude(pdev);
}
