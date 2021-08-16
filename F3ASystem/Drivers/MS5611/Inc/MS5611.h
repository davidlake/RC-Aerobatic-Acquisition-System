/*
 * MS5611.h
 *
 *  Created on: Nov 29, 2020
 *      Author: David
 */

#ifndef _MS5611_H_
#define _MS5611_H_

#include "main.h"

typedef enum _MS5611_OSR_MODE
{
	OSR_4096,
	OSR_2048,
	OSR_1024,
	OSR_512,
	OSR_256
} MS5611_OSR_mode;
typedef enum _MS5611_VARIABLE
{
	Pressure = 0,
	Temperature = 1,
}MS5611_VARIABLE;
typedef struct _MS5611_HAL_SERIF
{
	SPI_HandleTypeDef *SPI_HANDLE;
	GPIO_TypeDef *CS_PIN_PORT;
	uint16_t CS_PIN_NUMBER;
} MS5611_HAL_SERIF;
typedef struct _MS5611_DEVICE
{
	MS5611_HAL_SERIF serif;
	uint16_t FactoryData;
	uint16_t C[6];
	uint16_t CRCData;
	uint8_t Conv1_CMD;
	uint8_t Conv2_CMD;
	MS5611_VARIABLE current_ADC_Var;
	uint32_t D[2]; //D[0]: pressure; D[1]: temperature
	int32_t dT;
	int32_t T;
	int32_t T2;
	int64_t OFF;
	int64_t OFF2;
	int64_t SENS;
	int64_t SENS2;
	int32_t P;
	float T_Result;
	float P_Result;
	float T0;
	float P0;
	float deltaH;
}MS5611_DEVICE;


void MS5611_Init_Device(MS5611_DEVICE *pdev, SPI_HandleTypeDef *SPI_HANDLE, GPIO_TypeDef *CS_PIN_PORT, uint16_t CS_PIN_NUMBER);
void MS5611_Set_Config(MS5611_DEVICE *pdev, MS5611_OSR_mode pressure_OSR, MS5611_OSR_mode temperature_OSR);
void MS5611_Init_ADC_Conv(MS5611_DEVICE *pdev, MS5611_VARIABLE variable);
float MS5611_Read_ADC(MS5611_DEVICE *pdev);
void MS5611_Calibrate_Altitude(MS5611_DEVICE *pdev, uint8_t calcTime); //blocking routine. calcTime integer seconds
void MS5611_Get_DeltaAltitude(MS5611_DEVICE *pdev); //delta altitude in cm
void MS5611_Get_Measurements(MS5611_DEVICE *pdev); //blocking routine to get all measurements with a single function (after init and calibration)

//EXAMPLES
//  MS5611_Init_Device(&MS5611_Dev, &hspi2, MS5611_SPI2_CS_GPIO_Port, MS5611_SPI2_CS_Pin);
//  MS5611_Set_Config(&MS5611_Dev, OSR_4096, OSR_4096);

//	MS5611_Init_ADC_Conv(&MS5611_Dev, Temperature);
//	HAL_Delay(10);
//	MS5611_Read_ADC(&MS5611_Dev);
//	MS5611_Init_ADC_Conv(&MS5611_Dev, Pressure);
//	HAL_Delay(10);
//	MS5611_Read_ADC(&MS5611_Dev);
//	MS5611_Get_DeltaAltitude(&MS5611_Dev);

//  MS5611_Calibrate_Altitude(&MS5611_Dev, 5);
//  int i;
//  for (i=0; i<50; i++)
//  {
//	  MS5611_Get_Measurements(&MS5611_Dev);
//	  HAL_Delay(500);
//  }

#endif /* _MS5611_H_ */
