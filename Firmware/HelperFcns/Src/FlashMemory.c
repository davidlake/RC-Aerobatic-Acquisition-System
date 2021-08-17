/*
 * FlashMemory.c
 *
 *  Created on: 23 may. 2021
 *      Author: dlago
 */

#include "FlashMemory.h"
#include <string.h>

HAL_StatusTypeDef eraseSector_InternalFlash(uint32_t sectorNumber)
{
	HAL_StatusTypeDef status;
	if (sectorNumber < 8)
	{
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t sectorError;
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.Sector = sectorNumber; //7: Addresses 0x08060000 to 0x0807FFFF (last sector)
		EraseInitStruct.NbSectors = 1; //only erase last sector
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

		HAL_FLASH_Unlock();

		if (HAL_FLASHEx_Erase(&EraseInitStruct, &sectorError) != HAL_OK) //erase last sector of memory
		{
			status = HAL_FLASH_GetError();
		}

		HAL_FLASH_Lock();
	}
	else
	{
		status = HAL_ERROR;
	}
	return status;
}

HAL_StatusTypeDef writeStringData_InternalFlash(uint32_t flashAddress, char *pdata)
{
	HAL_StatusTypeDef status = HAL_OK;
	volatile uint32_t nWords = strlen(pdata)/4 + (strlen(pdata)%4 != 0); //calculates number or words (32bits) needed
	volatile uint32_t data32[nWords];
	strcpy((char*)data32, pdata);

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t sectorError;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = FLASH_SECTOR_7; //Addresses 0x08060000 to 0x0807FFFF (last sector)
	EraseInitStruct.NbSectors = 1; //only erase last sector
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();
	//HAL_FLASH_OB_Unlock();

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &sectorError) != HAL_OK) //erase last sector of memory
	{
		status = HAL_FLASH_GetError();
	}

	volatile uint32_t i;
	volatile uint32_t addCnt = 0;
	for (i=0; i<nWords; i++)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress+addCnt, data32[i]) == HAL_OK)
		{
			addCnt += 4;
		}
		else
		{
			status = HAL_FLASH_GetError();
		}
	}

	//HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();

	return status;
}

void readStringData_InternalFlash(uint32_t flashAddress, char *pdata)
{
	volatile uint32_t read_data;
	volatile uint32_t read_cnt = 0;
	do
	{
		read_data = *(uint32_t*)(flashAddress + read_cnt);
		if(read_data != 0xFFFFFFFF)
		{
			pdata[read_cnt] = (uint8_t)read_data;
			pdata[read_cnt + 1] = (uint8_t)(read_data >> 8);
			pdata[read_cnt + 2] = (uint8_t)(read_data >> 16);
			pdata[read_cnt + 3] = (uint8_t)(read_data >> 24);
			read_cnt += 4;
		}
	} while (read_data != 0xFFFFFFFF);
}


