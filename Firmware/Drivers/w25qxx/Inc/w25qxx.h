#ifndef _W25QXX_H
#define _W25QXX_H

/*
  Author:     Nima Askari
  WebSite:    http://www.github.com/NimaLTD
  Instagram:  http://instagram.com/github.NimaLTD
  Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw
  
  Version:    1.1.1
  
  
  Reversion History:
  
  (1.1.1)
  Fix some errors.
  
  (1.1.0)
  Fix some errors.
  
  (1.0.0)
  First release.
*/

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>
#include "main.h"

typedef struct _W25Qxx_HAL_SERIF
 {
 	SPI_HandleTypeDef *SPI_HANDLE;
 	GPIO_TypeDef *CS_PIN_PORT;
 	uint16_t CS_PIN_NUMBER;
 } W25Qxx_HAL_SERIF;
typedef enum _W25QXX_ID_t
{
	W25Q10=1,
	W25Q20,
	W25Q40,
	W25Q80,
	W25Q16,
	W25Q32,
	W25Q64,
	W25Q128,
	W25Q256,
	W25Q512,
}W25QXX_ID_t;
typedef struct _W25Qxx_DEVICE
{
	W25QXX_ID_t	ID; //manufacturer id
	uint8_t		UniqID[8]; //memory type
	uint16_t	PageSize; //page size for this memory
	uint32_t	PageCount; //number of pages on this memory
	uint32_t	SectorSize; //size of one sector of this memory
	uint32_t	SectorCount; //number of available sectors
	uint32_t	BlockSize; //size of one block of this memory
	uint32_t	BlockCount; //number of available blocks
	uint32_t	CapacityInKiloByte; //total capacity calculation
	uint8_t		StatusRegister1;
	uint8_t		StatusRegister2;
	uint8_t		StatusRegister3;	
	uint8_t		Lock; //locks the memory access while erasing or writing the memory
	W25Qxx_HAL_SERIF serif; //serial interface handle
	uint32_t	currentMemoryAddress; //variable that stores current address
	bool 		isErased; //if 0 the memory has some data written, if 1 the memory is empty
	uint32_t 	written4KSectorCount; //number of sectors written
}W25Qxx_DEVICE;

//############################################################################
// in Page,Sector and block read/write functions, can put 0 to read maximum bytes 
//Notes for W25Q64:
//Bytes address ranging from 0 to 8388607
//Page address ranging from 0 to 32767
//############################################################################
bool W25qxx_Init_Device(W25Qxx_DEVICE *pdev, SPI_HandleTypeDef *SPI_HANDLE, GPIO_TypeDef *CS_PIN_PORT, uint16_t CS_PIN_NUMBER);

void W25qxx_EraseChip(W25Qxx_DEVICE *pdev);
void W25qxx_Erase4KSector(W25Qxx_DEVICE *pdev, uint32_t SectorAddr);
void W25qxx_Erase32KBlock(W25Qxx_DEVICE *pdev, uint32_t BlockAddr);
void W25qxx_Erase64KBlock(W25Qxx_DEVICE *pdev, uint32_t BlockAddr);

uint32_t W25qxx_PageToSector(W25Qxx_DEVICE *pdev, uint32_t PageAddress);
uint32_t W25qxx_PageToBlock(W25Qxx_DEVICE *pdev, uint32_t PageAddress);
uint32_t W25qxx_SectorToBlock(W25Qxx_DEVICE *pdev, uint32_t SectorAddress);
uint32_t W25qxx_SectorToPage(W25Qxx_DEVICE *pdev, uint32_t SectorAddress);
uint32_t W25qxx_BlockToPage(W25Qxx_DEVICE *pdev, uint32_t BlockAddress);

bool W25qxx_IsEmptyPage(W25Qxx_DEVICE *pdev, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize);
bool W25qxx_IsEmptySector(W25Qxx_DEVICE *pdev, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize);
bool W25qxx_IsEmptyBlock(W25Qxx_DEVICE *pdev, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize);

void W25qxx_WriteByte(W25Qxx_DEVICE *pdev, uint8_t Buffer, uint32_t Bytes_Address);
void W25qxx_WritePage(W25Qxx_DEVICE *pdev, uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize);
void W25qxx_WriteSector(W25Qxx_DEVICE *pdev, uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize);
void W25qxx_WriteBlock(W25Qxx_DEVICE *pdev, uint8_t* pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize);

void W25qxx_ReadByte(W25Qxx_DEVICE *pdev, uint8_t *pBuffer, uint32_t Bytes_Address);
void W25qxx_ReadBytes(W25Qxx_DEVICE *pdev, uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
void W25qxx_ReadPage(W25Qxx_DEVICE *pdev, uint8_t *pBuffer, uint32_t Page_Address,uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize);
void W25qxx_ReadSector(W25Qxx_DEVICE *pdev, uint8_t *pBuffer, uint32_t Sector_Address,uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize);
void W25qxx_ReadBlock(W25Qxx_DEVICE *pdev, uint8_t* pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize);
//############################################################################

void W25Q64_WriteBytes(W25Qxx_DEVICE *pdev, uint8_t *pBuffer, uint32_t nBytes);
bool W25qxx_IsErased(W25Qxx_DEVICE *pdev);
void W25qxx_EraseWrittenMemory(W25Qxx_DEVICE *pdev);

#ifdef __cplusplus
}
#endif

 /*
 //Example-----------------------------------------------------------------
 W25qxx_Init();
 W25qxx_EraseSector(0);
 uint8_t bufferWrite[224] = {0};
 uint8_t bufferRead[3][256]; //stores data from 3 pages
 uint8_t cnt1;
 uint8_t cnt2;
 flashAddress = 0;
 for (cnt1 = 0; cnt1<3; cnt1++)
 {
 	  for (cnt2 = 0; cnt2<224; cnt2++)
 	  {
 		  bufferWrite[cnt2] = cnt1+1;
 	  }
 	  flashAddress = W25Q64_WriteBytes(&bufferWrite[0],flashAddress,224);
 	  HAL_Delay(15);
 }
 for (cnt1 = 0; cnt1<3; cnt1++)
 {
 	  W25qxx_ReadPage(&bufferRead[cnt1][0],cnt1,0,0);
 }*/

 //Other example

 //  uint32_t i;
 //  for (i=0; i<26; i++)
 //  {
 //	  W25qxx_WriteByte(&W25Q64V_Dev, 0x05, i*4096);
 //  }
 //  W25qxx_IsErased(&W25Q64V_Dev);
 //  W25qxx_EraseWrittenMemory(&W25Q64V_Dev);
 //  W25qxx_IsErased(&W25Q64V_Dev);
 //  W25qxx_EraseSector(&W25Q64V_Dev, 0);
 //  uint8_t bufferWrite[224] = {0};
 //  uint8_t bufferRead[3][256]; //stores data from 3 pages
 //  uint8_t cnt1;
 //  uint8_t cnt2;
 //
 //  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; //enable trace
 //
 //  for (cnt1 = 0; cnt1<3; cnt1++)
 //  {
 //	  for (cnt2 = 0; cnt2<224; cnt2++)
 //	  {
 //		  bufferWrite[cnt2] = cnt1+1;
 //	  }
 //
 //		DWT->CYCCNT = 0; //clear DWT cycle counter
 //		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
 //
 //		cycles1 = DWT->CYCCNT;
 //
 //		W25Q64_WriteBytes(&W25Q64V_Dev, &bufferWrite[0],224);
 //
 //		cycles2 = DWT->CYCCNT;
 //		cyclesDiff = cycles2-cycles1;
 //  }
 //  for (cnt1 = 0; cnt1<3; cnt1++)
 //  {
 //	  W25qxx_ReadPage(&W25Q64V_Dev, &bufferRead[cnt1][0],cnt1,0,0);
 //  }

#endif

