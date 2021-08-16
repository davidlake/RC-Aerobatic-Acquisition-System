
#include "w25qxx.h"

#if (_W25QXX_DEBUG==1)
#include <stdio.h>
#endif

#define W25QXX_DUMMY_BYTE         0xA5
#define _W25QXX_USE_FREERTOS          0
#define _W25QXX_DEBUG                 0

#if (_W25QXX_USE_FREERTOS==1)
#define	W25qxx_Delay(delay)		osDelay(delay)
#include "cmsis_os.h"
#else
#define	W25qxx_Delay(delay)		HAL_Delay(delay)
#endif

//Private functions
uint8_t	W25qxx_Spi(W25Qxx_DEVICE *pdev, uint8_t	Data)
{
	uint8_t	ret;
	HAL_SPI_TransmitReceive(pdev->serif.SPI_HANDLE,&Data,&ret,1,100);
	return ret;	
}
uint32_t W25qxx_ReadID(W25Qxx_DEVICE *pdev)
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
  HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
  W25qxx_Spi(pdev, 0x9F);
  Temp0 = W25qxx_Spi(pdev, W25QXX_DUMMY_BYTE);
  Temp1 = W25qxx_Spi(pdev, W25QXX_DUMMY_BYTE);
  Temp2 = W25qxx_Spi(pdev, W25QXX_DUMMY_BYTE);
  HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
  return Temp;
}
void W25qxx_ReadUniqID(W25Qxx_DEVICE *pdev)
{
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x4B);
	for(uint8_t	i=0;i<4;i++)
		W25qxx_Spi(pdev, W25QXX_DUMMY_BYTE);
	for(uint8_t	i=0;i<8;i++)
		pdev->UniqID[i] = W25qxx_Spi(pdev, W25QXX_DUMMY_BYTE);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
}
void W25qxx_WriteEnable(W25Qxx_DEVICE *pdev)
{
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x06);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	W25qxx_Delay(1);
}
void W25qxx_WriteDisable(W25Qxx_DEVICE *pdev)
{
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x04);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	W25qxx_Delay(1);
}
uint8_t W25qxx_ReadStatusRegister(W25Qxx_DEVICE *pdev, uint8_t SelectStatusRegister_1_2_3)
{
	uint8_t	status=0;
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	if(SelectStatusRegister_1_2_3==1)
	{
		W25qxx_Spi(pdev, 0x05);
		status=W25qxx_Spi(pdev, W25QXX_DUMMY_BYTE);
		pdev->StatusRegister1 = status;
	}
	else if(SelectStatusRegister_1_2_3==2)
	{
		W25qxx_Spi(pdev, 0x35);
		status=W25qxx_Spi(pdev, W25QXX_DUMMY_BYTE);
		pdev->StatusRegister2 = status;
	}
	else
	{
		W25qxx_Spi(pdev, 0x15);
		status=W25qxx_Spi(pdev, W25QXX_DUMMY_BYTE);
		pdev->StatusRegister3 = status;
	}	
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	return status;
}
void W25qxx_WriteStatusRegister(W25Qxx_DEVICE *pdev, uint8_t SelectStatusRegister_1_2_3,uint8_t Data)
{
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	if(SelectStatusRegister_1_2_3==1)
	{
		W25qxx_Spi(pdev, 0x01);
		pdev->StatusRegister1 = Data;
	}
	else if(SelectStatusRegister_1_2_3==2)
	{
		W25qxx_Spi(pdev, 0x31);
		pdev->StatusRegister2 = Data;
	}
	else
	{
		W25qxx_Spi(pdev, 0x11);
		pdev->StatusRegister3 = Data;
	}
	W25qxx_Spi(pdev, Data);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
}
void W25qxx_WaitForWriteEnd(W25Qxx_DEVICE *pdev)
{
	//W25qxx_Delay(1);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x05);
	do
	{
		pdev->StatusRegister1 = W25qxx_Spi(pdev, W25QXX_DUMMY_BYTE);
		//W25qxx_Delay(1);
	}
	while ((pdev->StatusRegister1 & 0x01) == 0x01);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
}

//Public functions
bool W25qxx_Init_Device(W25Qxx_DEVICE *pdev, SPI_HandleTypeDef *SPI_HANDLE, GPIO_TypeDef *CS_PIN_PORT, uint16_t CS_PIN_NUMBER)
{
	pdev->serif.SPI_HANDLE = SPI_HANDLE;
	pdev->serif.CS_PIN_PORT = CS_PIN_PORT;
	pdev->serif.CS_PIN_NUMBER = CS_PIN_NUMBER;
	pdev->Lock=1;
	while(HAL_GetTick()<100)
	W25qxx_Delay(1);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	W25qxx_Delay(100);
	uint32_t	id;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx Init Begin...\r\n");
	#endif
	id=W25qxx_ReadID(pdev);
	
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx ID:0x%X\r\n",id);
	#endif
	switch(id&0x0000FFFF)
	{
		case 0x401A:	// 	w25q512
			pdev->ID=W25Q512;
			pdev->BlockCount=1024;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q512\r\n");
			#endif
		break;
		case 0x4019:	// 	w25q256
			pdev->ID=W25Q256;
			pdev->BlockCount=512;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q256\r\n");
			#endif
		break;
		case 0x4018:	// 	w25q128
			pdev->ID=W25Q128;
			pdev->BlockCount=256;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q128\r\n");
			#endif
		break;
		case 0x4017:	//	w25q64
			pdev->ID=W25Q64;
			pdev->BlockCount=128;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q64\r\n");
			#endif
		break;
		case 0x4016:	//	w25q32
			pdev->ID=W25Q32;
			pdev->BlockCount=64;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q32\r\n");
			#endif
		break;
		case 0x4015:	//	w25q16
			pdev->ID=W25Q16;
			pdev->BlockCount=32;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q16\r\n");
			#endif
		break;
		case 0x4014:	//	w25q80
			pdev->ID=W25Q80;
			pdev->BlockCount=16;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q80\r\n");
			#endif
		break;
		case 0x4013:	//	w25q40
			pdev->ID=W25Q40;
			pdev->BlockCount=8;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q40\r\n");
			#endif
		break;
		case 0x4012:	//	w25q20
			pdev->ID=W25Q20;
			pdev->BlockCount=4;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q20\r\n");
			#endif
		break;
		case 0x4011:	//	w25q10
			pdev->ID=W25Q10;
			pdev->BlockCount=2;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q10\r\n");
			#endif
		break;
		default:
				#if (_W25QXX_DEBUG==1)
				printf("w25qxx Unknown ID\r\n");
				#endif
				pdev->Lock=0;
			return false;
				
	}		
	pdev->PageSize=256;
	pdev->SectorSize=0x1000;
	pdev->SectorCount=pdev->BlockCount*16;
	pdev->PageCount=(pdev->SectorCount*pdev->SectorSize)/pdev->PageSize;
	pdev->BlockSize=pdev->SectorSize*16;
	pdev->CapacityInKiloByte=(pdev->SectorCount*pdev->SectorSize)/1024;
	W25qxx_ReadUniqID(pdev);
	W25qxx_ReadStatusRegister(pdev, 1);
	W25qxx_ReadStatusRegister(pdev, 2);
	W25qxx_ReadStatusRegister(pdev, 3);
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx Page Size: %d Bytes\r\n",w25qxx.PageSize);
	printf("w25qxx Page Count: %d\r\n",w25qxx.PageCount);
	printf("w25qxx Sector Size: %d Bytes\r\n",w25qxx.SectorSize);
	printf("w25qxx Sector Count: %d\r\n",w25qxx.SectorCount);
	printf("w25qxx Block Size: %d Bytes\r\n",w25qxx.BlockSize);
	printf("w25qxx Block Count: %d\r\n",w25qxx.BlockCount);
	printf("w25qxx Capacity: %d KiloBytes\r\n",w25qxx.CapacityInKiloByte);
	printf("w25qxx Init Done\r\n");
	#endif
	pdev->Lock=0;
	W25qxx_IsErased(pdev); // check if there is data available on the SPI flash memory
	return true;
}	
void W25qxx_EraseChip(W25Qxx_DEVICE *pdev)
{
	while(pdev->Lock==1)
		W25qxx_Delay(1);
	pdev->Lock=1;
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();	
	printf("w25qxx EraseChip Begin...\r\n");
	#endif
	W25qxx_WriteEnable(pdev);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0xC7);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	W25qxx_WaitForWriteEnd(pdev);
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx EraseBlock done after %d ms!\r\n",HAL_GetTick()-StartTime);
	#endif
	W25qxx_Delay(10);
	pdev->Lock=0;
}
void W25qxx_Erase4KSector(W25Qxx_DEVICE *pdev, uint32_t SectorAddr)
{
	while(pdev->Lock==1)
		W25qxx_Delay(1);
	pdev->Lock=1;
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();	
	printf("w25qxx EraseSector %d Begin...\r\n",SectorAddr);
	#endif
	W25qxx_WaitForWriteEnd(pdev);
	SectorAddr = SectorAddr * pdev->SectorSize;
	W25qxx_WriteEnable(pdev);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x20);
	if(pdev->ID>=W25Q256)
		W25qxx_Spi(pdev, (SectorAddr & 0xFF000000) >> 24);
	W25qxx_Spi(pdev, (SectorAddr & 0xFF0000) >> 16);
	W25qxx_Spi(pdev, (SectorAddr & 0xFF00) >> 8);
	W25qxx_Spi(pdev, SectorAddr & 0xFF);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	W25qxx_WaitForWriteEnd(pdev);
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx EraseSector done after %d ms\r\n",HAL_GetTick()-StartTime);
	#endif
	W25qxx_Delay(1);
	pdev->Lock=0;
}
void W25qxx_Erase32KBlock(W25Qxx_DEVICE *pdev, uint32_t BlockAddr)
{
	while(pdev->Lock==1)
		W25qxx_Delay(1);
	pdev->Lock=1;
	W25qxx_WaitForWriteEnd(pdev);
	BlockAddr = BlockAddr * pdev->SectorSize*8;
	W25qxx_WriteEnable(pdev);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x52);
	if(pdev->ID>=W25Q256)
		W25qxx_Spi(pdev, (BlockAddr & 0xFF000000) >> 24);
	W25qxx_Spi(pdev, (BlockAddr & 0xFF0000) >> 16);
	W25qxx_Spi(pdev, (BlockAddr & 0xFF00) >> 8);
	W25qxx_Spi(pdev, BlockAddr & 0xFF);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	W25qxx_WaitForWriteEnd(pdev);
	W25qxx_Delay(1);
	pdev->Lock=0;
}
void W25qxx_Erase64KBlock(W25Qxx_DEVICE *pdev, uint32_t BlockAddr)
{
	while(pdev->Lock==1)
		W25qxx_Delay(1);
	pdev->Lock=1;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx EraseBlock %d Begin...\r\n",BlockAddr);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();	
	#endif
	W25qxx_WaitForWriteEnd(pdev);
	BlockAddr = BlockAddr * pdev->SectorSize*16;
	W25qxx_WriteEnable(pdev);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0xD8);
	if(pdev->ID>=W25Q256)
		W25qxx_Spi(pdev, (BlockAddr & 0xFF000000) >> 24);
	W25qxx_Spi(pdev, (BlockAddr & 0xFF0000) >> 16);
	W25qxx_Spi(pdev, (BlockAddr & 0xFF00) >> 8);
	W25qxx_Spi(pdev, BlockAddr & 0xFF);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	W25qxx_WaitForWriteEnd(pdev);
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx EraseBlock done after %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif
	W25qxx_Delay(1);
	pdev->Lock=0;
}
uint32_t W25qxx_PageToSector(W25Qxx_DEVICE *pdev, uint32_t PageAddress)
{
	return ((PageAddress*pdev->PageSize)/pdev->SectorSize);
}
uint32_t W25qxx_PageToBlock(W25Qxx_DEVICE *pdev, uint32_t PageAddress)
{
	return ((PageAddress*pdev->PageSize)/pdev->BlockSize);
}
uint32_t W25qxx_SectorToBlock(W25Qxx_DEVICE *pdev, uint32_t SectorAddress)
{
	return ((SectorAddress*pdev->SectorSize)/pdev->BlockSize);
}
uint32_t W25qxx_SectorToPage(W25Qxx_DEVICE *pdev, uint32_t SectorAddress)
{
	return (SectorAddress*pdev->SectorSize)/pdev->PageSize;
}
uint32_t W25qxx_BlockToPage(W25Qxx_DEVICE *pdev, uint32_t BlockAddress)
{
	return (BlockAddress*pdev->BlockSize)/pdev->PageSize;
}
bool W25qxx_IsEmptyPage(W25Qxx_DEVICE *pdev, uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_PageSize)
{
	while(pdev->Lock==1)
	W25qxx_Delay(1);
	pdev->Lock=1;
	if(((NumByteToCheck_up_to_PageSize+OffsetInByte)>pdev->PageSize)||(NumByteToCheck_up_to_PageSize==0))
		NumByteToCheck_up_to_PageSize=pdev->PageSize-OffsetInByte;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckPage:%d, Offset:%d, Bytes:%d begin...\r\n",Page_Address,OffsetInByte,NumByteToCheck_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();
	#endif		
	uint8_t	pBuffer[32];
	uint32_t	WorkAddress;
	uint32_t	i;
	for(i=OffsetInByte; i<pdev->PageSize; i+=sizeof(pBuffer))
	{
		HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
		WorkAddress=(i+Page_Address*pdev->PageSize);
		W25qxx_Spi(pdev, 0x0B);
		if(pdev->ID>=W25Q256)
			W25qxx_Spi(pdev, (WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi(pdev, (WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi(pdev, (WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(pdev, WorkAddress & 0xFF);
		W25qxx_Spi(pdev, 0);
		HAL_SPI_Receive(pdev->serif.SPI_HANDLE,pBuffer,sizeof(pBuffer),100);
		HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
		for(uint8_t x=0;x<sizeof(pBuffer);x++)
		{
			if(pBuffer[x]!=0xFF)
				goto NOT_EMPTY;		
		}			
	}	
	if((pdev->PageSize+OffsetInByte)%sizeof(pBuffer)!=0)
	{
		i-=sizeof(pBuffer);
		for( ; i<pdev->PageSize; i++)
		{
			HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
			WorkAddress=(i+Page_Address*pdev->PageSize);
			W25qxx_Spi(pdev, 0x0B);
			if(pdev->ID>=W25Q256)
				W25qxx_Spi(pdev, (WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi(pdev, (WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi(pdev, (WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(pdev, WorkAddress & 0xFF);
			W25qxx_Spi(pdev, 0);
			HAL_SPI_Receive(pdev->serif.SPI_HANDLE,pBuffer,1,100);
			HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
			if(pBuffer[0]!=0xFF)
				goto NOT_EMPTY;
		}
	}	
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckPage is Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	pdev->Lock=0;
	return true;	
	NOT_EMPTY:
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckPage is Not Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	pdev->Lock=0;
	return false;
}
bool W25qxx_IsEmptySector(W25Qxx_DEVICE *pdev, uint32_t Sector_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_SectorSize)
{
	while(pdev->Lock==1)
	W25qxx_Delay(1);
	pdev->Lock=1;
	if((NumByteToCheck_up_to_SectorSize>pdev->SectorSize)||(NumByteToCheck_up_to_SectorSize==0))
		NumByteToCheck_up_to_SectorSize=pdev->SectorSize;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckSector:%d, Offset:%d, Bytes:%d begin...\r\n",Sector_Address,OffsetInByte,NumByteToCheck_up_to_SectorSize);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();
	#endif		
	uint8_t	pBuffer[32];
	uint32_t WorkAddress;
	uint32_t i;
	for(i=OffsetInByte; i<pdev->SectorSize; i+=sizeof(pBuffer))
	{
		HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
		WorkAddress=(i+Sector_Address*pdev->SectorSize);
		W25qxx_Spi(pdev, 0x0B);
		if(pdev->ID>=W25Q256)
			W25qxx_Spi(pdev, (WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi(pdev, (WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi(pdev, (WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(pdev, WorkAddress & 0xFF);
		W25qxx_Spi(pdev, 0);
		HAL_SPI_Receive(pdev->serif.SPI_HANDLE,pBuffer,sizeof(pBuffer),100);
		HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
		for(uint8_t x=0;x<sizeof(pBuffer);x++)
		{
			if(pBuffer[x]!=0xFF)
				goto NOT_EMPTY;		
		}			
	}	
	if((pdev->SectorSize+OffsetInByte)%sizeof(pBuffer)!=0)
	{
		i-=sizeof(pBuffer);
		for( ; i<pdev->SectorSize; i++)
		{
			HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
			WorkAddress=(i+Sector_Address*pdev->SectorSize);
			W25qxx_Spi(pdev, 0x0B);
			if(pdev->ID>=W25Q256)
				W25qxx_Spi(pdev, (WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi(pdev, (WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi(pdev, (WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(pdev, WorkAddress & 0xFF);
			W25qxx_Spi(pdev, 0);
			HAL_SPI_Receive(pdev->serif.SPI_HANDLE,pBuffer,1,100);
			HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
			if(pBuffer[0]!=0xFF)
				goto NOT_EMPTY;
		}
	}	
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckSector is Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	pdev->Lock=0;
	return true;	
	NOT_EMPTY:
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckSector is Not Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	pdev->Lock=0;
	return false;
}
bool W25qxx_IsEmptyBlock(W25Qxx_DEVICE *pdev, uint32_t Block_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_BlockSize)
{
	while(pdev->Lock==1)
	W25qxx_Delay(1);
	pdev->Lock=1;
	if((NumByteToCheck_up_to_BlockSize>pdev->BlockSize)||(NumByteToCheck_up_to_BlockSize==0))
		NumByteToCheck_up_to_BlockSize=pdev->BlockSize;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckBlock:%d, Offset:%d, Bytes:%d begin...\r\n",Block_Address,OffsetInByte,NumByteToCheck_up_to_BlockSize);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();
	#endif		
	uint8_t	pBuffer[32];
	uint32_t	WorkAddress;
	uint32_t	i;
	for(i=OffsetInByte; i<pdev->BlockSize; i+=sizeof(pBuffer))
	{
		HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
		WorkAddress=(i+Block_Address*pdev->BlockSize);
		W25qxx_Spi(pdev, 0x0B);
		if(pdev->ID>=W25Q256)
			W25qxx_Spi(pdev, (WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi(pdev, (WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi(pdev, (WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(pdev, WorkAddress & 0xFF);
		W25qxx_Spi(pdev, 0);
		HAL_SPI_Receive(pdev->serif.SPI_HANDLE,pBuffer,sizeof(pBuffer),100);
		HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
		for(uint8_t x=0;x<sizeof(pBuffer);x++)
		{
			if(pBuffer[x]!=0xFF)
				goto NOT_EMPTY;		
		}			
	}	
	if((pdev->BlockSize+OffsetInByte)%sizeof(pBuffer)!=0)
	{
		i-=sizeof(pBuffer);
		for( ; i<pdev->BlockSize; i++)
		{
			HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
			WorkAddress=(i+Block_Address*pdev->BlockSize);
			W25qxx_Spi(pdev, 0x0B);
			if(pdev->ID>=W25Q256)
				W25qxx_Spi(pdev, (WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi(pdev, (WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi(pdev, (WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(pdev, WorkAddress & 0xFF);
			W25qxx_Spi(pdev, 0);
			HAL_SPI_Receive(pdev->serif.SPI_HANDLE,pBuffer,1,100);
			HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
			if(pBuffer[0]!=0xFF)
				goto NOT_EMPTY;
		}
	}	
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckBlock is Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	pdev->Lock=0;
	return true;	
	NOT_EMPTY:
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckBlock is Not Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	pdev->Lock=0;
	return false;
}
void W25qxx_WriteByte(W25Qxx_DEVICE *pdev, uint8_t Buffer, uint32_t WriteAddr_inBytes)
{
	while(pdev->Lock==1)
		W25qxx_Delay(1);
	pdev->Lock=1;
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();
	printf("w25qxx WriteByte 0x%02X at address %d begin...",pBuffer,WriteAddr_inBytes);
	#endif
	W25qxx_WaitForWriteEnd(pdev);
	W25qxx_WriteEnable(pdev);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x02);
	if(pdev->ID>=W25Q256)
		W25qxx_Spi(pdev, (WriteAddr_inBytes & 0xFF000000) >> 24);
	W25qxx_Spi(pdev, (WriteAddr_inBytes & 0xFF0000) >> 16);
	W25qxx_Spi(pdev, (WriteAddr_inBytes & 0xFF00) >> 8);
	W25qxx_Spi(pdev, WriteAddr_inBytes & 0xFF);
	W25qxx_Spi(pdev, Buffer);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	W25qxx_WaitForWriteEnd(pdev);
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx WriteByte done after %d ms\r\n",HAL_GetTick()-StartTime);
	#endif
	pdev->Lock=0;
}
void W25qxx_WritePage(W25Qxx_DEVICE *pdev, uint8_t *pBuffer	,uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToWrite_up_to_PageSize)
{
	while(pdev->Lock==1)
		W25qxx_Delay(1);
	pdev->Lock=1;
	if(((NumByteToWrite_up_to_PageSize+OffsetInByte)>pdev->PageSize)||(NumByteToWrite_up_to_PageSize==0))
		NumByteToWrite_up_to_PageSize=pdev->PageSize-OffsetInByte;
	if((OffsetInByte+NumByteToWrite_up_to_PageSize) > pdev->PageSize)
		NumByteToWrite_up_to_PageSize = pdev->PageSize-OffsetInByte;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx WritePage:%d, Offset:%d ,Writes %d Bytes, begin...\r\n",Page_Address,OffsetInByte,NumByteToWrite_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();
	#endif	
	W25qxx_WaitForWriteEnd(pdev);
	W25qxx_WriteEnable(pdev);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x02);
	Page_Address = (Page_Address*pdev->PageSize)+OffsetInByte;
	if(pdev->ID>=W25Q256)
		W25qxx_Spi(pdev, (Page_Address & 0xFF000000) >> 24);
	W25qxx_Spi(pdev, (Page_Address & 0xFF0000) >> 16);
	W25qxx_Spi(pdev, (Page_Address & 0xFF00) >> 8);
	W25qxx_Spi(pdev, Page_Address&0xFF);
	HAL_SPI_Transmit(pdev->serif.SPI_HANDLE,pBuffer,NumByteToWrite_up_to_PageSize,100);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	W25qxx_WaitForWriteEnd(pdev);
	#if (_W25QXX_DEBUG==1)
	StartTime = HAL_GetTick()-StartTime; 
	for(uint32_t i=0;i<NumByteToWrite_up_to_PageSize ; i++)
	{
		if((i%8==0)&&(i>2))
		{
			printf("\r\n");
			W25qxx_Delay(10);			
		}
		printf("0x%02X,",pBuffer[i]);		
	}	
	printf("\r\n");
	printf("w25qxx WritePage done after %d ms\r\n",StartTime);
	W25qxx_Delay(100);
	#endif	
	W25qxx_Delay(1);
	pdev->Lock=0;
}
void W25qxx_WriteSector(W25Qxx_DEVICE *pdev, uint8_t *pBuffer	,uint32_t Sector_Address,uint32_t OffsetInByte	,uint32_t NumByteToWrite_up_to_SectorSize)
{
	if((NumByteToWrite_up_to_SectorSize>pdev->SectorSize)||(NumByteToWrite_up_to_SectorSize==0))
		NumByteToWrite_up_to_SectorSize=pdev->SectorSize;
	#if (_W25QXX_DEBUG==1)
	printf("+++w25qxx WriteSector:%d, Offset:%d ,Write %d Bytes, begin...\r\n",Sector_Address,OffsetInByte,NumByteToWrite_up_to_SectorSize);
	W25qxx_Delay(100);
	#endif	
	if(OffsetInByte>=pdev->SectorSize)
	{
		#if (_W25QXX_DEBUG==1)
		printf("---w25qxx WriteSector Faild!\r\n");
		W25qxx_Delay(100);
		#endif	
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToWrite;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToWrite_up_to_SectorSize) > pdev->SectorSize)
		BytesToWrite = pdev->SectorSize-OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_SectorSize;	
	StartPage = W25qxx_SectorToPage(pdev, Sector_Address)+(OffsetInByte/pdev->PageSize);
	LocalOffset = OffsetInByte%pdev->PageSize;
	do
	{		
		W25qxx_WritePage(pdev, pBuffer,StartPage,LocalOffset,BytesToWrite);
		StartPage++;
		BytesToWrite-=pdev->PageSize-LocalOffset;
		pBuffer += pdev->PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToWrite>0);		
	#if (_W25QXX_DEBUG==1)
	printf("---w25qxx WriteSector Done\r\n");
	W25qxx_Delay(100);
	#endif	
}
void W25qxx_WriteBlock(W25Qxx_DEVICE *pdev, uint8_t* pBuffer ,uint32_t Block_Address	,uint32_t OffsetInByte	,uint32_t	NumByteToWrite_up_to_BlockSize)
{
	if((NumByteToWrite_up_to_BlockSize>pdev->BlockSize)||(NumByteToWrite_up_to_BlockSize==0))
		NumByteToWrite_up_to_BlockSize=pdev->BlockSize;
	#if (_W25QXX_DEBUG==1)
	printf("+++w25qxx WriteBlock:%d, Offset:%d ,Write %d Bytes, begin...\r\n",Block_Address,OffsetInByte,NumByteToWrite_up_to_BlockSize);
	W25qxx_Delay(100);
	#endif	
	if(OffsetInByte>=pdev->BlockSize)
	{
		#if (_W25QXX_DEBUG==1)
		printf("---w25qxx WriteBlock Faild!\r\n");
		W25qxx_Delay(100);
		#endif	
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToWrite;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToWrite_up_to_BlockSize) > pdev->BlockSize)
		BytesToWrite = pdev->BlockSize-OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_BlockSize;	
	StartPage = W25qxx_BlockToPage(pdev, Block_Address)+(OffsetInByte/pdev->PageSize);
	LocalOffset = OffsetInByte%pdev->PageSize;
	do
	{		
		W25qxx_WritePage(pdev, pBuffer,StartPage,LocalOffset,BytesToWrite);
		StartPage++;
		BytesToWrite-=pdev->PageSize-LocalOffset;
		pBuffer += pdev->PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToWrite>0);		
	#if (_W25QXX_DEBUG==1)
	printf("---w25qxx WriteBlock Done\r\n");
	W25qxx_Delay(100);
	#endif	
}
void W25qxx_ReadByte(W25Qxx_DEVICE *pdev, uint8_t *pBuffer,uint32_t Bytes_Address)
{
	while(pdev->Lock==1)
		W25qxx_Delay(1);
	pdev->Lock=1;
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();
	printf("w25qxx ReadByte at address %d begin...\r\n",Bytes_Address);
	#endif
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x0B);
	if(pdev->ID>=W25Q256)
		W25qxx_Spi(pdev, (Bytes_Address & 0xFF000000) >> 24);
	W25qxx_Spi(pdev, (Bytes_Address & 0xFF0000) >> 16);
	W25qxx_Spi(pdev, (Bytes_Address& 0xFF00) >> 8);
	W25qxx_Spi(pdev, Bytes_Address & 0xFF);
	W25qxx_Spi(pdev, 0);
	*pBuffer = W25qxx_Spi(pdev, W25QXX_DUMMY_BYTE);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx ReadByte 0x%02X done after %d ms\r\n",*pBuffer,HAL_GetTick()-StartTime);
	#endif
	pdev->Lock=0;
}
void W25qxx_ReadBytes(W25Qxx_DEVICE *pdev, uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	while(pdev->Lock==1)
		W25qxx_Delay(1);
	pdev->Lock=1;
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();
	printf("w25qxx ReadBytes at Address:%d, %d Bytes  begin...\r\n",ReadAddr,NumByteToRead);
	#endif	
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x0B);
	if(pdev->ID>=W25Q256)
		W25qxx_Spi(pdev, (ReadAddr & 0xFF000000) >> 24);
	W25qxx_Spi(pdev, (ReadAddr & 0xFF0000) >> 16);
	W25qxx_Spi(pdev, (ReadAddr& 0xFF00) >> 8);
	W25qxx_Spi(pdev, ReadAddr & 0xFF);
	W25qxx_Spi(pdev, 0);
	HAL_SPI_Receive(pdev->serif.SPI_HANDLE,pBuffer,NumByteToRead,2000);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	#if (_W25QXX_DEBUG==1)
	StartTime = HAL_GetTick()-StartTime; 
	for(uint32_t i=0;i<NumByteToRead ; i++)
	{
		if((i%8==0)&&(i>2))
		{
			printf("\r\n");
			W25qxx_Delay(10);
		}
		printf("0x%02X,",pBuffer[i]);		
	}
	printf("\r\n");
	printf("w25qxx ReadBytes done after %d ms\r\n",StartTime);
	W25qxx_Delay(100);
	#endif	
	W25qxx_Delay(1);
	pdev->Lock=0;
}
void W25qxx_ReadPage(W25Qxx_DEVICE *pdev, uint8_t *pBuffer,uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToRead_up_to_PageSize)
{
	while(pdev->Lock==1)
		W25qxx_Delay(1);
	pdev->Lock=1;
	if((NumByteToRead_up_to_PageSize>pdev->PageSize)||(NumByteToRead_up_to_PageSize==0))
		NumByteToRead_up_to_PageSize=pdev->PageSize;
	if((OffsetInByte+NumByteToRead_up_to_PageSize) > pdev->PageSize)
		NumByteToRead_up_to_PageSize = pdev->PageSize-OffsetInByte;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx ReadPage:%d, Offset:%d ,Read %d Bytes, begin...\r\n",Page_Address,OffsetInByte,NumByteToRead_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();
	#endif	
	Page_Address = Page_Address*pdev->PageSize+OffsetInByte;
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x0B);
	if(pdev->ID>=W25Q256)
		W25qxx_Spi(pdev, (Page_Address & 0xFF000000) >> 24);
	W25qxx_Spi(pdev, (Page_Address & 0xFF0000) >> 16);
	W25qxx_Spi(pdev, (Page_Address& 0xFF00) >> 8);
	W25qxx_Spi(pdev, Page_Address & 0xFF);
	W25qxx_Spi(pdev, 0);
	HAL_SPI_Receive(pdev->serif.SPI_HANDLE,pBuffer,NumByteToRead_up_to_PageSize,100);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	#if (_W25QXX_DEBUG==1)
	StartTime = HAL_GetTick()-StartTime; 
	for(uint32_t i=0;i<NumByteToRead_up_to_PageSize ; i++)
	{
		if((i%8==0)&&(i>2))
		{
			printf("\r\n");
			W25qxx_Delay(10);
		}
		printf("0x%02X,",pBuffer[i]);		
	}	
	printf("\r\n");
	printf("w25qxx ReadPage done after %d ms\r\n",StartTime);
	W25qxx_Delay(100);
	#endif	
	W25qxx_Delay(1);
	pdev->Lock=0;
}
void W25qxx_ReadSector(W25Qxx_DEVICE *pdev, uint8_t *pBuffer,uint32_t Sector_Address,uint32_t OffsetInByte,uint32_t NumByteToRead_up_to_SectorSize)
{	
	if((NumByteToRead_up_to_SectorSize>pdev->SectorSize)||(NumByteToRead_up_to_SectorSize==0))
		NumByteToRead_up_to_SectorSize=pdev->SectorSize;
	#if (_W25QXX_DEBUG==1)
	printf("+++w25qxx ReadSector:%d, Offset:%d ,Read %d Bytes, begin...\r\n",Sector_Address,OffsetInByte,NumByteToRead_up_to_SectorSize);
	W25qxx_Delay(100);
	#endif	
	if(OffsetInByte>=pdev->SectorSize)
	{
		#if (_W25QXX_DEBUG==1)
		printf("---w25qxx ReadSector Faild!\r\n");
		W25qxx_Delay(100);
		#endif	
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToRead;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToRead_up_to_SectorSize) > pdev->SectorSize)
		BytesToRead = pdev->SectorSize-OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_SectorSize;	
	StartPage = W25qxx_SectorToPage(pdev, Sector_Address)+(OffsetInByte/pdev->PageSize);
	LocalOffset = OffsetInByte%pdev->PageSize;
	do
	{		
		W25qxx_ReadPage(pdev, pBuffer,StartPage,LocalOffset,BytesToRead);
		StartPage++;
		BytesToRead-=pdev->PageSize-LocalOffset;
		pBuffer += pdev->PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToRead>0);		
	#if (_W25QXX_DEBUG==1)
	printf("---w25qxx ReadSector Done\r\n");
	W25qxx_Delay(100);
	#endif	
}
void W25qxx_ReadBlock(W25Qxx_DEVICE *pdev, uint8_t* pBuffer,uint32_t Block_Address,uint32_t OffsetInByte,uint32_t	NumByteToRead_up_to_BlockSize)
{
	if((NumByteToRead_up_to_BlockSize>pdev->BlockSize)||(NumByteToRead_up_to_BlockSize==0))
		NumByteToRead_up_to_BlockSize=pdev->BlockSize;
	#if (_W25QXX_DEBUG==1)
	printf("+++w25qxx ReadBlock:%d, Offset:%d ,Read %d Bytes, begin...\r\n",Block_Address,OffsetInByte,NumByteToRead_up_to_BlockSize);
	W25qxx_Delay(100);
	#endif	
	if(OffsetInByte>=pdev->BlockSize)
	{
		#if (_W25QXX_DEBUG==1)
		printf("w25qxx ReadBlock Faild!\r\n");
		W25qxx_Delay(100);
		#endif	
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToRead;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToRead_up_to_BlockSize) > pdev->BlockSize)
		BytesToRead = pdev->BlockSize-OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_BlockSize;	
	StartPage = W25qxx_BlockToPage(pdev, Block_Address)+(OffsetInByte/pdev->PageSize);
	LocalOffset = OffsetInByte%pdev->PageSize;
	do
	{		
		W25qxx_ReadPage(pdev, pBuffer,StartPage,LocalOffset,BytesToRead);
		StartPage++;
		BytesToRead-=pdev->PageSize-LocalOffset;
		pBuffer += pdev->PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToRead>0);		
	#if (_W25QXX_DEBUG==1)
	printf("---w25qxx ReadBlock Done\r\n");
	W25qxx_Delay(100);
	#endif	
}

void W25Q64_WriteBytes(W25Qxx_DEVICE *pdev, uint8_t *pBuffer,uint32_t nBytes)
{
	pdev->Lock=1;
	uint32_t Bytes_EndAddress = pdev->currentMemoryAddress + nBytes;
	uint32_t currentPage = pdev->currentMemoryAddress/256;
	uint32_t nextPage = currentPage + 1;
	uint32_t nextPage_BytesAddress = nextPage*256;
	uint32_t nBytes_currentPage;
	uint32_t nBytes_nextPage;

	if (Bytes_EndAddress <= nextPage_BytesAddress)
	{
		nBytes_currentPage = nBytes;
		nBytes_nextPage = 0;
	}
	else
	{
		nBytes_currentPage = nextPage*256-pdev->currentMemoryAddress;
		nBytes_nextPage = nBytes-nBytes_currentPage;
	}

	W25qxx_WaitForWriteEnd(pdev);
	W25qxx_WriteEnable(pdev);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
	W25qxx_Spi(pdev, 0x02); //page program command
	W25qxx_Spi(pdev, (pdev->currentMemoryAddress & 0xFF0000) >> 16);
	W25qxx_Spi(pdev, (pdev->currentMemoryAddress & 0xFF00) >> 8);
	W25qxx_Spi(pdev, pdev->currentMemoryAddress & 0xFF);
	HAL_SPI_Transmit(pdev->serif.SPI_HANDLE,&pBuffer[0],nBytes_currentPage,100);
	HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
	W25qxx_WaitForWriteEnd(pdev);

	if (nBytes_nextPage != 0)
	{
		W25qxx_WriteEnable(pdev);
		HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_RESET);
		W25qxx_Spi(pdev, 0x02);
		W25qxx_Spi(pdev, (nextPage_BytesAddress & 0xFF0000) >> 16);
		W25qxx_Spi(pdev, (nextPage_BytesAddress & 0xFF00) >> 8);
		W25qxx_Spi(pdev, nextPage_BytesAddress & 0xFF);
		HAL_SPI_Transmit(pdev->serif.SPI_HANDLE,&pBuffer[nBytes_currentPage],nBytes_nextPage,100);
		HAL_GPIO_WritePin(pdev->serif.CS_PIN_PORT,pdev->serif.CS_PIN_NUMBER,GPIO_PIN_SET);
		W25qxx_WaitForWriteEnd(pdev);
	}
	pdev->isErased = false;
	pdev->Lock=0;
	pdev->currentMemoryAddress = Bytes_EndAddress;
	pdev->written4KSectorCount = pdev->currentMemoryAddress/pdev->SectorSize;
}
bool W25qxx_IsErased(W25Qxx_DEVICE *pdev)
{
	uint32_t i;
	uint32_t written4KSectorCnt = 0;
	for (i=0; i<pdev->SectorCount; i++)
	{
		if (W25qxx_IsEmptySector(pdev, i, 0, pdev->SectorSize)) //if the sector is empty
		{
			break;
		}
		else
		{
			written4KSectorCnt = i+1;
		}
	}
	pdev->written4KSectorCount = written4KSectorCnt;
	if (written4KSectorCnt == 0)
	{
		pdev->isErased = true;
		pdev->currentMemoryAddress = 0;
	}
	else
	{
		pdev->isErased = false;
	}
	return pdev->isErased;
}
void W25qxx_EraseWrittenMemory(W25Qxx_DEVICE *pdev)
{
	uint32_t i;
	uint32_t written64KBlockCount = pdev->written4KSectorCount/16;
	uint32_t written32KBlockCount_Remainder = (pdev->written4KSectorCount-written64KBlockCount*16)/8;
	uint32_t written4kSectorCount_Remainder = pdev->written4KSectorCount-written32KBlockCount_Remainder*8-written64KBlockCount*16;
	if (written64KBlockCount != 0)
	{
		for (i=0; i<written64KBlockCount; i++)
		{
			W25qxx_Erase64KBlock(pdev, i);
		}
	}
	if (written32KBlockCount_Remainder != 0)
	{
		for (i=0; i<written32KBlockCount_Remainder; i++)
		{
			W25qxx_Erase32KBlock(pdev, i + written64KBlockCount*2);
		}
	}
	if (written4kSectorCount_Remainder != 0)
	{
		for (i=0; i<written4kSectorCount_Remainder; i++)
		{
			W25qxx_Erase4KSector(pdev, i + written64KBlockCount*16 + written32KBlockCount_Remainder*8);
		}
	}
	pdev->isErased = true;
	pdev->currentMemoryAddress = 0;
	pdev->written4KSectorCount = 0;
}
