/*
 * HelperFcns.c
 *
 *  Created on: Dec 13, 2020
 *      Author: dlago
 */

#include "HelperFcns.h"

void clearCharBuffer(char* buffer,int size)
{
	int i=0;
	for(i=0;i<size;i++)
	{
	    buffer[i] = 0;
	}
}
uint8_t modifyBits(uint8_t oldValue, uint8_t LSBitPos, uint8_t bitsNewVal)
{
	uint8_t mask;
	if ((bitsNewVal == 0)||(bitsNewVal == 1))
	{
		mask = (1<<LSBitPos);
	}
	else
	{
		mask = (int)pow(2,floor(log2(bitsNewVal))+1)<<LSBitPos;
	}
    return ((oldValue & ~mask) | (bitsNewVal << LSBitPos));
}
void float2uint8(float fData, uint8_t *pdataV, uint8_t nDecimals)
{
    int32_t dec[] = { 1, 10, 100, 1000, 10000, 100000, 1000000 };
    float roundfData = roundf(fData * dec[nDecimals]) / dec[nDecimals];
    int32_t dataInt = roundfData * dec[nDecimals];
    pdataV[0] = dataInt >> 24;
    pdataV[1] = dataInt >> 16;
    pdataV[2] = dataInt >> 8;
    pdataV[3] = dataInt;
    //Reinterpret the data: float val = (float)(int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3])/roundF[nDecimals];
}
