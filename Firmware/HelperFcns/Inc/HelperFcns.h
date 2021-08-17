/*
 * HelperFcns.h
 *
 *  Created on: Dec 13, 2020
 *      Author: dlago
 */

#ifndef HELPERFCNS_H_
#define HELPERFCNS_H_

#include <stdio.h>
#include <string.h>
#include <math.h>

void clearCharBuffer(char* buffer,int size);
uint8_t modifyBits(uint8_t oldValue, uint8_t LSBitPos, uint8_t bitsNewVal);
//uint8_t oldVal = 104;
//uint8_t newVal = modifyBits(oldVal, 5, 1); //104
//newVal = modifyBits(oldVal, 5, 0); //72
//newVal = modifyBits(oldVal, 4, 1); //120
//newVal = modifyBits(oldVal, 2, 6); //88
void float2uint8(float data, uint8_t *dataV, uint8_t nDecimals);

#endif /* HELPERFCNS_H_ */


//  Ejemplo medir timing codigo
//  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; //enable trace
//  DWT->CYCCNT = 0; //clear DWT cycle counter
//  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//
//  cycles1 = DWT->CYCCNT;
//  navResult = UBLOXM8N_getNav(&navData[0]);
//  cycles2 = DWT->CYCCNT;
//  cyclesDiff = cycles2-cycles1;
