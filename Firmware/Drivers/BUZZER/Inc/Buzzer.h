/*
 * Buzzer.h
 *
 *  Created on: May 15, 2021
 *      Author: dlago
 */

#ifndef BUZZER_INC_BUZZER_H_
#define BUZZER_INC_BUZZER_H_

#include "main.h"

typedef struct _BUZZER_DEVICE
{
	uint8_t state; //0: off; 1: on
	uint8_t itState; //0: interrupt off, 1:interrupt on
	GPIO_TypeDef *PIN_PORT;
	uint16_t PIN_NUMBER;
	TIM_HandleTypeDef *htim;
	uint16_t miliON;
	uint16_t miliOFF;
} BUZZER_DEVICE;

void BUZZER_Init(BUZZER_DEVICE *pdev, GPIO_TypeDef *CS_PIN_PORT, uint16_t CS_PIN_NUMBER, TIM_HandleTypeDef *htim);
void BUZZER_ON(BUZZER_DEVICE *pdev);
void BUZZER_OFF(BUZZER_DEVICE *pdev);
void BUZZER_TurnON_Blck(BUZZER_DEVICE *pdev,uint32_t miliseconds); //blocking routine
void BUZZER_TurnOFF_Blck(BUZZER_DEVICE *pdev,uint32_t miliseconds); //blocking routine
void BUZZER_nBeeps_Blck(BUZZER_DEVICE *pdev, uint8_t nBeeps); //blocking routine
void BUZZER_Beep_Start(BUZZER_DEVICE *pdev, uint16_t miliON, uint16_t miliOFF);
void BUZZER_Beep_Stop(BUZZER_DEVICE *pdev);

#endif /* BUZZER_INC_BUZZER_H_ */
