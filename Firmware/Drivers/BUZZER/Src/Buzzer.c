/*
 * Buzzer.c
 *
 *  Created on: May 15, 2021
 *      Author: dlago
 */

#include "Buzzer.h"

void BUZZER_Init(BUZZER_DEVICE *pdev, GPIO_TypeDef *PIN_PORT, uint16_t PIN_NUMBER, TIM_HandleTypeDef *htim)
{
	pdev->PIN_NUMBER = PIN_NUMBER;
	pdev->PIN_PORT = PIN_PORT;
	pdev->htim = htim;
	pdev->state = 0;
	HAL_GPIO_WritePin(pdev->PIN_PORT, pdev->PIN_NUMBER, GPIO_PIN_RESET); //turn off buzzer
}
void BUZZER_ON(BUZZER_DEVICE *pdev)
{
	pdev->state = 1;
	HAL_GPIO_WritePin(pdev->PIN_PORT, pdev->PIN_NUMBER, GPIO_PIN_SET);
}
void BUZZER_OFF(BUZZER_DEVICE *pdev)
{
	pdev->state = 0;
	HAL_GPIO_WritePin(pdev->PIN_PORT, pdev->PIN_NUMBER, GPIO_PIN_RESET);
}
void BUZZER_TurnON_Blck(BUZZER_DEVICE *pdev,uint32_t miliseconds)
{
	BUZZER_ON(pdev);
	HAL_Delay(miliseconds);
	BUZZER_OFF(pdev);
}
void BUZZER_TurnOFF_Blck(BUZZER_DEVICE *pdev,uint32_t miliseconds)
{
	BUZZER_OFF(pdev);
	HAL_Delay(miliseconds);
	BUZZER_ON(pdev);
}
void BUZZER_nBeeps_Blck(BUZZER_DEVICE *pdev, uint8_t nBeeps)
{
	int i;
	for (i=0;i<nBeeps;i++)
	{
		BUZZER_TurnON_Blck(pdev,100);
		HAL_Delay(50);
	}
}
void BUZZER_Beep_Start(BUZZER_DEVICE *pdev, uint16_t miliON, uint16_t miliOFF)
{
	pdev->miliON = miliON;
	pdev->miliOFF= miliOFF;
	pdev->itState = 1;
	HAL_TIM_Base_Start_IT(pdev->htim);
}
void BUZZER_Beep_Stop(BUZZER_DEVICE *pdev)
{
	pdev->itState = 0;
	HAL_TIM_Base_Stop_IT(pdev->htim);
}
