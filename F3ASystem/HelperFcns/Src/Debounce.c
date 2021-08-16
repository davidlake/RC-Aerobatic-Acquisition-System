/*
 * Debounce.c
 *
 *  Created on: Dec 22, 2020
 *      Author: dlago
 */

#include "Debounce.h"

void update_button(uint8_t *button_history, uint8_t newVal)
{
    *button_history = *button_history << 1;
    *button_history |= newVal;
}

uint8_t is_button_pressed(uint8_t *button_history)
{
    uint8_t pressed = 0;
    if ((*button_history & 0b11000111) == 0b00000111)
    {
        pressed = 1;
        *button_history = 0b11111111;
    }
    return pressed;
}

uint8_t is_button_released(uint8_t *button_history)
{
	uint8_t released = 0;
	if ((*button_history) == 0b11000000) // corregir esto
	{
		released = 1;
		*button_history = 0b00000000;
	}
	return released;
}

uint8_t is_button_down(uint8_t *button_history)
{
	return (*button_history == 0b11111111);
}

uint8_t is_button_up(uint8_t *button_history)
{
	return (*button_history == 0b00000000);
}

