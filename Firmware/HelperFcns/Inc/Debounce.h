/*
 * Debounce.h
 *
 *  Created on: Dec 22, 2020
 *      Author: dlago
 */

#ifndef INC_DEBOUNCE_H_
#define INC_DEBOUNCE_H_

#include <stdint.h>

void update_button(uint8_t *button_history, uint8_t newVal);
uint8_t is_button_up(uint8_t *button_history);
uint8_t is_button_down(uint8_t *button_history);
uint8_t is_button_pressed(uint8_t *button_history);
uint8_t is_button_released(uint8_t *button_history);

#endif /* INC_DEBOUNCE_H_ */
