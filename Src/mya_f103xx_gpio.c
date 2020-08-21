/*
 * mya_f103xx_gpio.c
 *
 *  Created on: Aug 15, 2020
 *      Author: muyustan
 */

#include "mya_f103xx_gpio.h"

/* reads the input data of a single pin */
uint8_t gpio_read(GPIO_Type* GPIOx, uint8_t pin_number){

	return (uint8_t)(GPIOx->IDR & (0x01 << pin_number));
}

void gpio_write(GPIO_Type* GPIOx, uint16_t GPIO_PIN_MASK, GPIO_State state){

	if(state == LOW) {
		GPIOx->BSRR = GPIO_PIN_MASK << 16; // reset bits
	}
	else
		GPIOx->BSRR = GPIO_PIN_MASK; // set bits

	return;
}

void gpio_toggle(GPIO_Type* GPIOx, uint16_t GPIO_PIN_MASK){

	uint8_t pos = 0x00;
	uint16_t hold = 0x00;
	uint16_t temp = 0x00;

	for (pos = 0; pos <= 15; pos++) {

		temp = (0x01 << pos);

		if ((temp & GPIO_PIN_MASK) != 0) { // do not use "== 1" instead of "!= 0"
			hold |= temp;
		}

	}

	GPIOx->ODR ^= hold;

	return;
}
