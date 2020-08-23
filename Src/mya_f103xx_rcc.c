/*
 * mya_f103xx_rcc.c
 *
 *  Created on: Aug 15, 2020
 *      Author: muyustan
 */

#include "mya_f103xx_rcc.h"

/* sets system clock to 8 x PLLMUL */
void clock_config(){

	// FLASH->ACR |= 0x00; // for 0<SYSCLK<24 (MHz)
	// FLASH->ACR |= 0x01; // for 24<SYSCLK<48 (MHz)
	FLASH->ACR |= 0x02; // for 48<SYSCLK<72 (MHz)


	RCC->CR |= RCC_CR_HSEON; // HSEON -> 1 Enable HSE clock
	while (!(RCC->CR & RCC_CR_HSERDY)) {;} // w8 for HSERDY flag

	/* disable PLL before changes */
	RCC->CR &= ~RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY)) {} // wait till PLL is unlocked

	RCC->CFGR |= RCC_CFGR_PLLSRC; // HSE selected as PLL source

	/* set PLL multiplier value */
	RCC->CFGR &= ~(RCC_CFGR_PLLMUL); // reset first
	RCC->CFGR |= RCC_CFGR_PLLMUL_MUL9; // PLL multiplier = 3

	RCC->CFGR &= ~RCC_CFGR_HPRE; // AHB prescaler clear
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // AHB @ 72 MHz

	RCC->CFGR &= ~RCC_CFGR_PPRE1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 @ 36 MHz

	RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB2 prescaler = 1, APB2 @ 72 MHz

	RCC->CR |= RCC_CR_PLLON; // activate PLL
	while (!(RCC->CR & RCC_CR_PLLRDY)) {} // wait till PLL is locked/ready

	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL; // choose PLL as system clock
	/* Wait for SYSCLK to be PPL */
	while((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

	return;

}
