/*
 * mya_stm32f103xx.h
 *
 *  Created on: Aug 8, 2020
 *      Author: muyustan
 */

#ifndef MYA_STM32F103XX_H_
#define MYA_STM32F103XX_H_

/* RCC */

#define RCC_BASE 0x40021000U

typedef struct RCC_Registers {

	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;

} RCC_Type;

#define RCC ((RCC_Type*) RCC_BASE)

#define RCC_CR_HSEON 	((uint32_t)0x00010000U)
#define RCC_CR_HSERDY 	((uint32_t)0x00020000U)
#define RCC_CR_CSSON 	((uint32_t)0x00080000U)
#define RCC_CR_PLLON 	((uint32_t)0x01000000U)
#define RCC_CR_PLLRDY 	((uint32_t)0x02000000U)

#define RCC_CFGR_PLLSRC ((uint32_t)0x00010000U)

#define RCC_CFGR_PLLMUL ((uint32_t)0x003C0000U)
	#define RCC_CFGR_PLLMUL_MUL2 	((uint32_t)0x00000000U)
	#define RCC_CFGR_PLLMUL_MUL3 	((uint32_t)0x00040000U)
	#define RCC_CFGR_PLLMUL_MUL4 	((uint32_t)0x00080000U)
	#define RCC_CFGR_PLLMUL_MUL5 	((uint32_t)0x000C0000U)
	#define RCC_CFGR_PLLMUL_MUL6 	((uint32_t)0x00100000U)
	#define RCC_CFGR_PLLMUL_MUL7 	((uint32_t)0x00140000U)
	#define RCC_CFGR_PLLMUL_MUL8 	((uint32_t)0x00180000U)
	#define RCC_CFGR_PLLMUL_MUL9 	((uint32_t)0x001C0000U)
	#define RCC_CFGR_PPRE2			((uint32_t)0x00003800U)
	#define RCC_CFGR_PPRE1			((uint32_t)0x00000700U)
	#define RCC_CFGR_HPRE			((uint32_t)0x000000F0U)

#define RCC_CFGR_SW		((uint32_t)0x00000003U) // system clock switch
	#define RCC_CFGR_SW_HSI ((uint32_t)0x00000000U) // choose HSI as system clock
	#define RCC_CFGR_SW_HSE ((uint32_t)0x00000001U) // choose HSE as system clock
	#define RCC_CFGR_SW_PLL ((uint32_t)0x00000002U) // choose PLL as system clock

#define RCC_CFGR_SWS	((uint32_t)0x0000000CU)
	#define RCC_CFGR_SWS_HSI ((uint32_t)0x00000000U) // HSI used as system clock
	#define RCC_CFGR_SWS_HSE ((uint32_t)0x00000004U) // HSE used as system clock
	#define RCC_CFGR_SWS_PLL ((uint32_t)0x00000008U) // PLL used as system clock


#define GPIOC_EN() (RCC->APB2ENR |= 0x10)

/* sets system clock to 8 x PLLMUL */
void myafc_clock_config(){

	RCC->CR |= RCC_CR_HSEON; // HSEON -> 1 Enable HSE clock
	while (!(RCC->CR & RCC_CR_HSERDY)) {;} // w8 for HSERDY flag

	/* disable PLL before changes */
	RCC->CR &= ~RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY)) {} // wait till PLL is unlocked

	RCC->CFGR |= RCC_CFGR_PLLSRC; // HSE selected as PLL source

	/* set PLL multiplier value */
	RCC->CFGR &= ~(RCC_CFGR_PLLMUL); // reset first
	RCC->CFGR |= RCC_CFGR_PLLMUL_MUL3; // PLL multiplier = 3

	RCC->CFGR &= ~RCC_CFGR_HPRE; // AHB prescaler = 1

	RCC->CFGR &= ~RCC_CFGR_PPRE1; // APB1 prescaler clear

	RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB2 prescaler clear

	RCC->CR |= RCC_CR_PLLON; // activate PLL
	while (!(RCC->CR & RCC_CR_PLLRDY)) {} // wait till PLL is locked/ready

	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL; // choose PLL as system clock
	/* Wait for SYSCLK to be PPL */
	while((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

	return;

}

/* END RCC */





/* GPIO */

#define GPIOC_BASE 0x40011000U

typedef struct GPIO_Registers {

	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;

} GPIO_Type;

#define GPIOC ((GPIO_Type*) GPIOC_BASE)

/* END GPIO */

#endif /* MYA_STM32F103XX_H_ */
