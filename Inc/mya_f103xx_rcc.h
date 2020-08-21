/*
 * mya_f103_rcc.h
 *
 *  Created on: Aug 15, 2020
 *      Author: muyustan
 */

#ifndef MYA_F103XX_RCC_H_
#define MYA_F103XX_RCC_H_

#include "mya_stm32f103xx.h"

#define GPIOA_EN()	(RCC->APB2ENR |= 0x04)
#define GPIOB_EN()	(RCC->APB2ENR |= 0x08)
#define GPIOC_EN()	(RCC->APB2ENR |= 0x10)
#define AFIO_EN()	(RCC->APB2ENR |= 0x01)

#define TIM2_EN()	(RCC->APB1ENR |= 0x01)
#define TIM3_EN()	(RCC->APB1ENR |= 0x02)
#define TIM4_EN()	(RCC->APB1ENR |= 0x04)

#define I2C1_EN() (RCC->APB1ENR |= 0x200000)
#define I2C2_EN() (RCC->APB1ENR |= 0x400000)

#define DMA1_EN() (RCC->AHBENR |= 0x01U)

/* function prototypes */
void clock_config();

#endif /* MYA_F103XX_RCC_H_ */
