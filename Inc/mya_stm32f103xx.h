/*
 * mya_stm32f103xx.h
 *
 *  Created on: Aug 8, 2020
 *      Author: muyustan
 */

#ifndef MYA_STM32F103XX_H_
#define MYA_STM32F103XX_H_

#include <stdint.h>

/* NVIC */

#define NVIC_BASE 0xE000E100U

typedef struct NVIC_Registers {

	volatile uint32_t ISER[3];
	volatile uint8_t RESERVED0[120];
	volatile uint32_t ICER[3];
	// ...

} NVIC_Type;

#define NVIC ((NVIC_Type*) NVIC_BASE)

/* END NVIC */

/* SysTick Timer */

#define SysTick_BASE	0xE000E010

typedef struct SysTick_Registers {

	volatile uint32_t CTRL;
	volatile uint32_t LOAD;
	volatile uint32_t VAL;
	volatile uint32_t CALIB;

} SysTick_Type;

#define SysTick ((SysTick_Type*) SysTick_BASE)

/* END SysTick Timer */

/********** FLASH ***************/

#define FLASH_BASE 0x40022000U

#define FLASH_ACR_LATENCY 		0x00000007U
#define FLASH_ACR_HLFCYA		0x00000008U
#define FLASH_ACR_PRFTBE		0x00000010U
#define FLASH_ACR_PRFTBS		0x00000020U


typedef struct FLASH {

	volatile uint32_t ACR;
	volatile uint32_t KEYR;
	volatile uint32_t OPTKEYR;
	volatile uint32_t SR;
	volatile uint32_t CR;
	volatile uint32_t AR;
	volatile uint32_t RESERVED;
	volatile uint32_t OBR;
	volatile uint32_t WRPR;

} FLASH_Type;

#define FLASH ((FLASH_Type*) FLASH_BASE)


/********* END FLASH **************/


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
	#define RCC_CFGR_PPRE1_DIV1		((uint32_t)0x00000000U)
	#define RCC_CFGR_PPRE1_DIV2		((uint32_t)0x00000400U)
	#define RCC_CFGR_PPRE1_DIV4		((uint32_t)0x00000500U)
	#define RCC_CFGR_PPRE1_DIV8		((uint32_t)0x00000600U)
	#define RCC_CFGR_PPRE1_DIV16	((uint32_t)0x00000700U)


#define RCC_CFGR_HPRE			((uint32_t)0x000000F0U)
	#define RCC_CFGR_HPRE_DIV1		((uint32_t)0x00000000U)
	#define RCC_CFGR_HPRE_DIV2		((uint32_t)0x00000080U)
	#define RCC_CFGR_HPRE_DIV4		((uint32_t)0x00000090U)
	#define RCC_CFGR_HPRE_DIV8		((uint32_t)0x000000A0U)
	#define RCC_CFGR_HPRE_DIV16		((uint32_t)0x000000B0U)
	#define RCC_CFGR_HPRE_DIV64		((uint32_t)0x000000C0U)
	#define RCC_CFGR_HPRE_DIV128	((uint32_t)0x000000D0U)
	#define RCC_CFGR_HPRE_DIV256	((uint32_t)0x000000E0U)
	#define RCC_CFGR_HPRE_DIV512	((uint32_t)0x000000F0U)

#define RCC_CFGR_SW		((uint32_t)0x00000003U) // system clock switch
	#define RCC_CFGR_SW_HSI ((uint32_t)0x00000000U) // choose HSI as system clock
	#define RCC_CFGR_SW_HSE ((uint32_t)0x00000001U) // choose HSE as system clock
	#define RCC_CFGR_SW_PLL ((uint32_t)0x00000002U) // choose PLL as system clock

#define RCC_CFGR_SWS	((uint32_t)0x0000000CU)
	#define RCC_CFGR_SWS_HSI ((uint32_t)0x00000000U) // HSI used as system clock
	#define RCC_CFGR_SWS_HSE ((uint32_t)0x00000004U) // HSE used as system clock
	#define RCC_CFGR_SWS_PLL ((uint32_t)0x00000008U) // PLL used as system clock

/* END RCC */

/* GPIO */

#define GPIOA_BASE 0x40010800U
#define GPIOB_BASE 0x40010C00U
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

#define GPIOA ((GPIO_Type*) GPIOA_BASE)
#define GPIOB ((GPIO_Type*) GPIOB_BASE)
#define GPIOC ((GPIO_Type*) GPIOC_BASE)

/* END GPIO */

/* AFIO */

#define AFIO_BASE 0x4001000

typedef struct AFIO_Registers {

	volatile uint32_t EVCR;
	volatile uint32_t MAPR;
	volatile uint32_t EXTICR1;
	volatile uint32_t EXTICR2;
	volatile uint32_t EXTICR3;
	volatile uint32_t EXTICR4;
	volatile uint32_t RESERVED0;
	volatile uint32_t MAPR2;

} AFIO_Type;

#define AFIO ((AFIO_Type*) AFIO_BASE)

/* END AFIO */

/* TIMx (2, 3, 4) */

#define TIM2_BASE 0x40000000U
#define TIM3_BASE 0x40000400U
#define TIM4_BASE 0x40000800U

typedef struct TIM_2_3_4_Registers {

	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t RESERVED0;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	volatile uint32_t RESERVED1;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;

} TIM_2_3_4_Type;

#define TIM2 ((TIM_2_3_4_Type*) TIM2_BASE)
#define TIM3 ((TIM_2_3_4_Type*) TIM3_BASE)
#define TIM4 ((TIM_2_3_4_Type*) TIM4_BASE)


/* END TIMx (2, 3, 4) */

/* I2Cx */

#define I2C1_BASE 0x40005400U
#define I2C2_BASE 0x40005800U

typedef struct I2Cx_Registers {

	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;

} I2Cx_Type;

/* register bit masks */
#define I2C_CR1_PE				0x0001U
#define I2C_CR1_SMBUS			0x0002U
#define I2C_CR1_SMBTYPE			0x0008U
#define I2C_CR1_ENARP			0x0010U
#define I2C_CR1_ENPEC			0x0020U
#define I2C_CR1_ENGC			0x0040U
#define I2C_CR1_NOSTRETCH		0x0080U
#define I2C_CR1_START			0x0100U
#define I2C_CR1_STOP			0x0200U
#define I2C_CR1_ACK				0x0400U
#define I2C_CR1_POS				0x0800U
#define I2C_CR1_PEC				0x1000U
#define I2C_CR1_ALERT			0x2000U
#define I2C_CR1_SWRST			0x8000U

#define I2C_CR2_FREQ			0x001FU
#define I2C_CR2_ITERREN			0x0100U
#define I2C_CR2_ITEVTEN			0x0200U
#define I2C_CR2_ITBUFEN			0x0400U
#define I2C_CR2_DMAEN			0x0800U
#define I2C_CR2_LAST			0x1000U


#define I2C_CCR_CCR				0x0FFFU

#define I2C1 ((I2Cx_Type*) I2C1_BASE)
#define I2C2 ((I2Cx_Type*) I2C2_BASE)

/* END I2Cx */

/* DMA */

#define DMA1_BASE 0x40020000U

typedef struct DMA_Channel_Registers {

	volatile uint32_t CCR;
	volatile uint32_t CNDTR;
	volatile uint32_t CPAR;
	volatile uint32_t CMAR;
	volatile uint32_t RESERVED;

} DMA_Channelx_Type;

typedef struct DMA_Global_Registers {

	volatile uint32_t ISR;
	volatile uint32_t IFCR;
	volatile DMA_Channelx_Type Channel1;
	volatile DMA_Channelx_Type Channel2;
	volatile DMA_Channelx_Type Channel3;
	volatile DMA_Channelx_Type Channel4;
	volatile DMA_Channelx_Type Channel5;
	volatile DMA_Channelx_Type Channel6;
	volatile DMA_Channelx_Type Channel7;

} DMAx_Type;

#define DMA1 ((DMAx_Type*) DMA1_BASE)

#define DMA_CCRx_EN			0x0001U
#define DMA_CCRx_TCIE		0x0002U
#define DMA_CCRx_HTIE		0x0004U
#define DMA_CCRx_TEIE		0x0008U
#define DMA_CCRx_DIR		0x0010U
#define DMA_CCRx_CIRC		0x0020U
#define DMA_CCRx_PINC		0x0040U
#define DMA_CCRx_MINC		0x0080U
#define DMA_CCRx_PSIZE		0x0300U
#define DMA_CCRx_MSIZE		0x0C00U
#define DMA_CCRx_PL			0x3000U
#define DMA_CCRx_MEM2MEM	0x4000U


/* END DMA */

#endif /* MYA_STM32F103XX_H_ */
