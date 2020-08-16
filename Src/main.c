#include "main.h"

void dummy_delay(uint32_t t){

	while(t--);
	return;

}

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void TIM2_IRQHandler(){

	// GPIOC->ODR ^= (0x1 << 13);
	TIM2->SR &= ~(0x01); // clear UIF
	return;
}


int main(void)
{

	clock_config();
	GPIOA_EN(); // pwm
	GPIOB_EN(); // i2c
	GPIOC_EN(); // led
	AFIO_EN();
	TIM2_EN(); // pwm

	GPIOC->CRH = 0x300000;
	GPIOC->BSRR |= 0x2000;

	/* TIM2 PWM */
	GPIOA->CRL |= 0x01; // output mode for PA0(T2C1)
	GPIOA->CRL &= ~(0x0C);
	GPIOA->CRL |= 0x08; // AF push-pull

	TIM2->PSC = 23; // Timer counts at 1 MHz
	TIM2->ARR = 19999;  // set frequency of counter overflow to 50 Hz
	// so the total width of one single wave is 20 ms
	// CCR = 999 : 1000 us
	// CCR = 1999 : 2000 us
	// desired pulse width (us) = x ;
	// CCR = x - 1

	TIM2->CCR1 = 1299;

	TIM2->DIER |= 0x01; // update interrupt enable

	TIM2->CCMR1 &= ~(0x03); // set CC1 channel as output channel
	TIM2->CCMR1 &= ~(0b111 << 4);
	TIM2->CCMR1 |= (0b110 << 4); // PWM mode 1

	TIM2->CCER |= 0x01; // CC1E enable the channel

	TIM2->CR1 |= 0x01; // Counter Enable (CEN)

	NVIC->ISER[0] |= 0x01 << 28; // enable TIM2 global interrupt

	/* I2C1 */

	GPIOB->CRL &= ~(0xF << 6*4);
	GPIOB->CRL |= (0xE << 6*4); // set PB6(SCL1) as AF output open-drain

	GPIOB->CRL &= ~(0xF << 7*4);
	GPIOB->CRL |= (0xE << 7*4); // set PB7(SDA1) as AF output open-drain

	for(;;){
	}
}
