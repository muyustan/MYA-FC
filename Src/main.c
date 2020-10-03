#include "main.h"

/* private definitions */

#define ACCELEROMETER_SENSITIVITY 16384 // per g
#define GYROSCOPE_SENSITIVITY 131 // per degrees/seconds

#define MS5611_I2C_ADDR		0x77 // barometer
#define HMC5883L_I2C_ADDR	0x1E // magnetometer

/* end private definitions */

/* global variables */

volatile uint32_t ms_since_startup;

int16_t accX_raw, accY_raw, accZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

uint8_t mpu6050_data_buffer[14] = {0};

float temperature;

uint8_t a;
uint8_t bmpID;
uint16_t coeffs[6];
/* end global variables */

void dummy_delay(uint32_t t){

	while(t--);
	return;

}

/* ticks cannot exceed 24 bits unsigned value */
#define SysTick_LOAD(ticks) (SysTick->LOAD = (ticks - 1))
    /* the time duration between two consecutive
    SysTick interrupt is (RE)LOAD + 1 */

void SysTick_Initialize(){

	ms_since_startup = 0;

    SysTick->CTRL = 0x00; // disable the SysTick
    // also sets clock for SysTick to AHB/8

    /* for now, the system clock AHB freq. is taken as
    constant and 72 MHz */
    /* So, clock for SysTick is at 72/8 = 9 MHz */

    SysTick_LOAD(9000);

    SysTick->VAL = 0x00; // clear the counter current value

    SysTick->CTRL |= (SysTick_CTRL_ENABLE | SysTick_CTRL_TICKINT);
    // enable both the counter and interrupt

	return;
}

void SysTick_Handler(){

	ms_since_startup++;

	return;
}

void delay_ms(uint16_t t){

	uint32_t start_ms = ms_since_startup;

	while((ms_since_startup - start_ms) <= t){;}

	return;
}


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void TIM2_IRQHandler(){

	return;
}

float mya_absf(float x){

	return x > 0.f ? x : -x;

}

int main(void)
{

	clock_config();
	SysTick_Initialize();

	accX_raw = accY_raw = accZ_raw = 0;
	accX = accY = accZ = 0.0f;
	gyroX_raw = gyroY_raw = gyroZ_raw = 0;
	gyroX = gyroY = gyroZ = 0.0f;
	temperature = 0;



	GPIOA_EN(); // pwm
	GPIOB_EN(); // i2c
	GPIOC_EN(); // led
	AFIO_EN();
	TIM2_EN(); // pwm
	DMA1_EN(); // for i2c read

	GPIOC->CRH = 0x300000;
	GPIOC->BSRR |= 0x2000;

	led_on();
	delay_ms(3000);
	led_off();

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

	// TIM2->DIER |= 0x01; // update interrupt enable

	TIM2->CCMR1 &= ~(0x03); // set CC1 channel as output channel
	TIM2->CCMR1 &= ~(0b111 << 4);
	TIM2->CCMR1 |= (0b110 << 4); // PWM mode 1

	TIM2->CCER |= 0x01; // CC1E enable the channel

	TIM2->CR1 |= 0x01; // Counter Enable (CEN)

	NVIC->ISER[0] |= 0x01 << 28; // enable TIM2 global interrupt

	/* GPIO configurations for I2C1 */

	GPIOB->CRL &= ~(0xF << 6*4);
	GPIOB->CRL |= (0xE << 6*4); // set PB6(SCL1) as AF output open-drain

	GPIOB->CRL &= ~(0xF << 7*4);
	GPIOB->CRL |= (0xE << 7*4); // set PB7(SDA1) as AF output open-drain

	/* I2C1 initiation */

	I2C1_EN();
	i2c_init();
	bmpID = 99;


	//i2c_write_single_byte(0x77, 0xf4, 0x22);

	//i2c_read_dma(0x77, 0xD0, 1, &bmpID);
	//bmpID = i2c_read_single_byte(0x77, 0xD0);


	/* MPU6050 initiation */

	mpu6050_wake_up();
	led_toggle();
	dummy_delay(84000); // to stabilize the sensor


	mpu6050_i2c_bypass_en();

	//i2c_write_single_byte(HMC5883L_I2C_ADDR, 0x01, 0x00);
	bmpID = i2c_read_single_byte(HMC5883L_I2C_ADDR, 12);

	ms5611_reset();

	dummy_delay(16800);

	ms5611_get_coefficients(coeffs);

	// led_toggle(); // sensor ready


	for(;;){


		mpu6050_read_burst(59, 14, mpu6050_data_buffer);

		accX_raw = (mpu6050_data_buffer[0] << 8) | mpu6050_data_buffer[1];
		accY_raw = (mpu6050_data_buffer[2] << 8) | mpu6050_data_buffer[3];
		accZ_raw = (mpu6050_data_buffer[4] << 8) | mpu6050_data_buffer[5];

		accX = (float)accX_raw / ACCELEROMETER_SENSITIVITY;
		accY = (float)accY_raw / ACCELEROMETER_SENSITIVITY;
		accZ = (float)accZ_raw / ACCELEROMETER_SENSITIVITY;

		temperature = ((int16_t) ((mpu6050_data_buffer[6] << 8) | mpu6050_data_buffer[7]) / 340.0f) + 36.53f;

		gyroX_raw = (mpu6050_data_buffer[8] << 8) | mpu6050_data_buffer[9];
		gyroY_raw = (mpu6050_data_buffer[10] << 8) | mpu6050_data_buffer[11];
		gyroZ_raw = (mpu6050_data_buffer[12] << 8) | mpu6050_data_buffer[13];

		gyroX = (float)gyroX_raw / GYROSCOPE_SENSITIVITY;
		gyroY = (float)gyroY_raw / GYROSCOPE_SENSITIVITY;
		gyroZ = (float)gyroZ_raw / GYROSCOPE_SENSITIVITY;

		if(mya_absf(accX) > 0.5){
			led_on();
		}
		else
			led_off();

	}
}


void i2c_clear_bus(){

	I2C1_EN();
	I2C1->CR1 &= ~0x01; // PE = 0
	GPIOB->CRL &= ~(0b1111 << 24); // pb6 gp od output scl
	GPIOB->CRL &= ~(0b1111 << 28); // pb7 gp od output sda
	GPIOB->CRL |= (0b0110 << 24); // pb6 gp od output scl
	GPIOB->CRL |= (0b0110 << 28); // pb7 gp od output sda
	GPIOB->ODR |= GPIO_PIN_6 | GPIO_PIN_7;
	while(!gpio_read(GPIOB, 6) || !gpio_read(GPIOB, 7));
	GPIOB->ODR &= ~GPIO_PIN_7;
	while(gpio_read(GPIOB, 7));
	GPIOB->ODR  &= ~(GPIO_PIN_6);
	while(gpio_read(GPIOB, 6));
	GPIOB->ODR |= GPIO_PIN_6;
	while(!gpio_read(GPIOB, 6));
	GPIOB->ODR |= GPIO_PIN_7;
	while(!gpio_read(GPIOB, 7));


	I2C1->CR1 |= 0x8000;	// software reset I2C1
	I2C1->CR1 &= ~0x8000;   // out of reset
	I2C1->CR1 |= 0x01; // PE = 1


}

