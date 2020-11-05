/*
 * mya_ms5611.c
 *
 *  Created on: Aug 23, 2020
 *      Author: muyustan
 */


#include "mya_ms5611.h" // barometric pressure sensor

void delay_ms(); // let the compiler know that there is a definition for this function somewhere else

/* global variables */
float temperature, pressure;
uint32_t D1, D2;
uint16_t coeffs[6] = {0};
int32_t dT, TEMP;
float T;
int64_t OFF, SENS, P;
/* end global variables */


/* static functions */
static uint32_t pow_int(int base, int exponent){

    int res = 1;

    while (exponent--){

        res *= base;

    }

    return res;
} // my integer power function

static void ms5611_send_command(uint8_t command){

	i2c_start();
	i2c_send_slave_addr_to_write(MS5611_I2C_ADDR);
	i2c_send_byte_to_bus(command);
	i2c_stop();

	return;
}

static void ms5611_get_answer(uint8_t *buff, uint8_t len){

	/* configure I2C peripheral for DMA multi byte read */
	I2C1->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;
	I2C1->CR1 |= I2C_CR1_ACK;

	/* I2C1 Rx is connected to DMA1 Channel 7 */
	DMA1->Channel7.CMAR = (uint32_t)buff;
	DMA1->Channel7.CPAR = (uint32_t) &(I2C1->DR);
	DMA1->Channel7.CNDTR = len;
	DMA1->Channel7.CCR |= DMA_CCRx_TCIE | DMA_CCRx_MINC;
	DMA1->Channel7.CCR |= DMA_CCRx_EN; // enable the DMA channel

	i2c_start();
	i2c_send_slave_addr_to_read(MS5611_I2C_ADDR);

	while ((DMA1->ISR & 0x02000000) == 0); // wait till transfer is completed (INTERRUPT!!)
	DMA1->Channel7.CCR &= ~DMA_CCRx_EN; // disable the DMA channel
	DMA1->IFCR |= 0x02000000; // clear the flag

	i2c_stop();

	/* bring I2C settings back to normal */
	I2C1->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
	I2C1->CR1 &= ~I2C_CR1_ACK;

}

static void ms5611_read_coefficients(uint16_t *arr){

	uint8_t i = 0;
	uint8_t temp[2] = {0};

	while(i < 6){

		ms5611_send_command(MS5611_COMMAND_PROM_READ_C1 + i*2);
		ms5611_get_answer(temp, 2);
		arr[i] = (temp[0] << 8) | temp[1];
		i++;
	}

	return;
}

static void ms5611_update_D1(void){

	uint8_t temp[3] = {0};
	ms5611_send_command(MS5611_COMMAND_CONVERT_D1_OSR_2048);
	delay_ms(5);
	ms5611_send_command(MS5611_COMMAND_ADC_READ);
	ms5611_get_answer(temp, 3);
	D1 = (uint32_t) (temp[0] << 16 | temp[1] << 8 | temp[2]);
	return;
}

static void ms5611_update_D2(void){

	uint8_t temp[3] = {0};
	ms5611_send_command(MS5611_COMMAND_CONVERT_D2_OSR_2048);
	delay_ms(5);
	ms5611_send_command(MS5611_COMMAND_ADC_READ);
	ms5611_get_answer(temp, 3);
	D2 = (uint32_t) (temp[0] << 16 | temp[1] << 8 | temp[2]);
	return;
}

static void ms5611_update_temperature(){

	ms5611_update_D2();

	dT = D2 - coeffs[4] * 256; // D2 - C5 * 2^8
	TEMP = 2000 + dT * coeffs[5] / pow_int(2, 23);
	T = TEMP / 100.0f;

	if (T < 100 && T > -30) // do not update the value of temperature if it is noise
		temperature = T;

	return;

}

static void ms5611_update_pressure(){ // compansate using most recent temperature value

	ms5611_update_D1();

	OFF = coeffs[1] * pow_int(2, 16) + (coeffs[3] * dT) / pow_int(2, 7); // OFF = C2 * 2^16 + (C4 * dT )/ 2^7
	SENS = coeffs[0] * pow_int(2, 15) + (coeffs[2]  * dT) / pow_int(2, 8);
	P = (D1 * SENS / pow_int(2, 21) - OFF) / pow_int(2, 15);

	pressure = P / 100.0f;

	return;
}


/* end static functions */


void ms5611_reset(void){

	ms5611_send_command(MS5611_COMMAND_RESET);
	delay_ms(50); // THIS DELAY IS MANDATORY FOR PROPER OPERATION!!
	return;
}

void ms5611_init(void){

	D1 = 0;
	D2 = 0;
	temperature = 0;
	pressure = 0;
	ms5611_reset();
	ms5611_read_coefficients(coeffs);
	return;
}

void ms5611_update(void) {

	ms5611_update_temperature();
	ms5611_update_pressure();

}

