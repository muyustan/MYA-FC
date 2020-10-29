/*
 * mya_ms5611.c
 *
 *  Created on: Aug 23, 2020
 *      Author: muyustan
 */


#include "mya_ms5611.h"

void delay_ms(); // let the compiler know that there is a definition for this function somewhere else

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


void ms5611_reset(){

	ms5611_send_command(MS5611_COMMAND_RESET);
	delay_ms(5); // THIS DELAY IS MANDATORY FOR PROPER OPERATION!!
	return;
}

void ms5611_get_coefficients(uint16_t *arr){

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
