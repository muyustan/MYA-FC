/*
 * mya_f103xx_i2c.c
 *
 *  Created on: Aug 15, 2020
 *      Author: muyustan
 */


#include "mya_f103xx_i2c.h"

void i2c_init(){

	I2C1->CR1 |= 0x8000;	// software reset I2C1
	I2C1->CR1 &= ~0x8000;   // out of reset

	I2C1->CR1 &= ~0x01; // PE = 0

	I2C1->CR2 &= ~0x1F;
	I2C1->CR2 |= 36; // APB1 CLK freq = 36 MHz

	/* timing */
	I2C1->CCR &= ~0xFFF;
	I2C1->CCR |= 180; // for PCLK1 = 36 MHz and I2C SCL at 100 kHz

	I2C1->TRISE = 37; // maximum rise time (I2C clock source freq. + 1)

	I2C1->CR1 |= 0x01; // PE(peripheral enable)

	// dummy_delay(168000); // wait some to stabilize

	/* ********** */

	return;
}

void i2c_start(){

	// while(I2C1->SR2 & 0x2); // wait until bus is not busy
	I2C1->CR1 |= 0x100; // generate start condition
	while (!(I2C1->SR1 & 0x01));   // wait until start flag is set
	return;
}


void i2c_send_slave_addr_to_read(uint8_t slave_addr){

	volatile uint16_t temp;
	I2C1->DR = ((slave_addr << 1) | 0x1);
	while(!(I2C1->SR1 & 0x2)); // wait for end of address transmission
	temp = I2C1->SR2; // to clear ADDR bit
}

void i2c_send_slave_addr_to_write(uint8_t slave_addr){

	volatile uint16_t temp;
	I2C1->DR = ((slave_addr << 1) | 0x0);
	while(!I2C1_FLAG_ADDR); // wait for end of address transmission
	temp = I2C1->SR2; // dummy reading just to clear ADDR bit
}

uint8_t i2c_read_single_byte(uint8_t slave_addr, uint8_t register_addr){

	i2c_start();
	i2c_send_slave_addr_to_write(slave_addr);
	I2C1->DR = register_addr;
	while (!I2C1_FLAG_TxE); // Wait until Tx Data Register is empty
	i2c_start(); // issue a restart
	i2c_send_slave_addr_to_read(slave_addr);
	i2c_stop(); // issue a stop before reading DR
	while(!I2C1_FLAG_RxNE); // wait till RxNE is set
	return ((uint8_t)(I2C1->DR));

}

void i2c_write_single_byte(uint8_t slave_addr, uint8_t mem_addr, uint8_t data){

	volatile uint16_t temp;

	I2C1->CR1 |= I2C_CR1_START;
	while(!I2C1_FLAG_SB); // wait for SB

	I2C1->DR = (slave_addr << 1); // write operation address for the slave
	while(!I2C1_FLAG_ADDR); // wait for ADDR
	temp = I2C1->SR2; // dummy read to clear ADDR

	I2C1->DR = mem_addr;
	while(!I2C1_FLAG_TxE); // wait for Tx buffer to be empty

	I2C1->DR = data;
	while(!I2C1_FLAG_TxE); // wait for Tx buffer to be empty

	I2C1->CR1 |= I2C_CR1_STOP; // generate stop
	while((I2C1->CR1 & I2C_CR1_STOP)); // wait for hardware to clear STOP bit (?)

	return;
}

void i2c_read_dma(uint8_t slave_addr, uint8_t mem_addr, uint8_t len, uint8_t *rxbuff){

	// DMA clock must be enabled before
	uint16_t temp;

	/* configure I2C peripheral for DMA multi byte read */
	I2C1->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;
	I2C1->CR1 |= I2C_CR1_ACK;

	/* I2C1 Rx is connected to DMA1 Channel 7 */
	DMA1->Channel7.CMAR = (uint32_t)rxbuff;
	DMA1->Channel7.CPAR = (uint32_t) &(I2C1->DR);
	DMA1->Channel7.CNDTR = len;
	DMA1->Channel7.CCR |= DMA_CCRx_TCIE | DMA_CCRx_MINC;
	DMA1->Channel7.CCR |= DMA_CCRx_EN; // enable the DMA channel

	I2C1->CR1 |= I2C_CR1_START; // issue a start condition
	while(!I2C1_FLAG_SB); // wait for SB

	I2C1->DR = (slave_addr << 1); // write operation address for the slave
	while(!I2C1_FLAG_ADDR); // wait for ADDR
	temp = I2C1->SR2; // dummy read to clear ADDR

	I2C1->DR = mem_addr;
	while(!I2C1_FLAG_TxE); // wait for Tx buffer to be empty

	I2C1->CR1 |= I2C_CR1_START; // issue a restart condition
	while(!I2C1_FLAG_SB); // wait for SB

	I2C1->DR = ((slave_addr << 1) | 0x01); // read operation address for the slave
	while(!I2C1_FLAG_ADDR); // wait for ADDR
	temp = I2C1->SR2; // dummy read to clear ADDR

	while ((DMA1->ISR & 0x02000000) == 0); // wait till transfer is completed (INTERRUPT!!)
	DMA1->Channel7.CCR &= ~DMA_CCRx_EN; // disable the DMA channel
	DMA1->IFCR |= 0x02000000; // clear the flag

	I2C1->CR1 |= I2C_CR1_STOP;
	while (I2C1->CR1 & I2C_CR1_STOP); // w8 until stop bit is cleared by hw

	/* bring I2C settings back to normal */
	I2C1->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
	I2C1->CR1 &= ~I2C_CR1_ACK;

}

/* reads a byte from the slave memory and applies logical or with @operand, then
 * writes it back.
 */
void i2c_logical_or(uint8_t slave_addr, uint8_t register_addr, uint8_t operand){

	uint8_t temp;
	temp = i2c_read_single_byte(slave_addr, register_addr);
	temp = temp | operand;
	i2c_write_single_byte(slave_addr, register_addr, temp);
	return;
}

void i2c_send_byte_to_bus(uint8_t byte){

	I2C1->DR = byte;
	while(!I2C1_FLAG_TxE); // wait for Tx buffer to be empty

	return;
}

void i2c_stop(){

	// while (!(I2C1->SR1 & 0x04));	// wait until transfer finished
	I2C1->CR1 |= I2C_CR1_STOP;
	while (I2C1->CR1 & I2C_CR1_STOP); // w8 until stop bit is cleared by hw
	return;
}
