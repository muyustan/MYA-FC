/*
 * mya_f103xx_i2c.h
 *
 *  Created on: Aug 15, 2020
 *      Author: muyustan
 */

#ifndef MYA_F103XX_I2C_H_
#define MYA_F103XX_I2C_H_

#include "mya_stm32f103xx.h"

#define I2C1_FLAG_SB	(I2C1->SR1 & 0x01)
#define I2C1_FLAG_ADDR	(I2C1->SR1 & 0x02)
#define I2C1_FLAG_BTF	(I2C1->SR1 & 0x04)
#define I2C1_FLAG_RxNE	(I2C1->SR1 & 0x40)
#define I2C1_FLAG_TxE	(I2C1->SR1 & 0x80)

void i2c_init();

void i2c_start();

void i2c_send_slave_addr_to_read(uint8_t slave_addr);

void i2c_send_slave_addr_to_write(uint8_t slave_addr);

uint8_t i2c_read_single_byte(uint8_t slave_addr, uint8_t register_addr);

void i2c_read_dma(uint8_t slave_addr, uint8_t mem_addr, uint8_t len, uint8_t *rxbuff);

void i2c_write_single_byte(uint8_t slave_addr, uint8_t mem_addr, uint8_t data);

void i2c_logical_or(uint8_t slave_addr, uint8_t register_addr, uint8_t operand);

void i2c_send_byte_to_bus(uint8_t byte);

void i2c_stop();


#endif /* MYA_F103XX_I2C_H_ */
