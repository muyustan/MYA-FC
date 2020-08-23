/*
 * mya_f103xx_mpu6050.c
 *
 *  Created on: Aug 21, 2020
 *      Author: muyustan
 */


#include "mya_f103xx_mpu6050.h"

void mpu6050_write(uint8_t reg_addr, uint8_t data){

	i2c_write_single_byte(MPU6050_I2C_ADDR, reg_addr, data);

}

void mpu6050_read_burst(uint8_t start_reg_addr, uint8_t num_bytes, uint8_t *data_container){

	i2c_read_dma(MPU6050_I2C_ADDR, start_reg_addr, num_bytes, data_container);

}

void mpu6050_wake_up(){

	mpu6050_write(MPU6050_PWR_MGMT_1, 0x00);

}

void mpu6050_i2c_bypass_en(){

	i2c_logical_or(MPU6050_I2C_ADDR, MPU6050_INT_PIN_CFG, MPU6050_INT_PIN_CFG_I2C_BYPASS_EN);

	return;
}
