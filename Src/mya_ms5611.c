/*
 * mya_ms5611.c
 *
 *  Created on: Aug 23, 2020
 *      Author: muyustan
 */


#include "mya_ms5611.h"


void ms5611_reset(){

	i2c_start();
	i2c_send_slave_addr_to_write(MS5611_I2C_ADDR);
	i2c_send_byte_to_bus(MS5611_COMMAND_RESET);
	i2c_stop();
	return;
}
