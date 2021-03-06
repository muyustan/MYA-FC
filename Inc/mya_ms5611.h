/*
 * mya_ms5611.h
 *
 *  Created on: Aug 23, 2020
 *      Author: muyustan
 */

#ifndef MYA_MS5611_H_
#define MYA_MS5611_H_

#include "mya_f103xx_i2c.h"

#define MS5611_I2C_ADDR		0x77 /* CSB tied low, PS tied high */

/* define command bytes */

#define MS5611_COMMAND_RESET				0x1E

#define MS5611_COMMAND_CONVERT_D1_OSR_256	0x40
#define MS5611_COMMAND_CONVERT_D1_OSR_512	0x42
#define MS5611_COMMAND_CONVERT_D1_OSR_1024	0x44
#define MS5611_COMMAND_CONVERT_D1_OSR_2048	0x46
#define MS5611_COMMAND_CONVERT_D1_OSR_4096	0x48

#define MS5611_COMMAND_CONVERT_D2_OSR_256	0x50
#define MS5611_COMMAND_CONVERT_D2_OSR_512	0x52
#define MS5611_COMMAND_CONVERT_D2_OSR_1024	0x54
#define MS5611_COMMAND_CONVERT_D2_OSR_2048	0x56
#define MS5611_COMMAND_CONVERT_D2_OSR_4096	0x58

#define MS5611_COMMAND_ADC_READ				0x00

#define MS5611_COMMAND_PROM_READ_C1			0xA2
#define MS5611_COMMAND_PROM_READ_C2			0xA4
#define MS5611_COMMAND_PROM_READ_C3			0xA6
#define MS5611_COMMAND_PROM_READ_C4			0xA8
#define MS5611_COMMAND_PROM_READ_C5			0xAA
#define MS5611_COMMAND_PROM_READ_C6			0xAC

/* function prototypes */

void ms5611_init(void);

void ms5611_reset(void);

void ms5611_update(void);


#endif /* MYA_MS5611_H_ */
