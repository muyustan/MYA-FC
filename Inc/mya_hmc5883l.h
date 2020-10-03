/*
 * mya_hmc5883l.h
 *
 *  Created on: Aug 26, 2020
 *      Author: muyustan
 */

#ifndef MYA_HMC5883L_H_
#define MYA_HMC5883L_H_

#define HMC5883L_I2C_ADDR	0x1E

/* define internal register addresses */

#define HMC5883L_CONFIGURATION_REG_A		0x00 // rw
#define HMC5883L_CONFIGURATION_REG_B		0x01 // rw
#define HMC5883L_MODE_REG					0x02 // rw
#define HMC5883L_DATA_OUTPUT_X_H			0x03 // all others r only
#define HMC5883L_DATA_OUTPUT_X_L			0x04
#define HMC5883L_DATA_OUTPUT_Z_H			0x05
#define HMC5883L_DATA_OUTPUT_Z_L			0x06
#define HMC5883L_DATA_OUTPUT_Y_H			0x07
#define HMC5883L_DATA_OUTPUT_Y_L			0x08
#define HMC5883L_STATUS_REG					0x09
#define HMC5883L_IDENTIFICATION_REG_A		0x0A
#define HMC5883L_IDENTIFICATION_REG_B		0x0B
#define HMC5883L_IDENTIFICATION_REG_C		0x0C



#endif /* MYA_HMC5883L_H_ */
