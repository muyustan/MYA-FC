/*
 * mya_f103xx_gpio.h
 *
 *  Created on: Aug 15, 2020
 *      Author: muyustan
 */

#ifndef MYA_F103XX_GPIO_H_
#define MYA_F103XX_GPIO_H_

#include "mya_stm32f103xx.h"

#define GPIO_PIN_0  0x00000001U
#define GPIO_PIN_1  0x00000002U
#define GPIO_PIN_2  0x00000004U
#define GPIO_PIN_3  0x00000008U
#define GPIO_PIN_4  0x00000010U
#define GPIO_PIN_5  0x00000020U
#define GPIO_PIN_6  0x00000040U
#define GPIO_PIN_7  0x00000080U
#define GPIO_PIN_8  0x00000100U
#define GPIO_PIN_9  0x00000200U
#define GPIO_PIN_10 0x00000400U
#define GPIO_PIN_11 0x00000800U
#define GPIO_PIN_12 0x00001000U
#define GPIO_PIN_13 0x00002000U
#define GPIO_PIN_14 0x00004000U
#define GPIO_PIN_15 0x00008000U

typedef enum {

	LOW = 0,
	HIGH = 1

} GPIO_State;

#define led_on()		gpio_write(GPIOC, GPIO_PIN_13, LOW)
#define led_off()		gpio_write(GPIOC, GPIO_PIN_13, HIGH)
#define led_toggle()	gpio_toggle(GPIOC, GPIO_PIN_13)

/* function prototypes */
uint8_t gpio_read(GPIO_Type* GPIOx, uint8_t pin_number);
void gpio_write(GPIO_Type* GPIOx, uint16_t GPIO_PIN_MASK, GPIO_State state);
void gpio_toggle(GPIO_Type* GPIOx, uint16_t GPIO_PIN_MASK);


#endif /* MYA_F103XX_GPIO_H_ */
