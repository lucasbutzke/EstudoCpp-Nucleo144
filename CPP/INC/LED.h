/*
 * LED.h
 *
 *  Created on: Sep 2, 2020
 *      Author: lucas
 */

#ifndef LED_H_
#define LED_H_

#include "main.h"
#include "stm32f7xx_ll_gpio.h"

class LED {
public:
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;

	LED(GPIO_TypeDef* gb, uint16_t gp_pin);
	virtual ~LED();
	void On();
	void Off();
	void Toogle();
};

#endif /* LED_H_ */
