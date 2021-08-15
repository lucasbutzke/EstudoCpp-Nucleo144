/*
 * BTN.h
 *
 *  Created on: Sep 2, 2020
 *      Author: lucas
 */

#ifndef BTN_H_
#define BTN_H_

#include "main.h"
#include "stm32f7xx_ll_gpio.h"

class BTN {
public:
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	BTN(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	virtual ~BTN();
	int Read();
};

#endif /* BTN_H_ */
