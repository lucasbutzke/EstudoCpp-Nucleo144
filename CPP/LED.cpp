/*
 * LED.cpp
 *
 *  Created on: Sep 2, 2020
 *      Author: lucas
 */
#include "LED.h"

LED::LED(GPIO_TypeDef* gb, uint16_t gp_pin) {
	// TODO Auto-generated constructor stub
	GPIOx = gb;
	GPIO_Pin = gp_pin;
}

LED::~LED() {
	// TODO Auto-generated destructor stub
}

void LED::On(){
	LL_GPIO_SetOutputPin(GPIOx, GPIO_Pin);
}

void LED::Off(){
	LL_GPIO_ResetOutputPin(GPIOx, GPIO_Pin);
}

void LED::Toogle(){
	LL_GPIO_TogglePin(GPIOx, GPIO_Pin);
	GPIOx -> ODR ^= GPIO_Pin;
}
