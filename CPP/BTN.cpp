/*
 * BTN.cpp
 *
 *  Created on: Sep 2, 2020
 *      Author: lucas
 */

#include <BTN.h>

BTN::BTN(GPIO_TypeDef* gb, uint16_t gp_pin) {
	// TODO Auto-generated constructor stub
	GPIOx = gb;
	GPIO_Pin = gp_pin;
}

BTN::~BTN() {
	// TODO Auto-generated destructor stub
}

// verificar se esta retornando o BIT correto de leitura do botao
int BTN::Read(){
	return LL_GPIO_ReadInputPort(GPIOx) & GPIO_Pin;
	USER_Btn_GPIO_Port -> IDR &  USER_Btn_Pin;
}
