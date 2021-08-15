/*
 * realizar tarefas FreeRTOS em C++ (ou colocando em C e chamando
 * depois), chamar funcao math_arm e realizar um codigo para controle
 * do EIT. Verificar os limites do hardware e velocidade de envio
 * dos dados para otimizacao.
 */
#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "LED.h"
#include "BTN.h"
#include <iostream>     // std::cout
#include <cstddef>      // std::size_t
#include <valarray>     // std::valarray

// retorna o maior entre dois valores inteiros
template <class T>
T GetMax (T a, T b) {
  T result;
  result = (a>b)? a : b;
  return (result);
}

template <class T, class U>
T GetMin (T a, U b) {
  return (a<b?a:b);
}

void CppMain(){
	LED LED1(LD2_GPIO_Port, LD2_Pin);
	LED LED2(LD3_GPIO_Port, LD3_Pin);
	LED LED3(LD1_GPIO_Port, LD1_Pin);

	// para a funcao GetMax
	int i=5, j=6, k;
	long l=10, m=5, n;
	k = GetMax<int>(i,j);
	n = GetMax<long>(l,m);
	//cout << k << endl;
	//cout << n << endl;

	// template misto para variaveis com declaracoes diferentes
	i = GetMin<int,long> (j,l);

	while(1){
		LED1.Toogle();
		LED2.Toogle();
		LED3.Toogle();
		HAL_Delay(100);
	}
}


