/*
 * Classe Base para funturas implementacoes de outras
 * classes que necessitam funcoes matematicas.
 *
 * Criar funcoes simples com templates e metodos com
 * ARM_MATH para utilizar. Utilizar o maximo possivel
 * para estudos com templates e structs tbm.
 */

#ifndef MATHFUNS_H_
#define MATHFUNS_H_

/*
#define ARM_MATH_CM7  // Use ARM Cortex M7
#define __FPU_PRESENT 1   // Does this device have a floating point unit?
#include <arm_math.h> // Include CMSIS header
*/
#include <vector>
#include <string>
#include <cmath>

/*
 * Wrapper around all desirable functions to call on
 * STM32 and ARM DSP and even string manipulations
 */
class MathFuns {
public:
	// declarar o vetor ou array onde seriam retornados
	// os valores para comunicao ou qualquer outra funcao
	/*
	 * example em templatesEstudo
	 */
	MathFuns();
	virtual ~MathFuns();
	float NewSine();
};

#endif /* MATHFUNS_H_ */
