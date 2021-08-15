/*
 * Implementar para USART
 * receber e salvar os dados dentro de vetores
 * tentar implementar DMA com compartilhamento com outros processos
 */

#ifndef CONEXCAO_H_
#define CONEXCAO_H_

#include "MathFuns.h"

/*
 * Class para communication e manipular as devidas funcoes
 * para enviar e receber os dados. Tentar desenvolver uma
 * conexcao assincrona e enviar os dados atraves de vetores.
 */
class Conexcao : public MathFuns
{
public:
	// mensagem inicial para enviar ao Serial
	std::vector<std::string> msg{ "Hello", "UART", "\r\n", "Starting", "Comm", "\r\n"};

	// receber USART handler
	Conexcao();
	virtual ~Conexcao();

	void Comecar();
	void Receber();
	void Enviar();
	void Terminar();
};

#endif /* CONEXCAO_H_ */
