/*
 * Implementacao com USART
 */

#include <Conexcao.h>

/*
 * Comecar enviando uma mensagem para mostrar que
 * comunicao esta funcionando
 */
Conexcao::Conexcao() {
	// TODO Auto-generated constructor stub

}

Conexcao::~Conexcao() {
	// TODO Auto-generated destructor stub
}

/*
 * Inicializar conexcao e esperar em LISTENING state
 * (talvez eh somente necessario para ETHERNET)
 */
void Conexcao::Comecar(){

}

/*
 * Receber dados e salvar no buffer enquanto executa outra thread.
 * Salvar em um vetor os dados e manipula-los depois com as funcoes
 * em MathFuns.
 */
void Conexcao::Receber(){

}

/*
 * enviar dados no buffer enquanto executa outra thread,
 * talvez verificar o dado (se eh array ou vetor) e enviar
 * ao serial.
 */
void Conexcao::Enviar(){
	//	for (const std::string& word : msg)

}

/*
 * quando conexcao acabar, chamar funcao para fechar socket
 */
void Conexcao::Terminar(){

}
