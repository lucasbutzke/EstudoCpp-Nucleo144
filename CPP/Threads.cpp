/*
 * Criacao e manipulacao das threads dentro do codigo,
 * verificar os Ticks e interrupcoes.
 */

#include "Threads.h"

/*
 * Create default task
 */
Threads::Threads(TaskFunction_t taskFunc,
		const char * const Tag,
		TaskHandle_t* taskHandler)
{
	// TODO Auto-generated constructor stub
	// receber os dados da thread e
	// retornar que TASK foi construida atraves do USART
	TaskHandlex = taskHandler;
	xTaskCreate(taskFunc, Tag, 100, (void*) 1, 1, &TaskHandlex);

}

/*
 * Show that TASK has been destroyed and verify process attached
 */
Threads::~Threads() {
	// TODO Auto-generated destructor stub
	// retornar que TASK foi destruida atraves do USART

}


/*
 * Funcao de FreeRTOS delay para segundos
 */
void Threads::cppDelay(int x){
	// osDelay(x);	// Delay para todo o sistema
	// Delay para a tarefa com "x" ms
	TickType_t xDelay = x / portTICK_PERIOD_MS;

	vTaskDelay(xDelay);
}

void Threads::cppDelete(){

}

