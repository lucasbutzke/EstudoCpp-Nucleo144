/*
 * realizar tarefas FreeRTOS em C++ (ou colocando em C e chamando
 * depois), chamar funcao math_arm e realizar um codigo para controle
 * do EIT. Verificar os limites do hardware e velocidade de envio
 * dos dados para otimizacao.
 *
 * Tentar utilizar o ADC, interrupcoes e USART (o q mais tiver)
 * para estudar e empregar em um futuro codigo. Fazer protitipo
 * FreeRTOS dos perifericos para prova de conceito do sistema,
 * enquanto tambem se escreve o codigo para comunicar, plotar,
 * salvar no computador.
 */
// TODO Receber dados continuamente e depois um seno matematico do stm
// TODO Pegar dados de temperatura ou ADC/DAC
// TODO Fazer DMA para todos realizarem suas tarefas mais rapidamente possivel
// TODO e receber pelo USART no windows (Arduino plot inicialmente)

#include "main.h"
#include "string.h"
#include "BTN.h"
#include "LED.h"
#include "Threads.h"
#include "Conexcao.h"
#include "MathFuns.h"
#include <iostream>     // std::cout
#include <cstddef>      // std::size_t
#include <valarray>     // std::valarray

// verificar o pq da necessidade os prototipos
// das funcoes devem manter este modelo ( VOID )
void vUpdateMailbox( void *pvParameters );
void vReadMailbox( void *pvParameters );

// enviar de uma tarefa a outra um objeto com os dados necessarios para comunicar
typedef struct xExampleStructure{
	  TickType_t xTimeStamp = 0;		// importante para debuggar pelo Serial
	  uint32_t ulValue = 0;				// valor ADC para se passar
} Example_t;

// criar MAILBOX unica para todas a threads em execucao
QueueHandle_t xMailbox;
TaskHandle_t Tarefa;
TaskHandle_t MandarMsg;

// GPIOs importantes para mostrar funcionamento
BTN BTN1(USER_Btn_GPIO_Port, USER_Btn_Pin);
LED LED1(LD2_GPIO_Port, LD2_Pin);
LED LED2(LD3_GPIO_Port, LD3_Pin);
LED LED3(LD1_GPIO_Port, LD1_Pin);

// variaveis funcoes
uint32_t ulNewValue = 0;
TickType_t xDelay = 500 / portTICK_PERIOD_MS;	// delay for 500ms

/*
 * criar task de conexcao com o servidor e executar as funcoes
 * paralelamente a todas as funcoes do algoritmo
 */
void CppMain()
{
	xMailbox = xQueueCreate( 1, sizeof( Example_t ) );

	// Class to create and start tasks schedule
	Threads Task1(vUpdateMailbox, "Maker", &Tarefa); // handler for Task1
	Threads Task2(vReadMailbox, "Sender", &MandarMsg);; // handler for Task2
	// Estudar novamente o codigo e testar novas possibilidades de adicionar
	// USART E ADC/DAC

	// Tanto para a criacao de tarefas no metodo tradicional quanto para
	// criacao dentro da classe esta com problemas, verificar como resolver
	// o erro. O algoritmo funciona no FREERTOS do Arduino.
	//xTaskCreate(vUpdateMailbox, "Maker", 100, (void*) 1, 1, &Tarefa);
	//xTaskCreate(vReadMailbox, "Sender", 100, (void*) 1, 1, &MandarMsg);
	/* arrumado mas deve ser testado e melhorado*/

	while(true){
		LED1.Toogle();
		LED2.Toogle();

		vTaskDelay(xDelay);
	}

}

/*
 * Pegar o valor desejado para salvar no buffer
 * para ser enviado assim que for possivel
 */
void vUpdateMailbox( void *pvParameters ){
	/* Example_t was defined in Listing 67. */
	Example_t xData;

	/* Write the new data into the Example_t structure.*/
	xData.ulValue = ulNewValue;
	ulNewValue++;

	/* Use the RTOS tick count as the time stamp stored in the Example_t structure. */
	xData.xTimeStamp = xTaskGetTickCount();

	/* Send the structure to the mailbox - overwriting any data that is already in the
	mailbox. */
	if (BTN1.Read() != 0){
		xQueueOverwrite( xMailbox, &xData );
	}

	if (ulNewValue > 100){
		ulNewValue = 0;
	}
	vTaskDelay(xDelay);
}

/*
 * Receber pela MAILBOX e enviar o valor pelo Serial.
 * Tentar modificar funcao para retornar BOOLEAN.
 */
void vReadMailbox( void * pvParameters )
{
	Example_t *pxData;
	TickType_t xPreviousTimeStamp;
	BaseType_t xDataUpdated;

	/* This function updates an Example_t structure with the latest value received
	from the mailbox. Record the time stamp already contained in *pxData before it
	gets overwritten by the new data. */
	xPreviousTimeStamp = pxData->xTimeStamp;

	/* Update the Example_t structure pointed to by pxData with the data contained in
	the mailbox. If xQueueReceive() was used here then the mailbox would be left
	empty, and the data could not then be read by any other tasks. Using
	xQueuePeek() instead of xQueueReceive() ensures the data remains in the mailbox.
	A block time is specified, so the calling task will be placed in the Blocked
	state to wait for the mailbox to contain data should the mailbox be empty. An
	infinite block time is used, so it is not necessary to check the value returned
	from xQueuePeek(), as xQueuePeek() will only return when data is available. */
	xQueuePeek( xMailbox, pxData, portMAX_DELAY );

	/* Return pdTRUE if the value read from the mailbox has been updated since this
	function was last called. Otherwise return pdFALSE. */
	if( pxData->xTimeStamp > xPreviousTimeStamp ){
			xDataUpdated = pdTRUE;
			LED1.Toogle();
			LED2.Toogle();

			vTaskDelay(xDelay);
	}else
	{
			xDataUpdated = pdFALSE;
			LED3.Toogle();
			vTaskDelay(xDelay);
	}

	//return xDataUpdated;
}

