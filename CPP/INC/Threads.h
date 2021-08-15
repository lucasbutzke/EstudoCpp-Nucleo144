/*
 * Get threads creation parameters/mutex/semaphores
 * Estudar como gerenciar MAILBOXES
 */

#ifndef THREADS_H_
#define THREADS_H_

#include "cmsis_os.h"

class Threads {
public:
	TaskHandle_t TaskHandlex; // handler for Task1

	// create task and start already
	Threads(TaskFunction_t, const char * const, TaskHandle_t*);
	virtual ~Threads();

	// function call on object construction
	void cppDelay(int);
	void cppDelete();
};

#endif /* THREADS_H_ */
