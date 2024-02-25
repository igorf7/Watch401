#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "stm32f4xx.h"
#include <stddef.h>

#define TASK_QUEUE_SIZE 8U

/* Structure of the task queue item */
typedef struct {
    void (*taskfunc)(void*); // task function pointer
    void *argument; // task function argument
    void *next; // pointer to the next item in the queue
} QueueItem_t;

/*** TASK SCHEDULER API ***/
void InitTaskQueue(void (*callback)(void));
void PutEvent(void (*task)(void*), void *arg);
void RunTaskSheduler(void);
#endif // __SCHEDULER_H
