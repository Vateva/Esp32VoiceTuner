#ifndef TASKS_H
#define TASKS_H

#include "config.h"

// freertos task functions
void audioTask(void* parameter);
void processingAndDisplayTask(void* parameter);

#endif // TASKS_H