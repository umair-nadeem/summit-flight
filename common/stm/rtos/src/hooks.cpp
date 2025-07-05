#include <cassert>

#include "FreeRTOS.h"
#include "task.h"

extern "C"
{

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
   (void)xTask;
   (void)pcTaskName;

   assert(false);
}

}
