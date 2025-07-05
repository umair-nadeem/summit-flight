#pragma once

#include <cassert>

#include "FreeRTOS.h"
#include "task.h"

namespace rtos
{

template <typename PeriodicTask>
[[noreturn]] void run_periodic_task(PeriodicTask& periodicTask)
{
   const size_t period_in_ms  = periodicTask.get_period_ms();
   TickType_t   xLastWakeTime = xTaskGetTickCount();
   BaseType_t   xWasDelayed;

   while (true)
   {
      periodicTask.run_once();

      xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(period_in_ms));
      assert(xWasDelayed == pdTRUE);
   }
}

}   // namespace rtos
