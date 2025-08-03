#pragma once

#include <cassert>

#include "FreeRTOS.h"
#include "task.h"

namespace rtos
{

template <typename PeriodicTask>
[[noreturn]] void run_periodic_task(PeriodicTask& periodicTask)
{
   const std::size_t period_in_ms  = periodicTask.get_period_ms();
   TickType_t        xLastWakeTime = xTaskGetTickCount();
   BaseType_t        xWasDelayed;
   UBaseType_t       high_water;

   while (true)
   {
      high_water = uxTaskGetStackHighWaterMark(NULL);
      if (high_water > 1)
      {
         //
      }

      periodicTask.run_once();

      xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(period_in_ms));
      error::verify(xWasDelayed == pdTRUE);
   }
}

}   // namespace rtos
