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
   UBaseType_t       stack_water_mark;

   while (true)
   {
      stack_water_mark = uxTaskGetStackHighWaterMark(NULL);
      if (stack_water_mark > 1)
      {
         //
      }

      periodicTask.run_once();

      xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(period_in_ms));
      error::verify(xWasDelayed == pdTRUE);
   }
}

}   // namespace rtos
