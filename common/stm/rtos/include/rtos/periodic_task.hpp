#pragma once

#include <cassert>

#include "FreeRTOS.h"
#include "task.h"

namespace rtos
{

template <typename PeriodicTask>
[[noreturn]] void run_periodic_task(PeriodicTask& periodic_task)
{
   const uint32_t period_in_ms     = periodic_task.get_period_ms();
   TickType_t     last_wakeup_time = xTaskGetTickCount();
   BaseType_t     was_delayed;
   UBaseType_t    stack_water_mark;

   while (true)
   {
      stack_water_mark = uxTaskGetStackHighWaterMark(NULL);
      if (stack_water_mark > 1)
      {
         //
      }

      periodic_task.run_once();

      // was_delayed will be false if the next wakeup time is in past
      was_delayed = xTaskDelayUntil(&last_wakeup_time, pdMS_TO_TICKS(period_in_ms));
      error::verify(was_delayed == pdTRUE);
   }
}

}   // namespace rtos
