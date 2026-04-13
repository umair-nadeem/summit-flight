#pragma once

#include <cassert>

#include "FreeRTOS.h"
#include "error/error_handler.hpp"
#include "task.h"

namespace rtos
{

template <typename PeriodicTask>
[[noreturn]] void run_periodic_task(PeriodicTask& periodic_task)
{
   const uint32_t period_in_ms     = periodic_task.get_period_ms();
   TickType_t     last_wakeup_time = xTaskGetTickCount();
   BaseType_t     was_delayed;

   while (true)
   {
      periodic_task.run_once();

      // was_delayed will be false if the next wakeup time is in past
      was_delayed = xTaskDelayUntil(&last_wakeup_time, pdMS_TO_TICKS(period_in_ms));
      error::verify(was_delayed == pdTRUE);
   }
}

}   // namespace rtos
