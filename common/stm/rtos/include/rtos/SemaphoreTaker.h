#pragma once

#include "FreeRTOS.h"
#include "error/error_handler.h"
#include "error/freertos_errors.h"
#include "semphr.h"

namespace rtos
{

class SemaphoreTaker
{
public:
   void take();
   bool take_if_possible();
   bool take_from_isr(const bool higher_priority_task_woken);

   void set_handle(SemaphoreHandle_t semaphore_handle)
   {
      error::verify(semaphore_handle != nullptr);
      m_handle = semaphore_handle;
   }

private:
   SemaphoreHandle_t m_handle{nullptr};
};

}   // namespace rtos
