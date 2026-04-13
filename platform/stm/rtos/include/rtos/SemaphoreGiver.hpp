#pragma once

#include "FreeRTOS.h"
#include "error/error_handler.hpp"
#include "error/freertos_errors.hpp"
#include "semphr.h"

namespace rtos
{

class SemaphoreGiver
{
public:
   void give();
   bool give_from_isr();

   void set_handle(SemaphoreHandle_t semaphore_handle)
   {
      error::verify(semaphore_handle != nullptr);
      m_handle = semaphore_handle;
   }

private:
   SemaphoreHandle_t m_handle{nullptr};
};

}   // namespace rtos
