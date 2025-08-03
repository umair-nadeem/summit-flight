#pragma once

#include "FreeRTOS.h"
#include "error/freertos_errors.h"
#include "semphr.h"

namespace rtos
{

struct Semaphore
{
   StaticSemaphore_t semaphore_buffer{};
   SemaphoreHandle_t semaphore_handle{nullptr};

   [[nodiscard]] SemaphoreHandle_t create()
   {
      semaphore_handle = xSemaphoreCreateBinaryStatic(&semaphore_buffer);
      error::freertos_assert(semaphore_handle != nullptr);
      return semaphore_handle;
   }

   [[nodiscard]] SemaphoreHandle_t get_handle() const
   {
      return semaphore_handle;
   }
};

}   // namespace rtos
