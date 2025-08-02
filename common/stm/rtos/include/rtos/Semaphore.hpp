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

   [[nodiscard]] SemaphoreHandle_t create(const bool initialize_as_given)
   {
      semaphore_handle = xSemaphoreCreateBinaryStatic(&semaphore_buffer);
      error::freertos_assert(semaphore_handle != nullptr);

      if (initialize_as_given)
      {
         error::freertos_assert(xSemaphoreGive(semaphore_handle) == pdPASS);
      }
      return semaphore_handle;
   }

   [[nodiscard]] SemaphoreHandle_t get_handle() const
   {
      return semaphore_handle;
   }
};

}   // namespace rtos
