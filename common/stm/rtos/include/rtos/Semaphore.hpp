#pragma once

#include "FreeRTOS.h"
#include "error/freertos_errors.hpp"
#include "semphr.h"

namespace rtos
{

struct Semaphore
{
   bool              created{false};
   StaticSemaphore_t semaphore_buffer{};
   SemaphoreHandle_t semaphore_handle{nullptr};

   [[nodiscard]] SemaphoreHandle_t create() noexcept
   {
      if (!created)
      {
         semaphore_handle = xSemaphoreCreateBinaryStatic(&semaphore_buffer);
         error::freertos_assert(semaphore_handle != nullptr);
         created = true;
      }
      return semaphore_handle;
   }

   [[nodiscard]] SemaphoreHandle_t get_handle() const
   {
      return semaphore_handle;
   }
};

}   // namespace rtos
