#pragma once

#include "error/freertos_errors.h"
#include "queue.h"

namespace rtos
{

struct Queue
{
   StaticQueue_t queue_buffer{};
   QueueHandle_t queue_handle{nullptr};

   void create(const size_t queue_len, const size_t item_size, uint8_t* storage_buffer)
   {
      queue_handle = xQueueCreateStatic(queue_len, item_size, storage_buffer, &queue_buffer);
      error::freertos_assert(queue_handle != nullptr);
   }

   [[nodiscard]] QueueHandle_t get_handle() const
   {
      return queue_handle;
   }
};

}   // namespace rtos
