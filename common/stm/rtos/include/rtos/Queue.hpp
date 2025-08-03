#pragma once

#include "error/freertos_errors.h"
#include "queue.h"

namespace rtos
{

template <std::size_t QueueLength, std::size_t ItemSize>
struct Queue
{
   static constexpr std::size_t storage_buffer_size = QueueLength * ItemSize;

   StaticQueue_t queue_buffer{};
   QueueHandle_t queue_handle{nullptr};
   alignas(std::max_align_t) std::byte storage_buffer[storage_buffer_size];

   void create()
   {
      queue_handle = xQueueCreateStatic(QueueLength, ItemSize, reinterpret_cast<uint8_t*>(storage_buffer), &queue_buffer);
      error::freertos_assert(queue_handle != nullptr);
   }

   [[nodiscard]] QueueHandle_t get_handle() const
   {
      return queue_handle;
   }
};

}   // namespace rtos
