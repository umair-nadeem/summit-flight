#pragma once

#include <optional>

#include "FreeRTOS.h"
#include "error/error_handler.hpp"
#include "error/freertos_errors.hpp"
#include "interfaces/rtos/IQueueReceiver.hpp"
#include "queue.h"

namespace rtos
{

template <typename T>
class QueueReceiver
{
public:
   T receive_blocking()
   {
      T                data;
      const BaseType_t result = xQueueReceive(m_handle, &data, portMAX_DELAY);

      error::freertos_assert(result == pdPASS);
      return data;
   }

   [[nodiscard]] std::optional<T> receive_if_available()
   {
      T                data;
      const BaseType_t result = xQueueReceive(m_handle, &data, 0);

      if (result == pdPASS)
      {
         return data;
      }

      return std::nullopt;
   }

   /// @brief Drain the queue and return the most recent message if any.
   /// @note  Intended for slow consumers that only need the latest sample.
   ///        Prevents queue overflow by discarding older messages.
   [[nodiscard]] std::optional<T> receive_latest()
   {
      bool received = false;
      T    data;

      while (xQueueReceive(m_handle, &data, 0) == pdPASS)
      {
         received = true;
      }

      if (received)
      {
         return data;
      }

      return std::nullopt;
   }

   [[nodiscard]] bool receive_from_isr(T& data)
   {
      BaseType_t       task_woken = pdFALSE;
      const BaseType_t result     = xQueueReceiveFromISR(m_handle, &data, &task_woken);
      portYIELD_FROM_ISR(task_woken);
      return (result == pdPASS);
   }

   void set_handle(QueueHandle_t queue_handle)
   {
      error::verify(queue_handle != nullptr);
      m_handle = queue_handle;
   }

private:
   QueueHandle_t m_handle{nullptr};
};

static_assert(interfaces::rtos::IQueueReceiver<QueueReceiver<int>, int>);

}   // namespace rtos
