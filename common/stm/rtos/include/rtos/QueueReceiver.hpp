#pragma once

#include <optional>

#include "FreeRTOS.h"
#include "error/error_handler.hpp"
#include "error/freertos_errors.hpp"
#include "interfaces/IQueueReceiver.hpp"
#include "queue.h"

namespace rtos
{

template <typename T>
class QueueReceiver
{
public:
   T receive_blocking()
   {
      T          data;
      BaseType_t result;
      do
      {
         result = xQueueReceive(m_handle, &data, portMAX_DELAY);
      } while (result == errQUEUE_EMPTY);
      error::freertos_assert(result == pdPASS);

      return data;
   }

   std::optional<T> receive_if_available()
   {
      T          data;
      BaseType_t result = xQueueReceive(m_handle, &data, 0);

      if (result == pdPASS)
      {
         return data;
      }

      return std::nullopt;
   }

   [[nodiscard]] bool receive_from_isr(T& data, const bool higher_priority_task_woken)
   {
      BaseType_t task_woken = (higher_priority_task_woken ? pdTRUE : pdFALSE);
      BaseType_t result     = xQueueReceiveFromISR(m_handle, &data, &task_woken);
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

static_assert(interfaces::IQueueReceiver<QueueReceiver<int>, int>);

}   // namespace rtos
