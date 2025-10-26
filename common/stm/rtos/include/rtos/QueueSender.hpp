#pragma once

#include "FreeRTOS.h"
#include "error/error_handler.hpp"
#include "error/freertos_errors.hpp"
#include "interfaces/rtos/IQueueSender.hpp"
#include "queue.h"

namespace rtos
{

template <typename T>
class QueueSender
{
public:
   void send_blocking(const T& element)
   {
      BaseType_t result;
      do
      {
         result = xQueueSend(m_handle, &element, portMAX_DELAY);
      } while (result == errQUEUE_FULL);
      error::freertos_assert(result == pdPASS);
   }

   [[nodiscard]] bool send_if_possible(const T& element)
   {
      const BaseType_t result = xQueueSend(m_handle, &element, 0);
      return (result == pdPASS);
   }

   [[nodiscard]] bool send_from_isr(const T& element)
   {
      BaseType_t       task_woken = pdFALSE;
      const BaseType_t result     = xQueueSendFromISR(m_handle, &element, task_woken);
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

static_assert(interfaces::rtos::IQueueSender<QueueSender<int>, int>);

}   // namespace rtos
