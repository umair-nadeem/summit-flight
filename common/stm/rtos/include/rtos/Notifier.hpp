#pragma once

#include <cstring>

#include "FreeRTOS.h"
#include "error/freertos_errors.hpp"
#include "task.h"

namespace rtos
{

template <typename EventFlags>
class Notifier
{
   static_assert(sizeof(EventFlags) == sizeof(uint32_t));
   static_assert(std::is_trivially_copyable_v<EventFlags>);

public:
   explicit Notifier(const EventFlags& flag)
   {
      m_flag = flag.to_ulong();
   }

   void notify()
   {
      const BaseType_t result = xTaskNotify(m_task_handle, m_flag, eSetBits);
      error::freertos_assert(result == pdTRUE);
   }

   void notify_from_isr()
   {
      BaseType_t       task_woken = pdFALSE;
      const BaseType_t result     = xTaskNotifyFromISR(m_task_handle, m_flag, eSetBits, &task_woken);
      portYIELD_FROM_ISR(task_woken);
      error::freertos_assert(result == pdTRUE);
   }

   void set_task_to_notify(TaskHandle_t handle)
   {
      error::freertos_assert(handle != nullptr);
      m_task_handle = handle;
   }

private:
   uint32_t     m_flag{0};
   TaskHandle_t m_task_handle{nullptr};
};

}   // namespace rtos
