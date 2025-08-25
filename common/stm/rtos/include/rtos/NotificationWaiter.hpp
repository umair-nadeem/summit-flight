#pragma once

#include "FreeRTOS.h"
#include "error/freertos_errors.hpp"
#include "interfaces/rtos/INotificationWaiter.hpp"
#include "task.h"

namespace rtos
{

template <typename EventFlags>
class NotificationWaiter
{
public:
   explicit NotificationWaiter()
   {
      static_assert(sizeof(EventFlags) == sizeof(uint32_t));
      static_assert(std::is_trivially_copyable_v<EventFlags>);
   }

   static std::optional<EventFlags> wait(const std::size_t wait_duration_ms)
   {
      EventFlags flags = 0;

      const BaseType_t result = xTaskNotifyWait(0, UINT32_MAX, reinterpret_cast<uint32_t*>(&flags), pdMS_TO_TICKS(wait_duration_ms));
      if (result == pdTRUE)
      {
         return flags;
      }
      return std::nullopt;
   }

   void set_handle(TaskHandle_t handle)
   {
      error::freertos_assert(handle != nullptr);
      m_task_handle = handle;
   }

private:
   TaskHandle_t m_task_handle{nullptr};
};

static_assert(interfaces::rtos::INotificationWaiter<NotificationWaiter<int>, int>);

}   // namespace rtos
