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
   static_assert(sizeof(EventFlags) == sizeof(uint32_t));
   static_assert(std::is_trivially_copyable_v<EventFlags>);

public:
   static std::optional<EventFlags> wait(const uint32_t wait_duration_ms)
   {
      uint32_t raw_flags = 0;

      const BaseType_t result = xTaskNotifyWait(0, UINT32_MAX, &raw_flags, pdMS_TO_TICKS(wait_duration_ms));
      if (result == pdTRUE)
      {
         return std::bit_cast<EventFlags>(raw_flags);
      }
      return std::nullopt;
   }
};

static_assert(interfaces::rtos::INotificationWaiter<NotificationWaiter<int>, int>);

}   // namespace rtos
