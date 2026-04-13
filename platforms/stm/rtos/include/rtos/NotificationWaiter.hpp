#pragma once

#include "FreeRTOS.h"
#include "error/freertos_errors.hpp"
#include "interfaces/rtos/INotificationWaiter.hpp"
#include "task.h"
#include "types/types.hpp"

namespace rtos
{

class NotificationWaiter
{
   static_assert(std::is_unsigned_v<types::EventBitsType>);
   static_assert(std::is_trivially_copyable_v<types::EventBitsType>);
   static_assert(sizeof(uint32_t) <= sizeof(types::EventBitsType));

public:
   static types::EventBitsType wait(const uint32_t wait_duration_ms)
   {
      uint32_t raw_bits = 0;

      const BaseType_t result = xTaskNotifyWait(0, UINT32_MAX, &raw_bits, pdMS_TO_TICKS(wait_duration_ms));
      if (result == pdTRUE)
      {
         return static_cast<types::EventBitsType>(raw_bits);
      }

      return 0u;
   }
};

static_assert(interfaces::rtos::INotificationWaiter<NotificationWaiter>);

}   // namespace rtos
