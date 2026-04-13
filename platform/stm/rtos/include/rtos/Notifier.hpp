#pragma once

#include <cstring>
#include <type_traits>

#include "FreeRTOS.h"
#include "error/freertos_errors.hpp"
#include "task.h"
#include "types/types.hpp"
#include "utilities/enum_to_bit_mask.hpp"

namespace rtos
{

class Notifier
{
   static_assert(std::is_unsigned_v<types::EventBitsType>);
   static_assert(std::is_trivially_copyable_v<types::EventBitsType>);
   static_assert(sizeof(uint32_t) <= sizeof(types::EventBitsType));

public:
   explicit Notifier(const types::EventBitsType bits)
       : m_bits{bits}
   {
   }

   void notify()
   {
      const BaseType_t result = xTaskNotify(m_task_handle, m_bits, eSetBits);
      error::freertos_assert(result == pdTRUE);
   }

   void notify_from_isr()
   {
      BaseType_t       task_woken = pdFALSE;
      const BaseType_t result     = xTaskNotifyFromISR(m_task_handle, m_bits, eSetBits, &task_woken);
      portYIELD_FROM_ISR(task_woken);
      error::freertos_assert(result == pdTRUE);
   }

   void set_task_to_notify(TaskHandle_t handle)
   {
      error::freertos_assert(handle != nullptr);
      m_task_handle = handle;
   }

private:
   types::EventBitsType m_bits;
   TaskHandle_t         m_task_handle{nullptr};
};

}   // namespace rtos
