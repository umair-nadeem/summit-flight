#pragma once

#include <concepts>

namespace interfaces::rtos
{

template <typename C, typename T>
concept IQueueSender = requires(C c, const T t) {
   {
      c.send_blocking(t)
   }
   -> std::same_as<void>;

   {
      c.send_if_possible(t)
   }
   -> std::same_as<bool>;

   {
      c.send_from_isr(t)
   }
   -> std::same_as<bool>;
};

}   // namespace interfaces::rtos
