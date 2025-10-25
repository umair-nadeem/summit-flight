#pragma once

#include <concepts>

namespace interfaces::rtos
{

template <typename C>
concept INotifier = requires(C c, const bool b) {
   {
      c.notify()
   }
   -> std::same_as<void>;

   {
      c.notify_from_isr(b)
   }
   -> std::same_as<void>;
};

}   // namespace interfaces::rtos
