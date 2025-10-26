#pragma once

#include <concepts>

namespace interfaces::rtos
{

template <typename C>
concept INotifier = requires(C c) {
   {
      c.notify()
   }
   -> std::same_as<void>;

   {
      c.notify_from_isr()
   }
   -> std::same_as<void>;
};

}   // namespace interfaces::rtos
