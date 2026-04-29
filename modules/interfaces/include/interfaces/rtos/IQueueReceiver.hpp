#pragma once

#include <optional>

namespace interfaces::rtos
{

template <typename C, typename T>
concept IQueueReceiver = requires(C c, T& t) {
   {
      c.receive_blocking()
   }
   -> std::same_as<T>;

   {
      c.receive_if_available()
   }
   -> std::same_as<std::optional<T>>;
};

}   // namespace interfaces::rtos
