#pragma once

#include <concepts>
#include <optional>

namespace interfaces::rtos
{

template <typename C, typename T>
concept INotificationWaiter = requires(C c, T t, const uint32_t s) {
   {
      c.wait(s)
   }
   -> std::same_as<std::optional<T>>;
};

}   // namespace interfaces::rtos
