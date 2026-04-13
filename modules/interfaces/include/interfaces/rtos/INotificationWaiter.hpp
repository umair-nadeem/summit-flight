#pragma once

#include <concepts>
#include <cstdint>

namespace interfaces::rtos
{

template <typename C>
concept INotificationWaiter = requires(C c, const uint32_t timeout_ms) {
   {
      c.wait(timeout_ms)
   }
   -> std::same_as<uint32_t>;
};

}   // namespace interfaces::rtos
