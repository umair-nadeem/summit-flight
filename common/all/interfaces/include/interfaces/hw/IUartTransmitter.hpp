#pragma once

#include <cstdint>

namespace interfaces::hw
{

template <typename C>
concept IUartTransmitter = requires(C c, const uint32_t s) {
   {
      c.send_blocking(s)
   }
   -> std::same_as<void>;

   {
      c.send_and_return(s)
   }
   -> std::same_as<void>;

   {
      c.get_buffer()
   }
   -> std::same_as<std::span<std::byte>>;
};

}   // namespace interfaces::hw
