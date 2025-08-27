#pragma once

#include <concepts>
#include <cstdint>

namespace interfaces
{

template <typename T>
concept IClockSource = requires {
   {
      T::now_ms()
   }
   -> std::same_as<uint32_t>;

   {
      T::now_s()
   }
   -> std::same_as<uint32_t>;
};

}   // namespace interfaces
