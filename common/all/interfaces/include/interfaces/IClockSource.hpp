#pragma once

namespace interfaces
{

template <typename T>
concept IClockSource = requires {
   {
      T::now_us()
   }
   -> std::same_as<uint64_t>;

   {
      T::now_ms()
   }
   -> std::same_as<uint64_t>;

   {
      T::now_s()
   }
   -> std::same_as<uint64_t>;
};

}   // namespace interfaces
