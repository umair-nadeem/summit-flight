#pragma once

namespace interfaces::hw
{

template <typename C>
concept IDigitalInput = requires(C c) {
   {
      c.is_high()
   }
   -> std::same_as<bool>;

   {
      c.is_low()
   }
   -> std::same_as<bool>;
};

}   // namespace interfaces::hw
