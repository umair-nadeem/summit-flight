#pragma once

namespace interfaces::hw
{

template <typename C>
concept IDigitalOutput = requires(C c) {
   {
      c.set_high()
   }
   -> std::same_as<void>;

   {
      c.set_low()
   }
   -> std::same_as<void>;

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
