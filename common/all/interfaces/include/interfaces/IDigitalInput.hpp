#pragma once

namespace interfaces
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

}   // namespace interfaces
