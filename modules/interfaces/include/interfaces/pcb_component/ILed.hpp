#pragma once

#include <concepts>

namespace interfaces::pcb_component
{

template <typename C>
concept ILed = requires(C c) {
   {
      c.turn_on()
   }
   -> std::same_as<void>;

   {
      c.turn_off()
   }
   -> std::same_as<void>;
};

}   // namespace interfaces::pcb_component
