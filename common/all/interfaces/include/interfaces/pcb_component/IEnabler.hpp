#pragma once

namespace interfaces::pcb_component
{

template <typename C>
concept IEnabler = requires(C c) {
   {
      c.enable()
   }
   -> std::same_as<void>;

   {
      c.disable()
   }
   -> std::same_as<void>;

   {
      c.is_enabled()
   }
   -> std::same_as<bool>;
};

}   // namespace interfaces::pcb_component
