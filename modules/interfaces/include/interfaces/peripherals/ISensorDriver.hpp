#pragma once

#include <concepts>

namespace interfaces::peripherals
{

template <typename C>
concept ISensorDriver = requires(C c) {
   {
      c.start()
   }
   -> std::same_as<void>;

   {
      c.stop()
   }
   -> std::same_as<void>;

   {
      c.execute()
   }
   -> std::same_as<void>;
};

}   // namespace interfaces::peripherals
