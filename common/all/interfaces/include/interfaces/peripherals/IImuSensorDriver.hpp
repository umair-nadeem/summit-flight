#pragma once

#include <concepts>

namespace interfaces::peripherals
{

template <typename C>
concept IImuSensorDriver = requires(C c) {
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

   {
      c.notify_receive_complete()
   }
   -> std::same_as<void>;
};

}   // namespace interfaces::peripherals
