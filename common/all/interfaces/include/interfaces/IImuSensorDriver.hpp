#pragma once

namespace interfaces
{

template <typename C>
concept IImuSensorDriver = requires(C c) {
   {
      c.execute()
   }
   -> std::same_as<void>;
};

}   // namespace interfaces
