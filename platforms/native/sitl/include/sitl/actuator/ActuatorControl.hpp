#pragma once

#include <array>

namespace sitl::actuator
{

template <std::size_t N>
struct ActuatorControl
{
   std::array<float, N> m_actuator_values{};

   void update(const std::array<float, N>& values, [[maybe_unused]] const bool telemetry)
   {
      m_actuator_values = values;
   }
};

}   // namespace sitl::actuator
