#pragma once

#include "math/Vector4.hpp"

namespace aeromight_boundaries
{

using ActuatorSetpoints = math::Vector4;

struct ActuatorParams
{
   // absoluate actuator limits
   static constexpr float       min           = 0.0f;
   static constexpr float       max           = 1.0f;
   static constexpr std::size_t num_actuators = 4u;
};

struct ActuatorControl
{
   ActuatorSetpoints setpoints{0.0f};
   bool              enabled{false};
};

struct ControlAxis
{
   static constexpr uint8_t roll   = 0;
   static constexpr uint8_t pitch  = 1u;
   static constexpr uint8_t yaw    = 2u;
   static constexpr uint8_t thrust = 3u;
};

}   // namespace aeromight_boundaries
