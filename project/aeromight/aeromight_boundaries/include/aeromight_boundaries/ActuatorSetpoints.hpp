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
   ActuatorSetpoints setpoints{};
   bool              enabled{false};
};

namespace control_axis
{

enum ControlAxis : uint8_t
{
   roll = 0,
   pitch,
   yaw,
   thrust
};

}

}   // namespace aeromight_boundaries
