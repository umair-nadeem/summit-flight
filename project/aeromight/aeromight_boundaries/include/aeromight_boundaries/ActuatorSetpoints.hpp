#pragma once

namespace aeromight_boundaries
{

struct ActuatorParams
{
   // absoluate actuator limits
   static constexpr float       min           = 0.0f;
   static constexpr float       max           = 1.0f;
   static constexpr std::size_t num_actuators = 4u;
};

struct ActuatorSetpoints
{
   float m1{};
   float m2{};
   float m3{};
   float m4{};
};

struct ActuatorControl
{
   ActuatorSetpoints setpoints{};
   bool              enabled{false};
};

}   // namespace aeromight_boundaries
