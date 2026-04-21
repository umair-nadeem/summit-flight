#pragma once

namespace aeromight_control
{

struct ControlAllocatorParams
{
   float thrust_limiting             = 0.3f;
   float actuator_min                = 0.0f;
   float actuator_max                = 1.0f - thrust_limiting;
   float actuator_idle               = 0.1f;
   float thrust_deadband             = 0.01f;
   float yaw_saturation_limit_factor = 0.25f;
   float slew_rate_limit_s           = 1.0f;
};

}   // namespace aeromight_control
