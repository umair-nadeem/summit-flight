#pragma once

namespace aeromight_control
{

struct EkfState
{
   float altitude{};
   float vertical_velocity{};
   float accel_z_bias{};
};

}   // namespace aeromight_control
