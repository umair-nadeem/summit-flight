#pragma once

namespace aeromight_estimation
{

struct EkfState
{
   float z{};          // altitude
   float v_z{};        // vertical_velocity
   float a_z_bias{};   // accelerometer z-axis bias
};

}   // namespace aeromight_estimation
