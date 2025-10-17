#pragma once

namespace aeromight_control
{

struct EkfState
{
   float z{};
   float v_z{};
   float a_z_bias{};
};

}   // namespace aeromight_control
