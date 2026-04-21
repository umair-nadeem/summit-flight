#pragma once

#include "math/Vector3.hpp"

namespace control::rate
{

struct RateControllerParams
{
   // p
   math::Vec3f gains_p{0.035f, 0.035f, 0.06f};

   // i
   math::Vec3f gains_i{0.025f, 0.025f, 0.015f};

   // d
   math::Vec3f gains_d{0.002f, 0.002f, 0.0f};

   // ff
   math::Vec3f gains_ff{0.008f, 0.008f, 0.006f};

   // torque
   float torque_output_limit = 1.0f;

   // integrator limit
   math::Vec3f integrator_limit{0.2f, 0.2f, 0.03f};
};

}   // namespace control::rate
