#pragma once

#include "math/constants.hpp"

namespace control
{

class Motor
{
public:
   template <typename Vec>
   static void apply_thrust_linearization(Vec& motor_outputs, const float linearization_factor, const float motor_min, const float motor_max)
   {
      if ((math::constants::epsilon < linearization_factor) && (linearization_factor <= 1.0f))
      {
         // thrust = factor * x^2 + (1 - factor) * x
         const float a     = linearization_factor;
         const float b     = 1.0f - linearization_factor;
         const float inv_a = 1.0f / a;

         const float tmp1 = b / (2.0f * a);
         const float tmp2 = b * b / (4.0f * a * a);

         for (std::size_t i = 0; i < Vec::size; i++)
         {
            const float thrust = motor_outputs[i];
            motor_outputs[i]   = std::clamp(-tmp1 + std::sqrtf(tmp2 + (thrust * inv_a)), motor_min, motor_max);
         }
      }
   }
};

}   // namespace control
