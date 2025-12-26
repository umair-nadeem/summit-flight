#pragma once

#include <cmath>

#include "Euler.hpp"
#include "Quaternion.hpp"

namespace math
{

inline Euler quaternion_to_euler(const Quaternion& q)
{
   // Roll (rotation around X axis)
   const float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
   const float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);

   // Pitch (rotation around Y axis)
   const float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);

   // Yaw (rotation around Z axis)
   const float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
   const float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);

   const float roll  = std::atan2(sinr_cosp, cosr_cosp);
   const float pitch = (std::fabs(sinp) >= 1.0f) ? std::copysignf(physics::constants::pi_by_2, sinp) : std::asin(sinp);
   const float yaw   = std::atan2(siny_cosp, cosy_cosp);

   return Euler{roll, pitch, yaw};
}

}   // namespace math
