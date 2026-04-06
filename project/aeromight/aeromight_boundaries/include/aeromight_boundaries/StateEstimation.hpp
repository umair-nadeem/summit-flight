#pragma once

#include <cstdint>

#include "math/Euler.hpp"
#include "math/Quaternion.hpp"

namespace aeromight_boundaries
{

struct StateEstimation
{
   uint32_t timestamp_ms{};
   bool     attitude_estimation_valid{};
   bool     altitude_estimation_valid{};

   math::Quaternion attitude{};
   math::Vector3    accel_mps2{};
   math::Vector3    gyro_radps{};
   math::Vector3    gyro_bias{};

   math::Euler euler{};

   // altitude
   float altitude;
   float vertical_velocity;

   void reset_attitude_state()
   {
      attitude.zero();
      accel_mps2.zero();
      gyro_bias.zero();
      gyro_bias.zero();
   }

   void reset_altitude_state()
   {
      altitude          = 0.0f;
      vertical_velocity = 0.0f;
   }
};

}   // namespace aeromight_boundaries
