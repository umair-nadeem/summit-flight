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

   math::Vec3f raw_accel_mps2{};
   math::Vec3f raw_gyro_radps{};
   math::Vec3f gyro_bias{};

   math::Euler euler{};

   // altitude
   float altitude;
   float vertical_velocity;
};

}   // namespace aeromight_boundaries
