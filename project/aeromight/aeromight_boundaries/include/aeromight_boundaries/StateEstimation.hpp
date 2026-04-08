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

   math::Vector3 raw_accel_mps2{};
   math::Vector3 raw_gyro_radps{};
   math::Vector3 gyro_bias{};

   math::Euler euler{};

   // altitude
   float altitude;
   float vertical_velocity;
};

}   // namespace aeromight_boundaries
