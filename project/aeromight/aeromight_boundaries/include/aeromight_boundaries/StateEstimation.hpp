#pragma once

#include <cstdint>

#include "math/Euler.hpp"
#include "math/Quaternion.hpp"

namespace aeromight_boundaries
{

struct StateEstimation
{
   uint32_t timestamp_ms{};

   math::Quaternion attitude{};
   math::Vector3    gyro_radps{};
   math::Vector3    gyro_bias{};

   math::Euler euler{};

   // altitude control
   float altitude;
   float vertical_velocity;
};

}   // namespace aeromight_boundaries
