#pragma once

#include <cstdint>

#include "math/Quaternion.hpp"

namespace aeromight_control
{

struct StateEstimation
{
   uint32_t timestamp_ms{};

   math::Quaternion attitude{};
   math::Vector3    gyro_radps{};

   math::Vector3 gyro_bias{};

   // Euler angles
   math::Vector3 euler{};

   // altitude control
   float altitude;
   float vertical_velocity;
};

}   // namespace aeromight_control
