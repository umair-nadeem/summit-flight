#pragma once

#include <optional>

#include "math/Vector3.hpp"

namespace imu_sensor
{

struct ImuData
{
   std::optional<math::Vector3> accel_mps2;      // Acceleration [m/s²]
   std::optional<math::Vector3> gyro_radps;      // Angular velocity [rad/s]
   std::optional<float>         temperature_c;   // Temperature [°C]
};

}   // namespace imu_sensor
