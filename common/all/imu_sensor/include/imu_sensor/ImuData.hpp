#pragma once

#include <optional>

#include "physics/Vector3.hpp"

namespace imu_sensor
{

struct ImuData
{
   std::optional<physics::Vector3> accel_mps2;      // Acceleration [m/s²]
   std::optional<physics::Vector3> gyro_radps;      // Angular velocity [rad/s]
   std::optional<float>            temperature_c;   // Temperature [°C]
};

}   // namespace imu_sensor
