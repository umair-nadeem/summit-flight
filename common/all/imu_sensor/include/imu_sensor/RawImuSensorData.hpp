#pragma once

#include <optional>

#include "math/Vector3.hpp"

namespace imu_sensor
{

struct RawImuSensorData
{
   std::size_t          count;
   math::Vector3        accel_mps2;
   math::Vector3        gyro_radps;
   std::optional<float> temperature_c;
};

}   // namespace imu_sensor
