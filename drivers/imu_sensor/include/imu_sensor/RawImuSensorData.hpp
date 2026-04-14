#pragma once

#include <optional>

#include "math/Vector3.hpp"

namespace imu_sensor
{

struct RawImuSensorData
{
   std::size_t          count;
   math::Vec3f          accel_mps2;
   math::Vec3f          gyro_radps;
   std::optional<float> temperature_c;
};

}   // namespace imu_sensor
