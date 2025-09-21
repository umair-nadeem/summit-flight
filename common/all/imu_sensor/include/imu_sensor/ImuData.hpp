#pragma once

#include <optional>

namespace imu_sensor
{

struct ImuData
{
   struct Vec3
   {
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
   };

   std::optional<Vec3>  accel_mps2;      // Acceleration [m/s²]
   std::optional<Vec3>  gyro_radps;      // Angular velocity [rad/s]
   std::optional<float> temperature_c;   // Temperature [°C]
};

}   // namespace imu_sensor
