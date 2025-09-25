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

      Vec3 operator+(const Vec3& other) const
      {
         Vec3 result{};
         result.x = this->x + other.x;
         result.y = this->y + other.y;
         result.z = this->z + other.z;
         return result;
      }

      Vec3 operator-(const Vec3& other) const
      {
         Vec3 result{};
         result.x = this->x - other.x;
         result.y = this->y - other.y;
         result.z = this->z - other.z;
         return result;
      }

      Vec3 operator*(const Vec3& other) const
      {
         Vec3 result{};
         result.x = this->x * other.x;
         result.y = this->y * other.y;
         result.z = this->z * other.z;
         return result;
      }

      Vec3 operator/(const float v) const
      {
         Vec3 result{};
         result.x = this->x / v;
         result.y = this->y / v;
         result.z = this->z / v;
         return result;
      }
   };

   std::optional<Vec3>  accel_mps2;      // Acceleration [m/s²]
   std::optional<Vec3>  gyro_radps;      // Angular velocity [rad/s]
   std::optional<float> temperature_c;   // Temperature [°C]
};

}   // namespace imu_sensor
