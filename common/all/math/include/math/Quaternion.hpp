#pragma once

#include <cmath>

#include "Vector3.hpp"
#include "physics/constants.hpp"

namespace math
{

struct Quaternion
{
   float w;                   // scalar (real)
   float x;                   // vector component i
   float y;                   // vector component j
   float z;                   // vector component k

   float epsilon = 0.0001f;   // for comparison

   Quaternion()
       : w{1.0f},
         x{0.0f},
         y{0.0f},
         z{0.0f}
   {
   }

   Quaternion(const float _w, const float _x, const float _y, const float _z)
       : w{_w},
         x{_x},
         y{_y},
         z{_z}
   {
   }

   static constexpr bool nearly_equal(const float a, const float b, const float epsilon) noexcept
   {
      return std::abs(a - b) <= epsilon;
   }

   bool operator==(const Quaternion& other) const noexcept
   {
      return (nearly_equal(w, other.w, epsilon) &&
              nearly_equal(x, other.x, epsilon) &&
              nearly_equal(y, other.y, epsilon) &&
              nearly_equal(z, other.z, epsilon));
   }

   bool operator!=(const Quaternion& other) const noexcept
   {
      return !(*this == other);
   }

   Quaternion operator+(const Quaternion& other) const
   {
      Quaternion result{};
      result.w = this->w + other.w;
      result.x = this->x + other.x;
      result.y = this->y + other.y;
      result.z = this->z + other.z;
      return result;
   }

   Quaternion& operator+=(const Quaternion& other)
   {
      this->w += other.w;
      this->x += other.x;
      this->y += other.y;
      this->z += other.z;
      return *this;
   }

   Quaternion operator-(const Quaternion& other) const
   {
      Quaternion result{};
      result.w = this->w - other.w;
      result.x = this->x - other.x;
      result.y = this->y - other.y;
      result.z = this->z - other.z;
      return result;
   }

   Quaternion& operator-=(const Quaternion& other)
   {
      this->w -= other.w;
      this->x -= other.x;
      this->y -= other.y;
      this->z -= other.z;
      return *this;
   }

   Quaternion operator*(const float v) const
   {
      Quaternion result{};
      result.w = this->w * v;
      result.x = this->x * v;
      result.y = this->y * v;
      result.z = this->z * v;
      return result;
   }

   void normalize()
   {
      const float norm = get_norm();
      if (norm > 1e-6f)
      {
         const float inv_norm = 1.0f / norm;
         w *= inv_norm;
         x *= inv_norm;
         y *= inv_norm;
         z *= inv_norm;
      }
      else
      {
         // Degenerate case - reset to identity
         w = 1.0f;
         x = 0.0f;
         y = 0.0f;
         z = 0.0f;
      }
   }

   void set_epsilon(const float eps)
   {
      epsilon = eps;
   }

   float get_norm() const
   {
      return sqrtf((w * w) + (x * x) + (y * y) + (z * z));
   }

   Vector3 to_euler() const
   {
      // Roll (rotation around X axis)
      const float sinr_cosp = 2.0f * (w * x + y * z);
      const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);

      // Pitch (rotation around Y axis)
      const float sinp = 2.0f * (w * y - z * x);

      // Yaw (rotation around Z axis)
      const float siny_cosp = 2.0f * (w * z + x * y);
      const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);

      const float roll  = std::atan2(sinr_cosp, cosr_cosp);
      const float pitch = (std::abs(sinp) >= 1.0f) ? std::copysign(physics::constants::pi_by_2, sinp) : std::asin(sinp);
      const float yaw   = std::atan2(siny_cosp, cosy_cosp);

      return Vector3{roll, pitch, yaw};
   }
};

}   // namespace math
