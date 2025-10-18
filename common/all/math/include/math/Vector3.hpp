#pragma once

#include <cmath>

namespace math
{

struct Vector3
{
   float x = 0.0f;
   float y = 0.0f;
   float z = 0.0f;

   float epsilon = 0.0001f;   // for comparison

   static constexpr bool nearly_equal(const float a, const float b, const float epsilon) noexcept
   {
      return std::abs(a - b) <= epsilon;
   }

   bool operator==(const Vector3& other) const noexcept
   {
      return (nearly_equal(x, other.x, epsilon) &&
              nearly_equal(y, other.y, epsilon) &&
              nearly_equal(z, other.z, epsilon));
   }

   bool operator!=(const Vector3& other) const noexcept
   {
      return !(*this == other);
   }

   Vector3 operator+(const Vector3& other) const
   {
      Vector3 result{};
      result.x = this->x + other.x;
      result.y = this->y + other.y;
      result.z = this->z + other.z;
      return result;
   }

   Vector3& operator+=(const Vector3& other)
   {
      this->x += other.x;
      this->y += other.y;
      this->z += other.z;
      return *this;
   }

   Vector3 operator-(const Vector3& other) const
   {
      Vector3 result{};
      result.x = this->x - other.x;
      result.y = this->y - other.y;
      result.z = this->z - other.z;
      return result;
   }

   Vector3& operator-=(const Vector3& other)
   {
      this->x -= other.x;
      this->y -= other.y;
      this->z -= other.z;
      return *this;
   }

   Vector3 operator*(const Vector3& other) const
   {
      Vector3 result{};
      result.x = this->x * other.x;
      result.y = this->y * other.y;
      result.z = this->z * other.z;
      return result;
   }

   Vector3 operator*(const float v) const
   {
      Vector3 result{};
      result.x = this->x * v;
      result.y = this->y * v;
      result.z = this->z * v;
      return result;
   }

   Vector3 operator/(const float v) const
   {
      Vector3 result{};
      result.x = this->x / v;
      result.y = this->y / v;
      result.z = this->z / v;
      return result;
   }

   void normalize()
   {
      const float norm = get_norm();
      if (norm > 1e-6f)
      {
         const float inv_norm = 1.0f / norm;
         x *= inv_norm;
         y *= inv_norm;
         z *= inv_norm;
      }
   }

   void set_epsilon(const float eps)
   {
      epsilon = eps;
   }

   float get_norm() const
   {
      return sqrtf((x * x) + (y * y) + (z * z));
   }
};

}   // namespace math
