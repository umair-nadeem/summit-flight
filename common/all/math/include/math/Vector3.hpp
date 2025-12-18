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

   Vector3 operator%(const Vector3& other) const
   {
      return (*this).cross(other);
   }

   Vector3 operator/(const float v) const
   {
      Vector3 result{};
      result.x = this->x / v;
      result.y = this->y / v;
      result.z = this->z / v;
      return result;
   }

   void zero()
   {
      x = 0;
      y = 0;
      z = 0;
   }

   void normalize()
   {
      const float v = norm();
      if (v > 1e-6f)
      {
         x /= v;
         y /= v;
         z /= v;
      }
   }

   Vector3 normalized() const
   {
      return (*this) / norm();
   }

   void set_epsilon(const float eps)
   {
      epsilon = eps;
   }

   float norm() const
   {
      return sqrtf((x * x) + (y * y) + (z * z));
   }

   float norm_squared() const
   {
      return ((x * x) + (y * y) + (z * z));
   }

   float length() const
   {
      return norm();
   }

private:
   Vector3 cross(const Vector3& other) const
   {
      return {(y * other.z) - (z * other.y),
              (-x * other.z) + (z * other.x),
              (x * other.y) - (y * other.x)};
   }
};

}   // namespace math
