#pragma once

#include <cmath>

namespace physics
{

struct Vector3
{
   float x = 0.0f;
   float y = 0.0f;
   float z = 0.0f;

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

   float get_norm() const
   {
      return sqrtf((x * x) + (y * y) + (z * z));
   }
};

}   // namespace physics
