#pragma once

#include "Vector.hpp"

namespace math
{

template <typename T>
class Vec3 final : public Vector<T, 3u>
{
public:
   Vec3()                       = default;
   Vec3(const Vec3&)            = default;
   Vec3& operator=(const Vec3&) = default;

   explicit Vec3(const std::array<T, 3u>& other)
       : Vector<T, 3u>{other}
   {
   }

   Vec3(const T x, const T y, const T z)
       : Vector<T, 3u>{}
   {
      auto& a = *(this);
      a[0]    = x;
      a[1]    = y;
      a[2]    = z;
   }

   explicit Vec3(const Vector<T, 3u>& other)
       : Vector<T, 3u>{other}
   {
   }

   Vec3& operator=(const Vector<T, 3u>& other)
   {
      Vector<T, 3u>::operator=(other);
      return *(this);
   }

   Vec3 operator+(const Vec3& other) const
   {
      return Vec3{Vector<T, 3u>::operator+(other)};
   }

   Vec3 operator+(const T scalar) const
   {
      return Vec3{Vector<T, 3u>::operator+(scalar)};
   }

   Vec3 operator-(const Vec3& other) const
   {
      return Vec3{Vector<T, 3u>::operator-(other)};
   }

   Vec3 operator-(const T scalar) const
   {
      return Vec3{Vector<T, 3u>::operator-(scalar)};
   }

   Vec3 emul(const Vec3& other) const
   {
      return Vec3{Vector<T, 3u>::emul(other)};
   }

   Vec3 operator*(const T scalar) const
   {
      return Vec3{Vector<T, 3u>::operator*(scalar)};
   }

   Vec3 ediv(const Vec3& other) const
   {
      return Vec3{Vector<T, 3u>::ediv(other)};
   }

   Vec3 operator/(const T scalar) const
   {
      return Vec3{Vector<T, 3u>::operator/(scalar)};
   }

   Vec3 normalized() const
   {
      return Vec3{Vector<T, 3u>::normalized()};
   }

   constexpr Vec3 cross(const Vec3& b) const noexcept
   {
      const auto& a = *(this);
      return {(a[1] * b[2]) - (a[2] * b[1]),
              (-a[0] * b[2]) + (a[2] * b[0]),
              (a[0] * b[1]) - (a[1] * b[0])};
   }
};

using Vector3 = Vec3<float>;

}   // namespace math
