#pragma once

#include "Vector.hpp"

namespace math
{

template <typename T>
class Vec3 final : public Vector<T, 3u>
{
public:
   static constexpr std::size_t size = 3u;

   using BaseVector = Vector<T, size>;

   Vec3()                       = default;
   Vec3(const Vec3&)            = default;
   Vec3& operator=(const Vec3&) = default;

   explicit Vec3(const std::array<T, size>& other)
       : BaseVector{other}
   {
   }

   explicit Vec3(const T scalar)
       : BaseVector{scalar}
   {
   }

   explicit Vec3(const T x, const T y, const T z)
       : BaseVector{}
   {
      auto& a = *(this);
      a[0]    = x;
      a[1]    = y;
      a[2]    = z;
   }

   explicit Vec3(const BaseVector& other)
       : BaseVector{other}
   {
   }

   Vec3& operator=(const BaseVector& other)
   {
      BaseVector::operator=(other);
      return *(this);
   }

   Vec3 operator+(const Vec3& other) const noexcept
   {
      return Vec3{BaseVector::operator+(other)};
   }

   Vec3 operator+(const T scalar) const noexcept
   {
      return Vec3{BaseVector::operator+(scalar)};
   }

   Vec3 operator-(const Vec3& other) const noexcept
   {
      return Vec3{BaseVector::operator-(other)};
   }

   Vec3 operator-(const T scalar) const noexcept
   {
      return Vec3{BaseVector::operator-(scalar)};
   }

   Vec3 emul(const Vec3& other) const noexcept
   {
      return Vec3{BaseVector::emul(other)};
   }

   Vec3 operator*(const T scalar) const noexcept
   {
      return Vec3{BaseVector::operator*(scalar)};
   }

   Vec3 ediv(const Vec3& other) const noexcept
   {
      return Vec3{BaseVector::ediv(other)};
   }

   Vec3 operator/(const T scalar) const noexcept
   {
      return Vec3{BaseVector::operator/(scalar)};
   }

   Vec3 normalized() const
   {
      return Vec3{BaseVector::normalized()};
   }

   constexpr Vec3 cross(const Vec3& b) const noexcept
   {
      const auto& a = *(this);
      return Vec3{(a[1] * b[2]) - (a[2] * b[1]),
                  (-a[0] * b[2]) + (a[2] * b[0]),
                  (a[0] * b[1]) - (a[1] * b[0])};
   }
};

using Vector3 = Vec3<float>;

}   // namespace math
