#pragma once

#include "Vector.hpp"

namespace math
{

template <typename T>
class Vec4 final : public Vector<T, 4u>
{
public:
   Vec4()                       = default;
   Vec4(const Vec4&)            = default;
   Vec4& operator=(const Vec4&) = default;

   explicit Vec4(const std::array<T, 4u>& other)
       : Vector<T, 4u>{other}
   {
   }

   explicit Vec4(const T w, const T x, const T y, const T z)
       : Vector<T, 4u>{}
   {
      auto& a = *(this);
      a[0]    = w;
      a[1]    = x;
      a[2]    = y;
      a[3]    = z;
   }

   explicit Vec4(const Vector<T, 4u>& other)
       : Vector<T, 4u>{other}
   {
   }

   Vec4& operator=(const Vector<T, 4u>& other)
   {
      Vector<T, 4u>::operator=(other);
      return *(this);
   }

   Vec4 operator+(const Vec4& other) const noexcept
   {
      return Vec4{Vector<T, 4u>::operator+(other)};
   }

   Vec4 operator+(const T scalar) const noexcept
   {
      return Vec4{Vector<T, 4u>::operator+(scalar)};
   }

   Vec4 operator-(const Vec4& other) const noexcept
   {
      return Vec4{Vector<T, 4u>::operator-(other)};
   }

   Vec4 operator-(const T scalar) const noexcept
   {
      return Vec4{Vector<T, 4u>::operator-(scalar)};
   }

   Vec4 emul(const Vec4& other) const noexcept
   {
      return Vec4{Vector<T, 4u>::emul(other)};
   }

   Vec4 operator*(const T scalar) const noexcept
   {
      return Vec4{Vector<T, 4u>::operator*(scalar)};
   }

   Vec4 ediv(const Vec4& other) const noexcept
   {
      return Vec4{Vector<T, 4u>::ediv(other)};
   }

   Vec4 operator/(const T scalar) const noexcept
   {
      return Vec4{Vector<T, 4u>::operator/(scalar)};
   }

   Vec4 normalized() const
   {
      return Vec4{Vector<T, 4u>::normalized()};
   }
};

using Vector4 = Vec4<float>;

}   // namespace math
