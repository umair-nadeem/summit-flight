#pragma once

#include "Vector.hpp"

namespace math
{

template <typename T>
class Vec4 final : public Vector<T, 4u>
{
public:
   static constexpr std::size_t size = 4u;

   using BaseVector = Vector<T, size>;

   Vec4(const Vec4&)            = default;
   Vec4& operator=(const Vec4&) = default;

   Vec4()
       : BaseVector{}
   {
   }

   explicit Vec4(const std::array<T, size>& other)
       : BaseVector{other}
   {
   }

   explicit Vec4(const T scalar)
       : BaseVector{scalar}
   {
   }

   explicit Vec4(const T w, const T x, const T y, const T z)
       : BaseVector{}
   {
      auto& a = *(this);
      a[0]    = w;
      a[1]    = x;
      a[2]    = y;
      a[3]    = z;
   }

   explicit Vec4(const BaseVector& other)
       : BaseVector{other}
   {
   }

   Vec4& operator=(const BaseVector& other)
   {
      BaseVector::operator=(other);
      return *(this);
   }

   Vec4 operator+(const Vec4& other) const noexcept
   {
      return Vec4{BaseVector::operator+(other)};
   }

   Vec4 operator+(const T scalar) const noexcept
   {
      return Vec4{BaseVector::operator+(scalar)};
   }

   Vec4 operator-(const Vec4& other) const noexcept
   {
      return Vec4{BaseVector::operator-(other)};
   }

   Vec4 operator-(const T scalar) const noexcept
   {
      return Vec4{BaseVector::operator-(scalar)};
   }

   Vec4 emul(const Vec4& other) const noexcept
   {
      return Vec4{BaseVector::emul(other)};
   }

   Vec4 operator*(const T scalar) const noexcept
   {
      return Vec4{BaseVector::operator*(scalar)};
   }

   Vec4 ediv(const Vec4& other) const noexcept
   {
      return Vec4{BaseVector::ediv(other)};
   }

   Vec4 operator/(const T scalar) const noexcept
   {
      return Vec4{BaseVector::operator/(scalar)};
   }

   Vec4 normalized() const
   {
      return Vec4{BaseVector::normalized()};
   }
};

using Vector4 = Vec4<float>;

}   // namespace math
