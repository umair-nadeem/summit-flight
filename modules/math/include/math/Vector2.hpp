#pragma once

#include "Vector.hpp"

namespace math
{

template <typename T>
class Vec2 final : public Vector<T, 2u>
{
public:
   static constexpr std::size_t size = 2u;

   using BaseVector = Vector<T, size>;

   Vec2(const Vec2&)            = default;
   Vec2& operator=(const Vec2&) = default;

   Vec2()
       : BaseVector{}
   {
   }

   explicit Vec2(const std::array<T, size>& other)
       : BaseVector{other}
   {
   }

   explicit Vec2(const T scalar)
       : BaseVector{scalar}
   {
   }

   explicit Vec2(const T x, const T y)
       : BaseVector{}
   {
      auto& a = *(this);
      a[0]    = x;
      a[1]    = y;
   }

   explicit Vec2(const BaseVector& other)
       : BaseVector{other}
   {
   }

   Vec2& operator=(const BaseVector& other)
   {
      BaseVector::operator=(other);
      return *(this);
   }

   Vec2 operator+(const Vec2& other) const noexcept
   {
      return Vec2{BaseVector::operator+(other)};
   }

   Vec2 operator+(const T scalar) const noexcept
   {
      return Vec2{BaseVector::operator+(scalar)};
   }

   Vec2 operator-(const Vec2& other) const noexcept
   {
      return Vec2{BaseVector::operator-(other)};
   }

   Vec2 operator-(const T scalar) const noexcept
   {
      return Vec2{BaseVector::operator-(scalar)};
   }

   Vec2 emul(const Vec2& other) const noexcept
   {
      return Vec2{BaseVector::emul(other)};
   }

   Vec2 operator*(const T scalar) const noexcept
   {
      return Vec2{BaseVector::operator*(scalar)};
   }

   Vec2 ediv(const Vec2& other) const noexcept
   {
      return Vec2{BaseVector::ediv(other)};
   }

   Vec2 operator/(const T scalar) const noexcept
   {
      return Vec2{BaseVector::operator/(scalar)};
   }

   Vec2 normalized() const
   {
      return Vec2{BaseVector::normalized()};
   }
};

using Vector2 = Vec2<float>;

}   // namespace math
