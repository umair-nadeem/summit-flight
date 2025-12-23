#pragma once

#include <cmath>

#include "Euler.hpp"
#include "physics/constants.hpp"

namespace math
{

struct Quaternion final : public Vector<float, 4u>
{
   Quaternion()
       : Vector<float, 4u>{}
   {
      auto& a = *(this);
      a[0]    = 1.0f;
      a[1]    = 0.0f;
      a[2]    = 0.0f;
      a[3]    = 0.0f;
   }

   explicit Quaternion(const float w, const float x, const float y, const float z)
       : Vector<float, 4u>{}
   {
      auto& a = *(this);
      a[0]    = w;
      a[1]    = x;
      a[2]    = y;
      a[3]    = z;
   }

   explicit Quaternion(const Vector<float, 4u>& other)
       : Vector<float, 4u>{other}
   {
   }

   Quaternion operator+(const Quaternion& other) const noexcept
   {
      return Quaternion{Vector<float, 4u>::operator+(other)};
   }

   Quaternion operator+(const float scalar) const noexcept
   {
      return Quaternion{Vector<float, 4u>::operator+(scalar)};
   }

   Quaternion operator-(const Quaternion& other) const noexcept
   {
      return Quaternion{Vector<float, 4u>::operator-(other)};
   }

   Quaternion operator-(const float scalar) const noexcept
   {
      return Quaternion{Vector<float, 4u>::operator-(scalar)};
   }

   Quaternion quat_mul(const Quaternion& other) const noexcept
   {
      const auto& a = *(this);
      return Quaternion{
          (a[0] * other[0]) - (a[1] * other[1]) - (a[2] * other[2]) - (a[3] * other[3]),
          (a[1] * other[0]) + (a[0] * other[1]) - (a[3] * other[2]) + (a[2] * other[3]),
          (a[2] * other[0]) + (a[3] * other[1]) + (a[0] * other[2]) - (a[1] * other[3]),
          (a[3] * other[0]) - (a[2] * other[1]) + (a[1] * other[2]) + (a[0] * other[3])};
   }

   Quaternion operator*(const float scalar) const noexcept
   {
      return Quaternion{Vector<float, 4u>::operator*(scalar)};
   }

   // derivative when rotated with angular velocity expressed in frame 1 (typically body frame)
   Quaternion derivative1(const Vector3& other) const noexcept
   {
      const Quaternion& q = *this;
      Quaternion        v(0, other[0], other[1], other[2]);
      return q.quat_mul(v) * 0.5f;
   }

   // derivative when rotated with angular velocity expressed in frame 2 (typically reference frame)
   Quaternion derivative2(const Vector3& other) const noexcept
   {
      const Quaternion& q = *this;
      Quaternion        v(0, other[0], other[1], other[2]);
      return v.quat_mul(q) * 0.5f;
   }

   constexpr bool is_all_finite() const noexcept
   {
      const auto& a = *(this);
      return (std::isfinite(a[0]) && std::isfinite(a[1]) && std::isfinite(a[2]) && std::isfinite(a[3]));
   }

   Quaternion normalized() const
   {
      Quaternion q{*this};
      q.normalize();
      return q;
   }

   void normalize()
   {
      const float n = norm();
      auto&       a = *(this);
      if (n > 1e-6f)
      {
         a /= n;
      }
      else
      {
         // Degenerate case - reset to identity
         a[0] = 1.0f;
         a[1] = 0.0f;
         a[2] = 0.0f;
         a[3] = 0.0f;
      }
   }
};

}   // namespace math
