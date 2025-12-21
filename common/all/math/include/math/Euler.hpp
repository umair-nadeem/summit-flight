#pragma once

#include "Vector3.hpp"

namespace math
{

struct Euler
{
   explicit Euler()
   {
   }

   explicit Euler(const float x,
                  const float y,
                  const float z)
       : m_vector3{x, y, z}
   {
   }

   explicit Euler(Vector3& v)
       : m_vector3{v}
   {
   }

   void roll(const float x)
   {
      m_vector3.x = x;
   }

   void pitch(const float y)
   {
      m_vector3.y = y;
   }

   void yaw(const float z)
   {
      m_vector3.z = z;
   }

   void set(const Vector3& v)
   {
      m_vector3 = v;
   }

   void zero()
   {
      m_vector3.zero();
   }

   constexpr float roll() const noexcept { return m_vector3.x; }
   constexpr float pitch() const noexcept { return m_vector3.y; }
   constexpr float yaw() const noexcept { return m_vector3.z; }

   constexpr Vector3 vector() const noexcept { return m_vector3; }

private:
   Vector3 m_vector3{};
};

}   // namespace math
