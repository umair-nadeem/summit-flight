#pragma once

#include <array>

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
       : m_data{x, y, z}
   {
   }

   void roll(const float x)
   {
      m_data[0] = x;
   }

   void pitch(const float y)
   {
      m_data[1] = y;
   }

   void yaw(const float z)
   {
      m_data[2] = z;
   }

   void zero()
   {
      m_data.fill(0.0f);
   }

   constexpr float roll() const noexcept { return m_data[0]; }
   constexpr float pitch() const noexcept { return m_data[1]; }
   constexpr float yaw() const noexcept { return m_data[2]; }

private:
   std::array<float, 3u> m_data{};
};

}   // namespace math
