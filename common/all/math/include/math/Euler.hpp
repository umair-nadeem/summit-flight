#pragma once

#include <array>

namespace math
{

struct Euler
{
   explicit Euler()
       : m_data{}
   {
      zero();
   }

   explicit Euler(const float x,
                  const float y,
                  const float z)
       : m_data{x, y, z}
   {
   }

   float& roll() noexcept
   {
      return m_data[0];
   }

   float& pitch() noexcept
   {
      return m_data[1];
   }

   float& yaw() noexcept
   {
      return m_data[2];
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
