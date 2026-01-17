#pragma once

#include "physics/constants.hpp"

namespace math
{

class FirstOrderLpf
{
public:
   explicit FirstOrderLpf(const float cutoff_frequency_hz)
       : m_rc{1.0f / (2.0f * physics::constants::pi * cutoff_frequency_hz)}
   {
   }

   float apply(const float sample, const float dt_s)
   {
      const float alpha = dt_s / (dt_s + m_rc);
      m_state += alpha * (sample - m_state);
      return m_state;
   }

   void reset(const float value = 0.0f)
   {
      m_state = value;
   }

private:
   const float m_rc;
   float       m_state{0.0f};
};

}   // namespace math
