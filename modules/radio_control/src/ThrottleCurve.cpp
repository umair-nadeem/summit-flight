#include "radio_control/ThrottleCurve.hpp"

#include <algorithm>

#include "error/error_handler.hpp"
#include "math/constants.hpp"

namespace radio_control
{

ThrottleCurve::ThrottleCurve(const float throttle_hover, const float exponential)
    : m_throttle_hover{throttle_hover},
      m_exponential{exponential}
{
   error::verify((0.0f < m_throttle_hover) && (m_throttle_hover < 1.0f));
   error::verify((0.0f <= m_exponential) && (m_exponential <= 1.0f));

   m_s  = std::max((1.0f - throttle_hover), throttle_hover);
   m_f0 = apply_raw(0.0f);
   m_f1 = apply_raw(1.0f);

   error::verify(std::abs(m_f1 - m_f0) > math::constants::epsilon);

   m_inv_range = 1.0f / (m_f1 - m_f0);
}

float ThrottleCurve::apply_raw(const float throttle) const
{
   const float x = (throttle - m_throttle_hover) / m_s;
   const float y = (1.0f - m_exponential) * x + (m_exponential * x * x * x);

   return (m_throttle_hover + (m_s * y));
}

float ThrottleCurve::apply(const float throttle) const
{
   const float y = apply_raw(throttle);

   // Normalize to [0, 1]
   return (y - m_f0) * m_inv_range;
}

}   // namespace radio_control
