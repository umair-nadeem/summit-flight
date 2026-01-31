#include "control/AttitudeControl.hpp"

#include <algorithm>

namespace control
{

float AttitudeControl::f0 = 0.0f;
float AttitudeControl::f1 = 0.0f;

void AttitudeControl::init(const float hover_throttle, const float exp)
{
   f0 = throttle_curve_raw(0.0f, hover_throttle, exp);
   f1 = throttle_curve_raw(1.0f, hover_throttle, exp);
}

float AttitudeControl::throttle_curve_raw(const float throttle, const float hover_throttle, const float exp)
{
   const float s = std::max((1.0f - hover_throttle), hover_throttle);
   const float x = (throttle - hover_throttle) / s;
   const float y = (1.0f - exp) * x + (exp * x * x * x);

   return (hover_throttle + (s * y));
}

float AttitudeControl::throttle_curve(const float throttle, const float hover_throttle, const float exp)
{
   const float y = throttle_curve_raw(throttle, hover_throttle, exp);

   // Normalize to [0, 1]
   return (y - f0) / (f1 - f0);
}

}   // namespace control
