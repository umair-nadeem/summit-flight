#pragma once

namespace control
{

class AttitudeControl
{
public:
   static void  init(const float hover_throttle, const float exp);
   static float throttle_curve_raw(const float throttle, const float hover_throttle, const float exp);
   static float throttle_curve(const float throttle, const float hover_throttle, const float exp);

private:
   static float f0;
   static float f1;
};

}   // namespace control
