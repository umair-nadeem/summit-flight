#pragma once

namespace rc
{

struct ThrottleCurveParams
{
   float throttle_hover = 0.3f;
   float exponential    = 0.7f;
};

}   // namespace rc