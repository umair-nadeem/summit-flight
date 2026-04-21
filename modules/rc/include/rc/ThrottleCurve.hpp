#pragma once

#include "ThrottleCurveParams.hpp"

namespace rc
{

class ThrottleCurve
{
public:
   explicit ThrottleCurve(const ThrottleCurveParams& params);
   float apply_raw(const float throttle) const;
   float apply(const float throttle) const;

private:
   const ThrottleCurveParams& m_params;
   float                      m_s{};
   float                      m_f0{};
   float                      m_f1{};
   float                      m_inv_range{};
};

}   // namespace rc
