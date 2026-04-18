#pragma once

namespace rc
{

class ThrottleCurve
{
public:
   explicit ThrottleCurve(const float throttle_hover, const float exponential);
   float apply_raw(const float throttle) const;
   float apply(const float throttle) const;

private:
   const float m_throttle_hover;
   const float m_exponential;
   float       m_s{};
   float       m_f0{};
   float       m_f1{};
   float       m_inv_range{};
};

}   // namespace rc
