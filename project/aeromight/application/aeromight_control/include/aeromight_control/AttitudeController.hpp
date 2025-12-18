#pragma once

#include "math/Vector3.hpp"

namespace aeromight_control
{

class AttitudeController
{
public:
   explicit AttitudeController(const math::Vector3 gains_p)
       : m_gains_p{gains_p}
   {
   }

   math::Vector3 update(const math::Vector3& angle_setpoint_rad, const math::Vector3& angle_estimation_rad) const
   {
      return m_gains_p * (angle_setpoint_rad - angle_estimation_rad);
   }

private:
   const math::Vector3 m_gains_p;
};
}   // namespace aeromight_control