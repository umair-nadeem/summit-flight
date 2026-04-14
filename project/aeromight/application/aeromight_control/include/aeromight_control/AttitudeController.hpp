#pragma once

#include "math/Vector3.hpp"

namespace aeromight_control
{

class AttitudeController
{
public:
   explicit AttitudeController(const math::Vec3f& gains_p)
       : m_gains_p{gains_p}
   {
   }

   math::Vec3f update(const math::Vec3f& angle_setpoint_rad, const math::Vec3f& angle_estimation_rad) const
   {
      return m_gains_p.emul(angle_setpoint_rad - angle_estimation_rad);
   }

private:
   const math::Vec3f m_gains_p;
};

}   // namespace aeromight_control