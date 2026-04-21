#pragma once

#include "control/attitude/AttitudeControllerParams.hpp"

namespace control::attitude
{

class AttitudeController
{
public:
   explicit AttitudeController(const AttitudeControllerParams& params)
       : m_params{params}
   {
   }

   math::Vec3f update(const math::Vec3f& angle_setpoint_rad, const math::Vec3f& angle_estimation_rad) const
   {
      return m_params.gains_p.emul(angle_setpoint_rad - angle_estimation_rad);
   }

private:
   const AttitudeControllerParams& m_params;
};

}   // namespace control::attitude
