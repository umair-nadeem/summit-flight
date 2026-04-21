#pragma once

#include <algorithm>

#include "control/rate/RateControllerParams.hpp"

namespace control::rate
{

class RateController
{
   using SaturationVector = std::array<bool, math::Vec3f::size>;

public:
   explicit RateController(const RateControllerParams& params)
       : m_params{params}
   {
   }

   math::Vec3f update(const math::Vec3f& rate_setpoints,
                      const math::Vec3f& gyro_rate,
                      const math::Vec3f& gyro_rate_dterm,
                      const float        dt_s,
                      const bool         run_integrator)
   {

      if (!m_initialized)
      {
         m_initialized              = true;
         m_previous_gyro_rate_dterm = gyro_rate_dterm;
         m_rate_integrator.zero();
         return {};
      }

      const math::Vec3f rate_error      = rate_setpoints - gyro_rate;
      const math::Vec3f gyro_derivative = {(gyro_rate_dterm - m_previous_gyro_rate_dterm) / dt_s};
      m_previous_gyro_rate_dterm        = gyro_rate_dterm;

      const math::Vec3f p_term  = m_params.gains_p.emul(rate_error);
      const math::Vec3f i_term  = m_rate_integrator;
      const math::Vec3f d_term  = m_params.gains_d.emul(gyro_derivative);
      const math::Vec3f ff_term = m_params.gains_ff.emul(rate_setpoints);

      math::Vec3f torque_cmd = p_term + i_term - d_term + ff_term;

      for (std::size_t i = 0; i < torque_cmd.size; i++)
      {
         torque_cmd[i] = std::clamp(torque_cmd[i], -m_params.torque_output_limit, m_params.torque_output_limit);
      }

      if (run_integrator)
      {
         update_integrator(rate_error, dt_s);
      }
      else
      {
         m_rate_integrator.zero();
      }

      return torque_cmd;
   }

   void set_saturation_status(const SaturationVector& control_saturation_positive, const SaturationVector& control_saturation_negative)
   {
      m_control_saturation_positive = control_saturation_positive;
      m_control_saturation_negative = control_saturation_negative;
   }

   void reset()
   {
      m_rate_integrator.zero();
      m_control_saturation_positive.fill(false);
      m_control_saturation_negative.fill(false);
      m_initialized = false;
   }

private:
   void update_integrator(const math::Vec3f& error, const float dt_s)
   {
      auto integrator_error = error;
      for (std::size_t i = 0; i < math::Vec3f::size; i++)
      {
         if (m_control_saturation_positive[i])
         {
            integrator_error[i] = std::min(integrator_error[i], 0.0f);
         }

         if (m_control_saturation_negative[i])
         {
            integrator_error[i] = std::max(integrator_error[i], 0.0f);
         }
      }

      m_rate_integrator += m_params.gains_i.emul(integrator_error) * dt_s;

      for (std::size_t i = 0; i < m_rate_integrator.size; i++)
      {
         m_rate_integrator[i] = std::clamp(m_rate_integrator[i], -m_params.integrator_limit[i], m_params.integrator_limit[i]);
      }
   }

   const RateControllerParams& m_params;
   math::Vec3f                 m_rate_integrator{};
   math::Vec3f                 m_previous_gyro_rate_dterm{};
   SaturationVector            m_control_saturation_positive{};
   SaturationVector            m_control_saturation_negative{};
   bool                        m_initialized{false};
};

}   // namespace control::rate
