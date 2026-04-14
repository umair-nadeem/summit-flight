#pragma once

#include <algorithm>

#include "math/Vector3.hpp"

namespace aeromight_control
{

class RateController
{
public:
   static constexpr std::size_t num_axis = aeromight_boundaries::num_axis;

   explicit RateController(const math::Vec3f& gains_p,
                           const math::Vec3f& gains_i,
                           const math::Vec3f& gains_d,
                           const math::Vec3f& gains_ff,
                           const math::Vec3f& integrator_limit,
                           const float        torque_output_limit)
       : m_gains_p{gains_p},
         m_gains_i{gains_i},
         m_gains_d{gains_d},
         m_gains_ff{gains_ff},
         m_integrator_limit{integrator_limit},
         m_torque_output_limit{torque_output_limit}
   {
   }

   math::Vec3f update(const math::Vec3f& rate_setpoint_radps,
                      const math::Vec3f& gyro_radps,
                      const math::Vec3f& gyro_derivative_radps2,
                      const float        dt_s,
                      const bool         run_integrator)
   {

      if (!initialized)
      {
         initialized = true;
         return {};
      }

      const math::Vec3f rate_error = rate_setpoint_radps - gyro_radps;
      const math::Vec3f p_term     = m_gains_p.emul(rate_error);
      const math::Vec3f i_term     = m_rate_integrator;
      const math::Vec3f d_term     = m_gains_d.emul(gyro_derivative_radps2);
      const math::Vec3f ff_term    = m_gains_ff.emul(rate_setpoint_radps);

      math::Vec3f torque_cmd = p_term + i_term - d_term + ff_term;

      for (std::size_t i = 0; i < torque_cmd.size; i++)
      {
         torque_cmd[i] = std::clamp(torque_cmd[i], -m_torque_output_limit, m_torque_output_limit);
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

   void set_saturation_status(const std::array<bool, num_axis>& control_saturation_positive, const std::array<bool, num_axis>& control_saturation_negative)
   {
      m_control_saturation_positive = control_saturation_positive;
      m_control_saturation_negative = control_saturation_negative;
   }

   void reset()
   {
      m_rate_integrator.zero();
      m_control_saturation_positive.fill(false);
      m_control_saturation_negative.fill(false);
      initialized = false;
   }

private:
   void update_integrator(const math::Vec3f& error, const float dt_s)
   {
      auto integrator_error = error;
      for (std::size_t i = 0; i < num_axis; i++)
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

      m_rate_integrator += m_gains_i.emul(integrator_error) * dt_s;

      for (std::size_t i = 0; i < m_rate_integrator.size; i++)
      {
         m_rate_integrator[i] = std::clamp(m_rate_integrator[i], -m_integrator_limit[i], m_integrator_limit[i]);
      }
   }

   const math::Vec3f          m_gains_p;
   const math::Vec3f          m_gains_i;
   const math::Vec3f          m_gains_d;
   const math::Vec3f          m_gains_ff;
   const math::Vec3f          m_integrator_limit;
   const float                m_torque_output_limit;
   math::Vec3f                m_rate_integrator{};
   std::array<bool, num_axis> m_control_saturation_positive{};
   std::array<bool, num_axis> m_control_saturation_negative{};
   bool                       initialized{false};
};

}   // namespace aeromight_control
