#pragma once

#include <algorithm>

#include "math/Vector3.hpp"

namespace aeromight_control
{

class RateController
{
public:
   static constexpr std::size_t num_axis = 3u;

   explicit RateController(const math::Vector3& gains_p,
                           const math::Vector3& gains_i,
                           const math::Vector3& gains_d,
                           const math::Vector3& integrator_limit,
                           const float          torque_output_limit,
                           const float          max_roll_rate_radps,
                           const float          max_pitch_rate_radps,
                           const float          max_yaw_rate_radps)
       : m_gains_p{gains_p},
         m_gains_i{gains_i},
         m_gains_d{gains_d},
         m_integrator_limit{integrator_limit},
         m_torque_output_limit{torque_output_limit},
         m_max_roll_rate_radps{max_roll_rate_radps},
         m_max_pitch_rate_radps{max_pitch_rate_radps},
         m_max_yaw_rate_radps{max_yaw_rate_radps}
   {
   }

   math::Vector3 update(const math::Vector3& rate_setpoint_radps,
                        const math::Vector3& rate_gyro_measured_radps,
                        const float          dt_s,
                        const bool           run_integrator)
   {

      if (!initialized)
      {
         reset();
         initialized = true;
         return {};
      }

      if (dt_s <= 0.0f)
      {
         return {};
      }

      const math::Vector3 setpoints_radps{std::clamp(rate_setpoint_radps[0], -m_max_roll_rate_radps, m_max_roll_rate_radps),
                                          std::clamp(rate_setpoint_radps[1], -m_max_pitch_rate_radps, m_max_pitch_rate_radps),
                                          std::clamp(rate_setpoint_radps[2], -m_max_yaw_rate_radps, m_max_yaw_rate_radps)};

      const math::Vector3 rate_error = setpoints_radps - rate_gyro_measured_radps;

      math::Vector3 torque_cmd = (m_gains_p.emul(rate_error)) + m_rate_integrator - (m_gains_d.emul(rate_gyro_measured_radps - m_previous_gyro_rate_measured) / dt_s);

      torque_cmd[0] = std::clamp(torque_cmd[0], -m_torque_output_limit, m_torque_output_limit);
      torque_cmd[1] = std::clamp(torque_cmd[1], -m_torque_output_limit, m_torque_output_limit);
      torque_cmd[2] = std::clamp(torque_cmd[2], -m_torque_output_limit, m_torque_output_limit);

      if (run_integrator)
      {
         update_integrator(rate_error, dt_s);
      }
      else
      {
         m_rate_integrator.zero();
      }

      m_previous_gyro_rate_measured = rate_gyro_measured_radps;
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
      m_previous_gyro_rate_measured.zero();
   }

private:
   void update_integrator(const math::Vector3& error, const float dt_s)
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
      m_rate_integrator[0] = std::clamp(m_rate_integrator[0], -m_integrator_limit[0], m_integrator_limit[0]);
      m_rate_integrator[1] = std::clamp(m_rate_integrator[1], -m_integrator_limit[1], m_integrator_limit[1]);
      m_rate_integrator[2] = std::clamp(m_rate_integrator[2], -m_integrator_limit[2], m_integrator_limit[2]);
   }

   const math::Vector3        m_gains_p;
   const math::Vector3        m_gains_i;
   const math::Vector3        m_gains_d;
   const math::Vector3        m_integrator_limit;
   const float                m_torque_output_limit;
   const float                m_max_roll_rate_radps;
   const float                m_max_pitch_rate_radps;
   const float                m_max_yaw_rate_radps;
   math::Vector3              m_rate_integrator{};
   math::Vector3              m_previous_gyro_rate_measured{};
   bool                       initialized{false};
   std::array<bool, num_axis> m_control_saturation_positive{};
   std::array<bool, num_axis> m_control_saturation_negative{};
};

}   // namespace aeromight_control
