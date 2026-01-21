#pragma once

#include <array>

#include "aeromight_boundaries/ActuatorSetpoints.hpp"
#include "error/error_handler.hpp"

namespace aeromight_control
{

class ControlAllocator
{
   using SaturationFlags = std::array<bool, 3u>;

public:
   explicit ControlAllocator(const float actuator_min,
                             const float actuator_idle,
                             const float actuator_max,
                             const float yaw_saturation_limit_factor)
       : m_actuator_min{actuator_min},
         m_actuator_idle{actuator_idle},
         m_actuator_max{actuator_max},
         m_yaw_saturation_limit_factor{yaw_saturation_limit_factor}
   {
      error::verify((actuator_min < actuator_idle) && (actuator_idle < actuator_max));
      error::verify(aeromight_boundaries::ActuatorParams::min <= actuator_min);
      error::verify(actuator_max <= aeromight_boundaries::ActuatorParams::max);
   }

   void set_control_setpoints(const math::Vector4& control_setpoints)
   {
      m_control_setpoints = control_setpoints;
   }

   aeromight_boundaries::ActuatorSetpoints allocate()
   {
      math::Vector4 actuator_torque{};
      mix(actuator_torque,
          m_control_setpoints[aeromight_boundaries::ControlAxis::roll],
          m_control_setpoints[aeromight_boundaries::ControlAxis::pitch],
          apply_yaw_limit(m_control_setpoints[aeromight_boundaries::ControlAxis::yaw]));

      perform_desaturation(actuator_torque, m_control_setpoints[aeromight_boundaries::ControlAxis::thrust]);

      return math::Vector4{actuator_torque + m_control_setpoints[aeromight_boundaries::ControlAxis::thrust]};
   }

   void estimate_saturation()
   {
      using namespace aeromight_boundaries;

      m_control_saturation_positive = {false, false, false};
      m_control_saturation_negative = {false, false, false};

      // estimate yaw saturation
      const float yaw_control_sp = m_control_setpoints[ControlAxis::yaw];
      const float yaw_limit      = get_yaw_limit(m_control_setpoints[ControlAxis::thrust]);

      if ((0.0f < yaw_control_sp) && (yaw_limit < yaw_control_sp))   // positive yaw saturation
      {
         m_control_saturation_positive[ControlAxis::yaw] = true;
      }
      else if ((yaw_control_sp < 0.0f) && (yaw_control_sp < -yaw_limit))   // negative yaw saturation
      {
         m_control_saturation_negative[ControlAxis::yaw] = true;
      }

      if (!m_actuator_saturation)
      {
         return;
      }

      // find axis with max torque magnitude
      float   max_mag       = 0.0f;
      uint8_t dominant_axis = ControlAxis::roll;
      for (uint8_t axis = ControlAxis::roll; axis <= ControlAxis::yaw; axis++)
      {
         const float absolute_magnitude = std::fabs(m_control_setpoints[axis]);
         if (max_mag < absolute_magnitude)
         {
            max_mag       = absolute_magnitude;
            dominant_axis = axis;
         }
      }

      if (0.0f < m_control_setpoints[dominant_axis])
      {
         m_control_saturation_positive[dominant_axis] = true;
      }
      else if (m_control_setpoints[dominant_axis] < 0.0f)
      {
         m_control_saturation_negative[dominant_axis] = true;
      }
   }

   void clip_actuator_setpoints(aeromight_boundaries::ActuatorSetpoints& setpionts) const
   {
      setpionts[0] = std::clamp(setpionts[0], m_actuator_min, m_actuator_max);
      setpionts[1] = std::clamp(setpionts[1], m_actuator_min, m_actuator_max);
      setpionts[2] = std::clamp(setpionts[2], m_actuator_min, m_actuator_max);
      setpionts[3] = std::clamp(setpionts[3], m_actuator_min, m_actuator_max);
   }

   constexpr const SaturationFlags& get_actuator_saturation_positive() const noexcept
   {
      return m_control_saturation_positive;
   }

   constexpr const SaturationFlags& get_actuator_saturation_negative() const noexcept
   {
      return m_control_saturation_negative;
   }

private:
   void perform_desaturation(math::Vector4& actuator_torque, const float collective_thrust)
   {
      const float upper_margin   = m_actuator_max - collective_thrust;
      const float lower_margin   = std::max((collective_thrust - m_actuator_idle), 0.0f);
      const float available_span = 2.0f * std::min(upper_margin, lower_margin);   // will become zero at throttle extremes

      float torque_min = 0.0f;
      float torque_max = 0.0f;
      estimate_actuator_min_max(actuator_torque, torque_min, torque_max);

      // make torque zero-centered
      const float torque_center = 0.5f * (torque_min + torque_max);
      actuator_torque           = actuator_torque - torque_center;
      estimate_actuator_min_max(actuator_torque, torque_min, torque_max);

      const float torque_span = torque_max - torque_min;
      m_actuator_saturation   = (torque_span > available_span);

      if (m_actuator_saturation)
      {
         const float scale = (torque_span > epsilon) ? (available_span / torque_span) : 1.0f;   // scale down torque differential
         actuator_torque   = actuator_torque * scale;                                           // update actuator torque
      }
   }

   static constexpr void estimate_actuator_min_max(const math::Vector4& actuator_sp, float& actuator_min, float& actuator_max)
   {
      actuator_min = actuator_sp[0];
      actuator_max = actuator_sp[0];

      for (std::size_t i = 1u; i < actuator_sp.size; i++)
      {
         if (actuator_sp[i] < actuator_min)
         {
            actuator_min = actuator_sp[i];
         }
         else if (actuator_max < actuator_sp[i])
         {
            actuator_max = actuator_sp[i];
         }
      }
   }

   static void mix(math::Vector4& actuator_torque, const float roll, const float pitch, const float yaw) noexcept
   {
      actuator_torque[0] = roll + pitch + yaw;    // front-left CCW
      actuator_torque[1] = -roll + pitch - yaw;   // front-right CW
      actuator_torque[2] = -roll - pitch + yaw;   // rear-right CCW
      actuator_torque[3] = roll - pitch - yaw;    // rear-left CW
   }

   constexpr float get_yaw_limit(const float thrust) const noexcept
   {
      return (thrust * m_yaw_saturation_limit_factor);
   }

   float apply_yaw_limit(const float yaw_control_sp) const noexcept
   {
      const float yaw_limit = get_yaw_limit(m_control_setpoints[aeromight_boundaries::ControlAxis::thrust]);
      return std::clamp(yaw_control_sp, -yaw_limit, yaw_limit);
   }

   static constexpr float epsilon = 1e-6f;

   const float     m_actuator_min;
   const float     m_actuator_idle;
   const float     m_actuator_max;
   const float     m_yaw_saturation_limit_factor;
   math::Vector4   m_control_setpoints{0.0f};
   bool            m_actuator_saturation{false};
   SaturationFlags m_control_saturation_positive{};
   SaturationFlags m_control_saturation_negative{};
};

}   // namespace aeromight_control
