#pragma once

#include <array>
#include <bitset>

#include "aeromight_boundaries/ActuatorSetpoints.hpp"
#include "error/error_handler.hpp"

namespace aeromight_control
{

class ControlAllocator
{
public:
   explicit ControlAllocator(const float actuator_min,
                             const float actuator_max,
                             const float thrust_model_factor)
       : m_actuator_min{actuator_min},
         m_actuator_max{actuator_max},
         m_thrust_model_factor{thrust_model_factor}
   {
      error::verify(actuator_min < actuator_max);
      error::verify(aeromight_boundaries::ActuatorParams::min <= actuator_min);
      error::verify(actuator_max <= aeromight_boundaries::ActuatorParams::max);
   }

   aeromight_boundaries::ActuatorSetpoints allocate(const math::Vector4& control_setpoints)
   {
      const float roll   = control_setpoints[aeromight_boundaries::control_axis::roll];
      const float pitch  = control_setpoints[aeromight_boundaries::control_axis::pitch];
      const float yaw    = control_setpoints[aeromight_boundaries::control_axis::yaw];
      const float thrust = control_setpoints[aeromight_boundaries::control_axis::thrust];

      const math::Vector4 torque_deltas{
          roll + pitch - yaw,    // front-left CW
          -roll + pitch + yaw,   // front-right CCW
          -roll - pitch - yaw,   // rear-right CW
          roll - pitch + yaw,    // rear-left CCW
      };

      aeromight_boundaries::ActuatorSetpoints actuator_setpoints = torque_deltas + thrust;

      m_control_saturation_positive = {false, false, false};
      m_control_saturation_negative = {false, false, false};

      m_last_control_setpoints = control_setpoints;

      return actuator_setpoints;
   }

   void clip_actuator_setpoints(aeromight_boundaries::ActuatorSetpoints& setpionts) const
   {
      // apply final clamping
      setpionts[0] = std::clamp(setpionts[0], m_actuator_min, m_actuator_max);
      setpionts[1] = std::clamp(setpionts[1], m_actuator_min, m_actuator_max);
      setpionts[2] = std::clamp(setpionts[2], m_actuator_min, m_actuator_max);
      setpionts[3] = std::clamp(setpionts[3], m_actuator_min, m_actuator_max);
   }

   float estimate_collective_thrust(const float throttle) const
   {
      const float thrust_signal = m_actuator_min + (throttle * (1.0f - m_actuator_min));
      return (m_thrust_model_factor * (thrust_signal * thrust_signal)) + ((1.0f - m_thrust_model_factor) * thrust_signal);
   }

   const std::array<bool, 3u>& get_actuator_saturation_positive() const
   {
      return m_control_saturation_positive;
   }

   const std::array<bool, 3u>& get_actuator_saturation_negative() const
   {
      return m_control_saturation_negative;
   }

private:
   const float          m_actuator_min;
   const float          m_actuator_max;
   const float          m_thrust_model_factor;
   std::array<bool, 3u> m_control_saturation_positive{};
   std::array<bool, 3u> m_control_saturation_negative{};
   math::Vector4        m_last_control_setpoints{};
};

}   // namespace aeromight_control
