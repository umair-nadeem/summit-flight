#pragma once

#include <array>
#include <bitset>

#include "aeromight_boundaries/ActuatorSetpoints.hpp"
#include "error/error_handler.hpp"
#include "math/Euler.hpp"

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

   aeromight_boundaries::ActuatorSetpoints allocate(const math::Euler& torque_setpoint,
                                                    const float        thrust_setpoint)
   {
      const float roll  = torque_setpoint.roll();
      const float pitch = torque_setpoint.pitch();
      const float yaw   = torque_setpoint.yaw();

      const std::array<float, aeromight_boundaries::ActuatorParams::num_actuators> torque_deltas{
          roll + pitch - yaw,    // front-left CW
          -roll + pitch + yaw,   // front-right CCW
          -roll - pitch - yaw,   // rear-right CW
          roll - pitch + yaw,    // rear-left CCW
      };

      aeromight_boundaries::ActuatorSetpoints actuator_setpoints{};
      actuator_setpoints.m1 = thrust_setpoint + torque_deltas[0];
      actuator_setpoints.m2 = thrust_setpoint + torque_deltas[1];
      actuator_setpoints.m3 = thrust_setpoint + torque_deltas[2];
      actuator_setpoints.m4 = thrust_setpoint + torque_deltas[3];

      m_control_saturation_positive = {false, false, false};
      m_control_saturation_negative = {false, false, false};

      return actuator_setpoints;
   }

   void clip_actuator_setpoints(aeromight_boundaries::ActuatorSetpoints& setpionts)
   {
      // apply final clamping
      setpionts.m1 = std::clamp(setpionts.m1, m_actuator_min, m_actuator_max);
      setpionts.m2 = std::clamp(setpionts.m2, m_actuator_min, m_actuator_max);
      setpionts.m3 = std::clamp(setpionts.m3, m_actuator_min, m_actuator_max);
      setpionts.m4 = std::clamp(setpionts.m4, m_actuator_min, m_actuator_max);
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
};

}   // namespace aeromight_control
