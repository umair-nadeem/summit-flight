#pragma once

#include <array>
#include <bitset>

#include "aeromight_boundaries/ActuatorSetpoints.hpp"
#include "math/Euler.hpp"

namespace aeromight_control
{

class ControlAllocator
{
public:
   aeromight_boundaries::ActuatorSetpoints allocate(const math::Euler& torque_setpoint,
                                                    const float        thrust_setpoint,
                                                    const float        actuator_min,
                                                    const float        actuator_max)
   {
      const float roll  = torque_setpoint.roll();
      const float pitch = torque_setpoint.pitch();
      const float yaw   = torque_setpoint.yaw();

      std::array<float, aeromight_boundaries::ActuatorParams::num_actuators> torque_deltas{
          roll + pitch - yaw,    // front-left CW
          -roll + pitch + yaw,   // front-right CCW
          -roll - pitch - yaw,   // rear-right CW
          roll - pitch + yaw,    // rear-left CCW
      };

      // torque delta bounds
      const float upper_margin = actuator_max - thrust_setpoint;
      const float lower_margin = actuator_min - thrust_setpoint;

      std::bitset<aeromight_boundaries::ActuatorParams::num_actuators> max_delta_map{};
      std::bitset<aeromight_boundaries::ActuatorParams::num_actuators> min_delta_map{};

      float max_delta = std::numeric_limits<float>::min();   // initialize with opposite extremes
      float min_delta = std::numeric_limits<float>::max();

      for (std::size_t i = 0; i < aeromight_boundaries::ActuatorParams::num_actuators; i++)
      {
         if (torque_deltas[i] > max_delta)
         {
            max_delta = torque_deltas[i];
            max_delta_map.reset();
            max_delta_map.set(i, true);
         }

         if (torque_deltas[i] < min_delta)
         {
            min_delta = torque_deltas[i];
            min_delta_map.reset();
            min_delta_map.set(i, true);
         }
      }

      // scale only if range exceeded
      float scale = 1.0f;

      if (upper_margin < max_delta)
      {
         scale = std::min(scale, (upper_margin / max_delta));
      }

      if (min_delta < lower_margin)
      {
         scale = std::min(scale, (lower_margin / min_delta));
      }

      aeromight_boundaries::ActuatorSetpoints scaled_torque_deltas{torque_deltas[0] * scale,
                                                                   torque_deltas[1] * scale,
                                                                   torque_deltas[2] * scale,
                                                                   torque_deltas[3] * scale};

      m_control_saturation_positive = {false, false, false};
      m_control_saturation_negative = {false, false, false};

      if (scale < 1.0f)   // scaling needed
      {
         enum Motors : uint8_t
         {
            motor1 = 0,
            motor2,
            motor3,
            motor4
         };

         const bool left_side_up    = max_delta_map.test(Motors::motor1) || max_delta_map.test(Motors::motor4);
         const bool right_side_down = min_delta_map.test(Motors::motor2) || min_delta_map.test(Motors::motor3);
         const bool left_side_down  = min_delta_map.test(Motors::motor1) || min_delta_map.test(Motors::motor4);
         const bool right_side_up   = max_delta_map.test(Motors::motor2) || max_delta_map.test(Motors::motor3);

         const bool nose_up   = max_delta_map.test(Motors::motor1) || max_delta_map.test(Motors::motor2);
         const bool tail_down = min_delta_map.test(Motors::motor3) || min_delta_map.test(Motors::motor4);
         const bool nose_down = min_delta_map.test(Motors::motor1) || min_delta_map.test(Motors::motor2);
         const bool tail_up   = max_delta_map.test(Motors::motor3) || max_delta_map.test(Motors::motor4);

         const bool nose_right = max_delta_map.test(Motors::motor2) || max_delta_map.test(Motors::motor4);
         const bool tail_left  = min_delta_map.test(Motors::motor1) || min_delta_map.test(Motors::motor3);
         const bool nose_left  = min_delta_map.test(Motors::motor2) || min_delta_map.test(Motors::motor4);
         const bool tail_right = max_delta_map.test(Motors::motor1) || max_delta_map.test(Motors::motor3);

         m_control_saturation_positive[0] = left_side_up && right_side_down;
         m_control_saturation_negative[0] = left_side_down && right_side_up;

         m_control_saturation_positive[1] = nose_up && tail_down;
         m_control_saturation_negative[1] = nose_down && tail_up;

         m_control_saturation_positive[2] = nose_right && tail_left;
         m_control_saturation_negative[2] = nose_left && tail_right;
      }

      aeromight_boundaries::ActuatorSetpoints actuator_setpoints{};
      actuator_setpoints.m1 = thrust_setpoint + scaled_torque_deltas.m1;
      actuator_setpoints.m2 = thrust_setpoint + scaled_torque_deltas.m2;
      actuator_setpoints.m3 = thrust_setpoint + scaled_torque_deltas.m3;
      actuator_setpoints.m4 = thrust_setpoint + scaled_torque_deltas.m4;

      // apply final clamping
      actuator_setpoints.m1 = std::clamp(actuator_setpoints.m1, actuator_min, actuator_max);
      actuator_setpoints.m2 = std::clamp(actuator_setpoints.m2, actuator_min, actuator_max);
      actuator_setpoints.m3 = std::clamp(actuator_setpoints.m3, actuator_min, actuator_max);
      actuator_setpoints.m4 = std::clamp(actuator_setpoints.m4, actuator_min, actuator_max);

      return actuator_setpoints;
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
   std::array<bool, 3u> m_control_saturation_positive{};
   std::array<bool, 3u> m_control_saturation_negative{};
};

}   // namespace aeromight_control
