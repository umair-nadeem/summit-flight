#pragma once

#include <algorithm>

#include "StateEstimation.hpp"
#include "aeromight_boundaries/ActuatorSetpoints.hpp"
#include "aeromight_boundaries/ControlHealth.hpp"
#include "aeromight_boundaries/ControlSetpoints.hpp"
#include "aeromight_boundaries/ControlState.hpp"
#include "boundaries/SharedData.hpp"
#include "error/error_handler.hpp"
#include "interfaces/IClockSource.hpp"

namespace aeromight_control
{

template <typename AttitudeController, typename RateController, interfaces::IClockSource ClockSource, typename Logger>
class Control
{
   using ActuatorSetpoints = boundaries::SharedData<aeromight_boundaries::ActuatorSetpoints>;
   using ControlHealth     = boundaries::SharedData<aeromight_boundaries::ControlHealth>;
   using ControlSetpoints  = boundaries::SharedData<aeromight_boundaries::ControlSetpoints>;

public:
   explicit Control(AttitudeController&     attitude_controller,
                    RateController&         rate_controller,
                    ActuatorSetpoints&      actuator_setpoints_storage,
                    ControlHealth&          control_health_storage,
                    const ControlSetpoints& control_setpoints_storage,
                    const StateEstimation&  state_estimation_data,
                    Logger&                 logger,
                    const float             max_roll_rate_radps,
                    const float             max_pitch_rate_radps,
                    const float             max_yaw_rate_radps,
                    const float             max_tilt_angle_rad,
                    const float             idle_thrust,
                    const float             max_thrust,
                    const float             lift_throttle,
                    const float             thrust_model_factor)
       : m_attitude_controller{attitude_controller},
         m_rate_controller{rate_controller},
         m_actuator_setpoints_storage{actuator_setpoints_storage},
         m_control_health_storage{control_health_storage},
         m_control_setpoints_storage{control_setpoints_storage},
         m_state_estimation_data{state_estimation_data},
         m_logger{logger},
         m_max_roll_rate_radps{max_roll_rate_radps},
         m_max_pitch_rate_radps{max_pitch_rate_radps},
         m_max_yaw_rate_radps{max_yaw_rate_radps},
         m_max_tilt_angle_rad{max_tilt_angle_rad},
         m_idle_thrust{idle_thrust},
         m_max_thrust{max_thrust},
         m_lift_throttle{lift_throttle},
         m_thrust_model_factor{thrust_model_factor}

   {
      error::verify(0.0f <= idle_thrust);
      error::verify(1.0f >= max_thrust);

      m_logger.enable();
   }

   void start()
   {
      if (m_local_control_health.state == aeromight_boundaries::ControlState::inactive)
      {
         get_time();
         move_to_disarmed();
         publish_health();
      }
   }

   void execute()
   {
      get_time();

      get_control_setpoints();

      run_state_machine();

      publish_health();
   }

   aeromight_boundaries::ControlState get_state() const
   {
      return m_local_control_health.state;
   }

private:
   void run_state_machine()
   {
      switch (m_local_control_health.state)
      {
         case aeromight_boundaries::ControlState::inactive:
            // do nothing
            break;

         case aeromight_boundaries::ControlState::disarmed:
            if (armed())
            {
               move_to_armed();
            }
            break;

         case aeromight_boundaries::ControlState::armed_on_ground:
            if (kill())
            {
               move_to_killed();
               break;
            }

            if (!armed())
            {
               move_to_disarmed();
               break;
            }

            if (airborne())
            {
               move_to_airborne();
            }

            run_control(false);
            break;

         case aeromight_boundaries::ControlState::airborne:
            if (kill())
            {
               move_to_killed();
               break;
            }

            if (!armed())
            {
               move_to_armed_on_ground();
            }

            run_control(true);
            break;

         case aeromight_boundaries::ControlState::emergency_kill:
            if (!kill())
            {
               move_to_disarmed();
            }
            break;

         default:
            error::stop_operation();
            break;
      }
   }

   void get_time()
   {
      m_current_time_ms        = ClockSource::now_ms();
      m_time_delta_s           = static_cast<float>(m_current_time_ms - m_last_execution_time_ms) / 1000.0f;
      m_last_execution_time_ms = m_current_time_ms;
   }

   void get_control_setpoints()
   {
      m_last_control_setpoints = m_control_setpoints_storage.get_latest();
   }

   void move_to_killed()
   {
      reset();
      stop_actuator();
      publish_actuator_setpoints();
      m_local_control_health.state = aeromight_boundaries::ControlState::emergency_kill;
      m_logger.print("killed");
   }

   void move_to_disarmed()
   {
      reset();
      stop_actuator();
      publish_actuator_setpoints();
      m_local_control_health.state = aeromight_boundaries::ControlState::disarmed;
      m_logger.print("disarmed");
   }

   void move_to_armed()
   {
      reset();
      start_actuator();
      m_local_control_health.state = aeromight_boundaries::ControlState::armed_on_ground;
      m_logger.print("armed on ground");
   }

   void move_to_armed_on_ground()
   {
      reset();
      m_local_control_health.state = aeromight_boundaries::ControlState::armed_on_ground;
      m_logger.print("armed on ground");
   }

   void move_to_airborne()
   {
      reset();
      m_local_control_health.state = aeromight_boundaries::ControlState::airborne;
      m_logger.print("airborne");
   }

   void run_control(const bool run_integrator)
   {
      estimate_collective_thrust();
      run_attitude_controller();

      if (m_time_delta_s <= 0.0f)   // can't run rate controller
      {
         m_local_control_health.error.set(static_cast<uint8_t>(aeromight_boundaries::ControlHealth::Error::invalid_time_delta));
         return;
      }

      const math::Euler torque_cmd{run_rate_controller(run_integrator)};
      const auto        scaled_torque_deltas = estimate_torque_saturation(torque_cmd);

      run_mixer(scaled_torque_deltas);
      publish_actuator_setpoints();

      if ((m_counter++ % 250) == 0)
      {
         m_logger.printf("r=%.2f, p=%.2f, y=%.2f, t=%.2f, r=%.2f, p=%.2f, y=%.2f, 1=%.2f, 2=%.2f, 3=%.2f, 4=%.2f",
                         m_state_estimation_data.euler.roll(),
                         m_state_estimation_data.euler.pitch(),
                         m_state_estimation_data.euler.yaw(),
                         m_collective_thrust,
                         torque_cmd.roll(),
                         torque_cmd.pitch(),
                         torque_cmd.yaw(),
                         m_local_actuator_setpoints.torque.m1,
                         m_local_actuator_setpoints.torque.m2,
                         m_local_actuator_setpoints.torque.m3,
                         m_local_actuator_setpoints.torque.m4);
      }
   }

   void estimate_collective_thrust()
   {
      const float thrust_signal = m_idle_thrust + (m_last_control_setpoints.data.throttle * (1.0f - m_idle_thrust));
      m_collective_thrust       = (m_thrust_model_factor * (thrust_signal * thrust_signal)) + ((1.0f - m_thrust_model_factor) * thrust_signal);
   }

   void run_attitude_controller()
   {
      const math::Vector3 angle_setpoint_rad   = {m_last_control_setpoints.data.roll * m_max_tilt_angle_rad,
                                                  m_last_control_setpoints.data.pitch * m_max_tilt_angle_rad,
                                                  0.0f};
      const math::Vector3 angle_estimation_rad = {m_state_estimation_data.euler.roll(),
                                                  m_state_estimation_data.euler.pitch(),
                                                  m_state_estimation_data.euler.yaw()};

      m_desired_rate_radps.set(m_attitude_controller.update(angle_setpoint_rad, angle_estimation_rad));

      // yaw always rate-controlled
      m_desired_rate_radps.yaw(m_last_control_setpoints.data.yaw * m_max_yaw_rate_radps);
   }

   math::Vector3 run_rate_controller(const bool run_integrator)
   {
      return m_rate_controller.update(m_desired_rate_radps.vector(), m_state_estimation_data.gyro_radps, m_time_delta_s, run_integrator);
   }

   aeromight_boundaries::ActuatorSetpoints::Torque estimate_torque_saturation(const math::Euler& torque_cmd)
   {
      const float roll  = torque_cmd.roll();
      const float pitch = torque_cmd.pitch();
      const float yaw   = torque_cmd.yaw();

      aeromight_boundaries::ActuatorSetpoints::Torque torque_deltas{
          torque_deltas.m1 = roll + pitch + yaw,    // front-left CCW
          torque_deltas.m2 = -roll + pitch - yaw,   // front-right CW
          torque_deltas.m3 = -roll - pitch + yaw,   // rear-right CCW
          torque_deltas.m4 = roll - pitch - yaw,    // rear-left CW
      };

      // torque delta bounds
      const float upper_margin = m_max_thrust - m_collective_thrust;
      const float lower_margin = m_idle_thrust - m_collective_thrust;

      const float max_delta_d1 = std::max(torque_deltas.m1, torque_deltas.m2);
      const float max_delta_d2 = std::max(torque_deltas.m3, torque_deltas.m4);
      const float max_delta    = std::max(max_delta_d1, max_delta_d2);

      const float min_delta_d1 = std::min(torque_deltas.m1, torque_deltas.m2);
      const float min_delta_d2 = std::min(torque_deltas.m3, torque_deltas.m4);
      const float min_delta    = std::min(min_delta_d1, min_delta_d2);

      // scale
      float scale = 1.0f;

      if (upper_margin < max_delta)
      {
         scale = std::min(scale, (upper_margin / max_delta));
      }

      if (min_delta < lower_margin)
      {
         scale = std::min(scale, (lower_margin / min_delta));
      }

      aeromight_boundaries::ActuatorSetpoints::Torque scaled_torque_deltas{torque_deltas.m1 * scale,
                                                                           torque_deltas.m2 * scale,
                                                                           torque_deltas.m3 * scale,
                                                                           torque_deltas.m4 * scale};

      const float headroom_up_m1 = upper_margin - scaled_torque_deltas.m1;
      const float headroom_up_m2 = upper_margin - scaled_torque_deltas.m2;
      const float headroom_up_m3 = upper_margin - scaled_torque_deltas.m3;
      const float headroom_up_m4 = upper_margin - scaled_torque_deltas.m4;

      const float headroom_down_m1 = scaled_torque_deltas.m1 - lower_margin;
      const float headroom_down_m2 = scaled_torque_deltas.m2 - lower_margin;
      const float headroom_down_m3 = scaled_torque_deltas.m3 - lower_margin;
      const float headroom_down_m4 = scaled_torque_deltas.m4 - lower_margin;

      static constexpr float eps = 1e-6f;

      // check fully saturated movements
      const bool left_side_up    = (headroom_up_m1 <= eps) || (headroom_up_m4 <= eps);
      const bool right_side_down = (headroom_down_m2 <= eps) || (headroom_down_m3 <= eps);
      const bool left_side_down  = (headroom_down_m1 <= eps) || (headroom_down_m4 <= eps);
      const bool right_side_up   = (headroom_up_m2 <= eps) || (headroom_up_m3 <= eps);

      const bool nose_up   = (headroom_up_m1 <= eps) || (headroom_up_m2 <= eps);
      const bool tail_down = (headroom_down_m3 <= eps) || (headroom_down_m4 <= eps);
      const bool nose_down = (headroom_down_m1 <= eps) || (headroom_down_m2 <= eps);
      const bool tail_up   = (headroom_up_m3 <= eps) || (headroom_up_m4 <= eps);

      const bool nose_right = (headroom_up_m1 <= eps) || (headroom_up_m3 <= eps);
      const bool tail_left  = (headroom_down_m2 <= eps) || (headroom_down_m4 <= eps);
      const bool nose_left  = (headroom_up_m2 <= eps) || (headroom_up_m4 <= eps);
      const bool tail_right = (headroom_down_m1 <= eps) || (headroom_down_m3 <= eps);

      m_control_saturation_positive[0] = left_side_up && right_side_down;
      m_control_saturation_negative[0] = left_side_down && right_side_up;

      m_control_saturation_positive[1] = nose_up && tail_down;
      m_control_saturation_negative[1] = nose_down && tail_up;

      m_control_saturation_positive[2] = nose_right && tail_left;
      m_control_saturation_negative[2] = nose_left && tail_right;

      m_rate_controller.set_saturation_status(m_control_saturation_positive, m_control_saturation_negative);

      return scaled_torque_deltas;
   }

   void run_mixer(const aeromight_boundaries::ActuatorSetpoints::Torque& torque_deltas)
   {
      m_local_actuator_setpoints.torque.m1 = m_collective_thrust + torque_deltas.m1;
      m_local_actuator_setpoints.torque.m2 = m_collective_thrust + torque_deltas.m2;
      m_local_actuator_setpoints.torque.m3 = m_collective_thrust + torque_deltas.m3;
      m_local_actuator_setpoints.torque.m4 = m_collective_thrust + torque_deltas.m4;

      // apply final clamping
      m_local_actuator_setpoints.torque.m1 = std::clamp(m_local_actuator_setpoints.torque.m1, m_idle_thrust, m_max_thrust);
      m_local_actuator_setpoints.torque.m2 = std::clamp(m_local_actuator_setpoints.torque.m2, m_idle_thrust, m_max_thrust);
      m_local_actuator_setpoints.torque.m3 = std::clamp(m_local_actuator_setpoints.torque.m3, m_idle_thrust, m_max_thrust);
      m_local_actuator_setpoints.torque.m4 = std::clamp(m_local_actuator_setpoints.torque.m4, m_idle_thrust, m_max_thrust);
   }

   void start_actuator()
   {
      m_local_actuator_setpoints.enabled = true;
   }

   void stop_actuator()
   {
      m_local_actuator_setpoints.enabled = false;
   }

   void publish_actuator_setpoints()
   {
      m_actuator_setpoints_storage.update_latest(m_local_actuator_setpoints, m_current_time_ms);
   }

   void publish_health()
   {
      m_control_health_storage.update_latest(m_local_control_health, m_current_time_ms);
   }

   void reset()
   {
      m_desired_rate_radps.zero();
      m_rate_controller.reset();
      m_collective_thrust        = 0.0f;
      m_local_actuator_setpoints = {};
      m_control_saturation_positive.fill(false);
      m_control_saturation_negative.fill(false);
   }

   bool armed() const
   {
      return m_last_control_setpoints.data.armed;
   }

   bool kill() const
   {
      return m_last_control_setpoints.data.kill;
   }

   bool airborne() const
   {
      return (m_last_control_setpoints.data.throttle > m_lift_throttle);
   }

   AttitudeController&                        m_attitude_controller;
   RateController&                            m_rate_controller;
   ActuatorSetpoints&                         m_actuator_setpoints_storage;
   ControlHealth&                             m_control_health_storage;
   const ControlSetpoints&                    m_control_setpoints_storage;
   const StateEstimation&                     m_state_estimation_data;
   Logger&                                    m_logger;
   const float                                m_max_roll_rate_radps;
   const float                                m_max_pitch_rate_radps;
   const float                                m_max_yaw_rate_radps;
   const float                                m_max_tilt_angle_rad;
   const float                                m_idle_thrust;
   const float                                m_max_thrust;
   const float                                m_lift_throttle;
   const float                                m_thrust_model_factor;
   aeromight_boundaries::ActuatorSetpoints    m_local_actuator_setpoints{};
   aeromight_boundaries::ControlHealth        m_local_control_health{};
   ControlSetpoints::Sample                   m_last_control_setpoints{};
   math::Euler                                m_desired_rate_radps{};
   std::array<bool, RateController::num_axis> m_control_saturation_positive{};
   std::array<bool, RateController::num_axis> m_control_saturation_negative{};
   float                                      m_collective_thrust{};
   float                                      m_time_delta_s{0.0f};
   uint32_t                                   m_current_time_ms{0};
   uint32_t                                   m_last_execution_time_ms{0};

   uint32_t m_counter{};
};

}   // namespace aeromight_control
