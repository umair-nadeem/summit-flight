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
         m_lift_throttle{lift_throttle},
         m_thrust_model_factor{thrust_model_factor}

   {
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

            estimate_collective_thrust();
            run_attitude_controller();
            run_rate_controller(false);
            run_mixer();
            publish_actuator_setpoints();
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

            estimate_collective_thrust();
            run_attitude_controller();
            run_rate_controller(true);
            run_mixer();
            publish_actuator_setpoints();
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

   void publish_health()
   {
      m_control_health_storage.update_latest(m_local_control_health, m_current_time_ms);
   }

   void publish_actuator_setpoints()
   {
      m_actuator_setpoints_storage.update_latest(m_local_actuator_setpoints, m_current_time_ms);
   }

   void reset()
   {
      m_collective_thrust = 0.0f;
      m_desired_rate_radps.zero();
      m_torque_cmd.zero();
      m_rate_controller.reset();
      m_local_actuator_setpoints = {};
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

   void estimate_collective_thrust()
   {
      const float thurst_signal = m_idle_thrust + m_last_control_setpoints.data.throttle * (1.0f - m_idle_thrust);
      m_collective_thrust       = m_thrust_model_factor * (thurst_signal * thurst_signal) + (1.0f - m_thrust_model_factor) * thurst_signal;
   }

   void run_attitude_controller()
   {
      if (m_attitude_controller_to_be_updated)
      {
         const math::Vector3 angle_setpoint_rad   = {m_last_control_setpoints.data.roll * m_max_tilt_angle_rad,
                                                     m_last_control_setpoints.data.pitch * m_max_tilt_angle_rad,
                                                     0.0f};
         const math::Vector3 angle_estimation_rad = {m_state_estimation_data.euler.roll(),
                                                     m_state_estimation_data.euler.pitch(),
                                                     m_state_estimation_data.euler.yaw()};

         m_desired_rate_radps.set(m_attitude_controller.update(angle_setpoint_rad, angle_estimation_rad));
      }

      // yaw always rate-controlled
      m_desired_rate_radps.yaw(m_last_control_setpoints.data.yaw * m_max_yaw_rate_radps);

      m_attitude_controller_to_be_updated = !m_attitude_controller_to_be_updated;
   }

   void run_rate_controller(const bool run_integrator)
   {
      if (m_time_delta_s <= 0.0f)   // can't run rate controller
      {
         m_local_control_health.error.set(static_cast<uint8_t>(aeromight_boundaries::ControlHealth::Error::invalid_time_delta));
         return;
      }

      m_torque_cmd.set(m_rate_controller.update(m_desired_rate_radps.vector(), m_state_estimation_data.gyro_radps, m_time_delta_s, run_integrator));

      if ((m_counter++ % 250) == 0)
      {
         m_logger.printf("imu r=%.2f, p=%.2f, y=%.2f, t=%.2f, r=%.2f, p=%.2f, y=%.2f, %.2f, %.2f, %.2f, %.2f",
                         m_state_estimation_data.euler.roll(),
                         m_state_estimation_data.euler.pitch(),
                         m_state_estimation_data.euler.yaw(),
                         m_collective_thrust,
                         m_torque_cmd.roll(),
                         m_torque_cmd.pitch(),
                         m_torque_cmd.yaw(),
                         m_local_actuator_setpoints.m1,
                         m_local_actuator_setpoints.m2,
                         m_local_actuator_setpoints.m3,
                         m_local_actuator_setpoints.m4);
      }
   }

   void run_mixer()
   {
      m_local_actuator_setpoints.m1 = m_collective_thrust + m_torque_cmd.roll() + m_torque_cmd.pitch() + m_torque_cmd.yaw();   // front-left CCW
      m_local_actuator_setpoints.m2 = m_collective_thrust - m_torque_cmd.roll() + m_torque_cmd.pitch() - m_torque_cmd.yaw();   // front-right CW
      m_local_actuator_setpoints.m3 = m_collective_thrust - m_torque_cmd.roll() - m_torque_cmd.pitch() + m_torque_cmd.yaw();   // rear-right CCW
      m_local_actuator_setpoints.m4 = m_collective_thrust + m_torque_cmd.roll() - m_torque_cmd.pitch() - m_torque_cmd.yaw();   // rear-left CW
   }

   void start_actuator()
   {
      m_local_actuator_setpoints.enabled = true;
   }

   void stop_actuator()
   {
      m_local_actuator_setpoints.enabled = false;
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

   AttitudeController&                     m_attitude_controller;
   RateController&                         m_rate_controller;
   ActuatorSetpoints&                      m_actuator_setpoints_storage;
   ControlHealth&                          m_control_health_storage;
   const ControlSetpoints&                 m_control_setpoints_storage;
   const StateEstimation&                  m_state_estimation_data;
   Logger&                                 m_logger;
   const float                             m_max_roll_rate_radps;
   const float                             m_max_pitch_rate_radps;
   const float                             m_max_yaw_rate_radps;
   const float                             m_max_tilt_angle_rad;
   const float                             m_idle_thrust;
   const float                             m_lift_throttle;
   const float                             m_thrust_model_factor;
   aeromight_boundaries::ActuatorSetpoints m_local_actuator_setpoints{};
   aeromight_boundaries::ControlHealth     m_local_control_health{};
   ControlSetpoints::Sample                m_last_control_setpoints{};
   math::Euler                             m_desired_rate_radps{};
   math::Euler                             m_torque_cmd{};
   float                                   m_collective_thrust{};
   float                                   m_time_delta_s{0.0f};
   uint32_t                                m_current_time_ms{0};
   uint32_t                                m_last_execution_time_ms{0};
   bool                                    m_attitude_controller_to_be_updated{false};

   uint32_t m_counter{};
};

}   // namespace aeromight_control
