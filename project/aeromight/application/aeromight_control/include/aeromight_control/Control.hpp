#pragma once

#include "StateEstimation.hpp"
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
   using ControlHealth = boundaries::SharedData<aeromight_boundaries::ControlHealth>;
   using Setpoints     = boundaries::SharedData<aeromight_boundaries::ControlSetpoints>;

public:
   explicit Control(AttitudeController&    attitude_controller,
                    RateController&        rate_controller,
                    ControlHealth&         control_health_storage,
                    const Setpoints&       control_setpoints_storage,
                    const StateEstimation& state_estimation_data,
                    Logger&                logger,
                    const float            max_roll_rate_radps,
                    const float            max_pitch_rate_radps,
                    const float            max_yaw_rate_radps,
                    const float            max_tilt_angle_rad,
                    const float            lift_throttle)
       : m_attitude_controller{attitude_controller},
         m_rate_controller{rate_controller},
         m_control_health_storage{control_health_storage},
         m_control_setpoints_storage{control_setpoints_storage},
         m_state_estimation_data{state_estimation_data},
         m_logger{logger},
         m_max_roll_rate_radps{max_roll_rate_radps},
         m_max_pitch_rate_radps{max_pitch_rate_radps},
         m_max_yaw_rate_radps{max_yaw_rate_radps},
         m_max_tilt_angle_rad{max_tilt_angle_rad},
         m_lift_throttle{lift_throttle}

   {
      m_logger.enable();
   }

   void start()
   {
      if (m_local_control_health.state == aeromight_boundaries::ControlState::inactive)
      {
         get_time();
         m_local_control_health.state = aeromight_boundaries::ControlState::disarmed;
         publish_health();
         m_logger.print("started control");
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
            }

            if (!armed())
            {
               move_to_disarmed();
            }

            if (airborne())
            {
               move_to_airborne();
            }

            run_attitude_controller();
            run_rate_controller(false);
            break;

         case aeromight_boundaries::ControlState::airborne:
            if (kill())
            {
               move_to_killed();
            }

            if (!armed())
            {
               move_to_armed_on_ground();
            }

            run_attitude_controller();
            run_rate_controller(true);
            break;

         case aeromight_boundaries::ControlState::emergency_kill:
            // do nothing
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

   void reset()
   {
      m_rate_controller.reset();
   }

   void move_to_killed()
   {
      reset();
      stop_motors();
      m_logger.print("killed");
      m_local_control_health.state = aeromight_boundaries::ControlState::emergency_kill;
   }

   void move_to_disarmed()
   {
      reset();
      stop_motors();
      m_logger.print("stopped motors");
      m_local_control_health.state = aeromight_boundaries::ControlState::disarmed;
   }

   void move_to_armed()
   {
      reset();
      start_motors();
      m_logger.print("started motors");
      m_local_control_health.state = aeromight_boundaries::ControlState::armed_on_ground;
   }

   void move_to_armed_on_ground()
   {
      reset();
      m_logger.print("landed");
      m_local_control_health.state = aeromight_boundaries::ControlState::armed_on_ground;
   }

   void move_to_airborne()
   {
      m_logger.print("airborne");
      m_local_control_health.state = aeromight_boundaries::ControlState::airborne;
   }

   void run_attitude_controller()
   {
      if (m_attitude_controller_to_be_updated)
      {
         const math::Vector3 angle_setpoint_rad   = {m_last_control_setpoints.data.roll * m_max_tilt_angle_rad, m_last_control_setpoints.data.pitch * m_max_tilt_angle_rad, 0.0f};
         const math::Vector3 angle_estimation_rad = {m_state_estimation_data.euler.x, m_state_estimation_data.euler.y, m_state_estimation_data.euler.z};

         m_desired_rate_radps = m_attitude_controller.update(angle_setpoint_rad, angle_estimation_rad);
      }

      // yaw always rate-controlled
      m_desired_rate_radps.z = m_last_control_setpoints.data.yaw * m_max_yaw_rate_radps;

      m_attitude_controller_to_be_updated = !m_attitude_controller_to_be_updated;
   }

   void run_rate_controller(const bool run_integrator)
   {
      if (m_time_delta_s <= 0.0f)   // can't run rate controller
      {
         m_last_execution_time_ms = m_current_time_ms;
         m_local_control_health.error.set(static_cast<uint8_t>(aeromight_boundaries::ControlHealth::Error::invalid_time_delta));
         return;
      }

      m_torque_cmd = m_rate_controller.update(m_desired_rate_radps, m_state_estimation_data.gyro_radps, m_time_delta_s, run_integrator);

      if ((m_counter++ % 250) == 0)
      {
         m_logger.printf("torque x=%.2f, y=%.2f, z=%.2f, throt=%0.2f", m_torque_cmd.x, m_torque_cmd.y, m_torque_cmd.z, m_last_control_setpoints.data.throttle);
      }
   }

   // cppcheck-suppress functionStatic
   void start_motors()
   {
      // throttle is clamp of collective thrust and min idle
   }

   // cppcheck-suppress functionStatic
   void stop_motors()
   {
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

   AttitudeController&                 m_attitude_controller;
   RateController&                     m_rate_controller;
   ControlHealth&                      m_control_health_storage;
   const Setpoints&                    m_control_setpoints_storage;
   const StateEstimation&              m_state_estimation_data;
   Logger&                             m_logger;
   const float                         m_max_roll_rate_radps;
   const float                         m_max_pitch_rate_radps;
   const float                         m_max_yaw_rate_radps;
   const float                         m_max_tilt_angle_rad;
   const float                         m_lift_throttle;
   aeromight_boundaries::ControlHealth m_local_control_health{};
   Setpoints::Sample                   m_last_control_setpoints{};
   uint32_t                            m_current_time_ms{0};
   uint32_t                            m_last_execution_time_ms{0};
   float                               m_time_delta_s{0.0f};
   bool                                m_attitude_controller_to_be_updated{false};
   math::Vector3                       m_desired_rate_radps{};
   math::Vector3                       m_torque_cmd{};

   uint32_t m_counter{};
};

}   // namespace aeromight_control
