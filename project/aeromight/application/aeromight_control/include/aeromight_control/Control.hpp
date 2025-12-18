#pragma once

#include "StateEstimation.hpp"
#include "aeromight_boundaries/ControlHealth.hpp"
#include "aeromight_boundaries/ControlSetpoints.hpp"
#include "boundaries/SharedData.hpp"
#include "interfaces/IClockSource.hpp"

namespace aeromight_control
{

template <typename AttitudeController, typename RateController, interfaces::IClockSource ClockSource, typename Logger>
class Control
{
   using ControlHealth = ::boundaries::SharedData<aeromight_boundaries::ControlHealth>;
   using Setpoints     = boundaries::SharedData<aeromight_boundaries::ControlSetpoints>;

public:
   explicit Control(ControlHealth&         control_health_storage,
                    AttitudeController&    attitude_controller,
                    RateController&        rate_controller,
                    const Setpoints&       control_setpoints,
                    const StateEstimation& state_estimation_data,
                    Logger&                logger,
                    const float            max_roll_rate_radps,
                    const float            max_pitch_rate_radps,
                    const float            max_yaw_rate_radps,
                    const float            max_tilt_angle_rad)
       : m_control_health_storage{control_health_storage},
         m_attitude_controller{attitude_controller},
         m_rate_controller{rate_controller},
         m_control_setpoints{control_setpoints},
         m_state_estimation_data{state_estimation_data},
         m_logger{logger},
         m_max_roll_rate_radps{max_roll_rate_radps},
         m_max_pitch_rate_radps{max_pitch_rate_radps},
         m_max_yaw_rate_radps{max_yaw_rate_radps},
         m_max_tilt_angle_rad{max_tilt_angle_rad}

   {
      m_logger.enable();
   }

   void start()
   {
      get_time();
      m_rate_controller.reset();
      m_local_control_health.enabled = true;
      publish_health();
      m_logger.print("started control");
   }

   void stop()
   {
      get_time();
      m_rate_controller.reset();
      m_local_control_health.enabled = false;
      publish_health();
      m_logger.print("stopped control");
   }

   void execute()
   {
      get_time();

      if (!m_local_control_health.enabled)
      {
         m_last_execution_time_ms = m_current_time_ms;
         return;
      }

      const auto dt_s = static_cast<float>(m_current_time_ms - m_last_execution_time_ms) / 1000.0f;

      if (dt_s <= 0.0f)
      {
         m_last_execution_time_ms = m_current_time_ms;
         return;
      }

      const auto setpoints = m_control_setpoints.get_latest();

      if (m_attitude_controller_to_be_updated)
      {
         const math::Vector3 angle_setpoint_rad   = {setpoints.data.roll * m_max_tilt_angle_rad, setpoints.data.pitch * m_max_tilt_angle_rad, 0.0f};
         const math::Vector3 angle_estimation_rad = {m_state_estimation_data.euler.x, m_state_estimation_data.euler.y, m_state_estimation_data.euler.z};

         m_desired_rate_radps = m_attitude_controller.update(angle_setpoint_rad, angle_estimation_rad);
      }

      // yaw always rate-controlled
      m_desired_rate_radps.z = setpoints.data.yaw * m_max_yaw_rate_radps;

      m_torque_cmd = m_rate_controller.update(setpoints.data.throttle, m_desired_rate_radps, m_state_estimation_data.gyro_radps, dt_s);

      m_attitude_controller_to_be_updated = !m_attitude_controller_to_be_updated;
      m_last_execution_time_ms            = m_current_time_ms;

      publish_health();

      if ((m_counter++ % 250) == 0)
      {
         m_logger.printf("torque x=%.2f, y=%.2f, z=%.2f, throt=%0.2f", m_torque_cmd.x, m_torque_cmd.y, m_torque_cmd.z, setpoints.data.throttle);
      }
   }

private:
   void get_time()
   {
      m_current_time_ms = ClockSource::now_ms();
   }

   void publish_health()
   {
      m_control_health_storage.update_latest(m_local_control_health, m_current_time_ms);
   }

   ControlHealth&                      m_control_health_storage;
   AttitudeController&                 m_attitude_controller;
   RateController&                     m_rate_controller;
   const Setpoints&                    m_control_setpoints;
   const StateEstimation&              m_state_estimation_data;
   Logger&                             m_logger;
   const float                         m_max_roll_rate_radps;
   const float                         m_max_pitch_rate_radps;
   const float                         m_max_yaw_rate_radps;
   const float                         m_max_tilt_angle_rad;
   aeromight_boundaries::ControlHealth m_local_control_health{};
   uint32_t                            m_current_time_ms{0};
   uint32_t                            m_last_execution_time_ms{0};
   bool                                m_attitude_controller_to_be_updated{false};
   math::Vector3                       m_desired_rate_radps{};
   math::Vector3                       m_torque_cmd{};

   uint32_t m_counter{};
};

}   // namespace aeromight_control
