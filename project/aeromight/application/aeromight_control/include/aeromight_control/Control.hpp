#pragma once

#include <algorithm>

#include "aeromight_boundaries/ActuatorSetpoints.hpp"
#include "aeromight_boundaries/ControlStatus.hpp"
#include "aeromight_boundaries/FlightControlSetpoints.hpp"
#include "aeromight_boundaries/StateEstimation.hpp"
#include "aeromight_boundaries/SystemStateInfo.hpp"
#include "boundaries/SharedData.hpp"
#include "control/Motor.hpp"
#include "interfaces/IClockSource.hpp"
#include "math/Vector2.hpp"

namespace aeromight_control
{

template <typename AttitudeController,
          typename RateController,
          typename ControlAllocator,
          typename ControlInput,
          typename FirstOrderLpf,
          typename ButterworthLpf2,
          interfaces::IClockSource ClockSource,
          typename Logger>
class Control
{
   using ActuatorControlPublisher  = boundaries::SharedData<aeromight_boundaries::ActuatorControl>;
   using ControlHealthPublisher    = boundaries::SharedData<aeromight_boundaries::ControlStatus>;
   using SystemStateInfoSubscriber = boundaries::SharedData<aeromight_boundaries::SystemStateInfo>;
   using ControlAxis               = aeromight_boundaries::ControlAxis;

public:
   static constexpr std::size_t num_axis = 3u;

   explicit Control(AttitudeController&                          attitude_controller,
                    RateController&                              rate_controller,
                    ControlAllocator&                            control_allocator,
                    ControlInput&                                control_input,
                    FirstOrderLpf&                               roll_input_lpf,
                    FirstOrderLpf&                               pitch_input_lpf,
                    FirstOrderLpf&                               yaw_input_lpf,
                    FirstOrderLpf&                               gyro_x_lpf,
                    FirstOrderLpf&                               gyro_y_lpf,
                    FirstOrderLpf&                               gyro_z_lpf,
                    ButterworthLpf2&                             pid_dterm_x_lpf,
                    ButterworthLpf2&                             pid_dterm_y_lpf,
                    ButterworthLpf2&                             pid_dterm_z_lpf,
                    ActuatorControlPublisher&                    actuator_control_publisher,
                    ControlHealthPublisher&                      control_health_publisher,
                    const SystemStateInfoSubscriber&             system_state_info_subscriber,
                    const aeromight_boundaries::StateEstimation& state_estimation_subscriber,
                    Logger&                                      logger,
                    const uint32_t                               max_age_state_estimation_data_ms,
                    const float                                  min_dt_s,
                    const float                                  max_dt_s,
                    const bool                                   run_attitude_controller,
                    const float                                  max_tilt_angle_rad,
                    const math::Vector3                          max_rate_radps,
                    const float                                  actuator_min,
                    const float                                  actuator_max,
                    const float                                  throttle_arming,
                    const float                                  throttle_gate_integrator,
                    const float                                  thrust_linearization_factor)
       : m_attitude_controller{attitude_controller},
         m_rate_controller{rate_controller},
         m_control_allocator{control_allocator},
         m_control_input{control_input},
         m_stick_input_lpf{roll_input_lpf, pitch_input_lpf, yaw_input_lpf},
         m_gyro_lpf{gyro_x_lpf, gyro_y_lpf, gyro_z_lpf},
         m_pid_dterm_lpf{pid_dterm_x_lpf, pid_dterm_y_lpf, pid_dterm_z_lpf},
         m_actuator_control_publisher{actuator_control_publisher},
         m_control_health_publisher{control_health_publisher},
         m_system_state_info_subscriber{system_state_info_subscriber},
         m_state_estimation_subscriber{state_estimation_subscriber},
         m_logger{logger},
         m_max_age_state_estimation_data_ms{max_age_state_estimation_data_ms},
         m_min_dt_s{min_dt_s},
         m_max_dt_s{max_dt_s},
         m_run_attitude_controller{run_attitude_controller},
         m_max_tilt_angle_rad{max_tilt_angle_rad},
         m_max_rate_radps{max_rate_radps},
         m_actuator_min{actuator_min},
         m_actuator_max{actuator_max},
         m_throttle_arming{throttle_arming},
         m_throttle_gate_integrator{throttle_gate_integrator},
         m_thrust_linearization_factor{thrust_linearization_factor}
   {
      m_logger.enable();
   }

   void start()
   {
      m_control_status.enabled = true;
      get_time();
      publish_health();
      m_logger.print("started");
   }

   void execute()
   {
      if (!m_control_status.enabled)
      {
         return;
      }

      get_time();

      get_system_state_info();

      get_flight_control_setpoints();

      get_state_estimation();

      m_control_status.error.reset();

      if (!is_state_estimation_valid())
      {
         reset_everything();
         m_control_status.error.set(static_cast<types::ErrorBitsType>(aeromight_boundaries::ControlStatus::Error::invalid_estimation_data));
      }
      else   // run control
      {
         get_rate_setpoints();

         get_torque_setpoints();

         update_actuator_setpoints();

         if (m_actuator_control.enabled)
         {
            if (!m_system_state_info.armed)
            {
               reset_everything();
               m_logger.print("disarmed");
            }
         }
         else
         {
            if (m_system_state_info.armed && (m_flight_control_setpoints.throttle < m_throttle_arming))
            {
               reset();
               reset_filters();
               m_actuator_control.enabled = true;
               m_logger.print("armed");
            }

            m_actuator_control.setpoints.zero();
         }
      }

      publish_actuator_setpoints();

      publish_health();

      print_log();
   }

   bool is_enabled() const
   {
      return m_control_status.enabled;
   }

private:
   void get_time()
   {
      m_last_execution_time_us = m_current_time_us;
      m_current_time_us        = ClockSource::now_us();
      m_current_time_ms        = m_current_time_us / 1000u;

      m_dt_s = static_cast<float>(m_current_time_us - m_last_execution_time_us) * 0.000001f;
      m_dt_s = std::clamp(m_dt_s, m_min_dt_s, m_max_dt_s);
   }

   void get_system_state_info()
   {
      m_system_state_info = m_system_state_info_subscriber.get_latest().data;
   }

   void get_flight_control_setpoints()
   {
      m_flight_control_setpoints = m_control_input.get_flight_control_setpoints();
   }

   void get_state_estimation()
   {
      m_state_estimation = m_state_estimation_subscriber;
   }

   void get_rate_setpoints()
   {
      using Axis = aeromight_boundaries::ControlAxis;

      if (m_run_attitude_controller)
      {
         math::Vector2 manual_setpoints{m_stick_input_lpf[Axis::roll].get().apply(m_flight_control_setpoints.roll * m_max_tilt_angle_rad, m_dt_s),
                                        m_stick_input_lpf[Axis::pitch].get().apply(m_flight_control_setpoints.pitch * m_max_tilt_angle_rad, m_dt_s)};

         const float tilt_norm = manual_setpoints.norm();

         if (tilt_norm > m_max_tilt_angle_rad)
         {
            manual_setpoints *= (m_max_tilt_angle_rad / tilt_norm);
         }

         const math::Vector3 angle_estimation_rad{m_state_estimation.euler.roll(),
                                                  m_state_estimation.euler.pitch(),
                                                  m_state_estimation.euler.yaw()};

         const math::Vector3 angle_setpoints{manual_setpoints[Axis::roll], manual_setpoints[Axis::pitch], 0.0f};

         m_angular_rate_setpoints = m_attitude_controller.update(angle_setpoints, angle_estimation_rad);

         m_angular_rate_setpoints[Axis::roll]  = std::clamp(m_angular_rate_setpoints[Axis::roll], -m_max_rate_radps[Axis::roll], m_max_rate_radps[Axis::roll]);
         m_angular_rate_setpoints[Axis::pitch] = std::clamp(m_angular_rate_setpoints[Axis::pitch], -m_max_rate_radps[Axis::pitch], m_max_rate_radps[Axis::pitch]);
      }
      else   // generate rate setpoints from sticks
      {
         m_angular_rate_setpoints[Axis::roll]  = m_stick_input_lpf[Axis::roll].get().apply(m_flight_control_setpoints.roll * m_max_rate_radps[Axis::roll], m_dt_s);
         m_angular_rate_setpoints[Axis::pitch] = m_stick_input_lpf[Axis::pitch].get().apply(m_flight_control_setpoints.pitch * m_max_rate_radps[Axis::pitch], m_dt_s);
      }

      // yaw rate from manual setpoint
      m_angular_rate_setpoints[Axis::yaw] = m_stick_input_lpf[Axis::yaw].get().apply(m_flight_control_setpoints.yaw * m_max_rate_radps[Axis::yaw], m_dt_s);
   }

   void get_torque_setpoints()
   {
      // gyro LPF
      for (std::size_t i = 0; i < num_axis; i++)
      {
         m_gyro_radps[i] = m_gyro_lpf[i].get().apply(m_state_estimation.raw_gyro_radps[i], m_dt_s);
      }

      // PID d-term LPF
      m_previous_dterm_gyro_radps = m_dterm_gyro_radps;
      for (std::size_t i = 0; i < num_axis; i++)
      {
         m_dterm_gyro_radps[i] = m_pid_dterm_lpf[i].get().apply(m_gyro_radps[i]);
      }

      const math::Vector3 angular_acceleration{(m_dterm_gyro_radps - m_previous_dterm_gyro_radps) / m_dt_s};

      const bool run_integrator{m_system_state_info.armed &&
                                (m_flight_control_setpoints.throttle > m_throttle_gate_integrator)};

      m_torque_setpoints = m_rate_controller.update(m_angular_rate_setpoints,
                                                    m_gyro_radps,
                                                    angular_acceleration,
                                                    m_dt_s,
                                                    run_integrator);
   }

   void update_actuator_setpoints()
   {
      const math::Vector4 control_setpoints{m_torque_setpoints[ControlAxis::roll],
                                            m_torque_setpoints[ControlAxis::pitch],
                                            m_torque_setpoints[ControlAxis::yaw],
                                            m_flight_control_setpoints.throttle};

      m_control_allocator.set_control_setpoints(control_setpoints);

      m_control_allocator.allocate();

      m_control_allocator.clip_actuator_setpoints();

      m_actuator_control.setpoints = m_control_allocator.get_actuator_setpoints();

      // determine allocator saturation
      m_control_allocator.estimate_saturation();
      m_rate_controller.set_saturation_status(m_control_allocator.get_actuator_saturation_positive(), m_control_allocator.get_actuator_saturation_negative());

      control::Motor::apply_thrust_linearization(m_actuator_control.setpoints, m_thrust_linearization_factor, m_actuator_min, m_actuator_max);
   }

   void publish_actuator_setpoints()
   {
      m_actuator_control_publisher.update_latest(m_actuator_control, m_current_time_ms);
   }

   void publish_health()
   {
      m_control_health_publisher.update_latest(m_control_status, m_current_time_ms);
   }

   void reset_everything()
   {
      reset();
      reset_filters();
      m_actuator_control.enabled = false;
      m_actuator_control.setpoints.zero();
   }

   void reset()
   {
      m_angular_rate_setpoints.zero();
      m_torque_setpoints.zero();
      m_rate_controller.reset();
      m_control_allocator.reset();
   }

   void reset_filters()
   {
      for (auto& filter : m_stick_input_lpf)
      {
         filter.get().reset();
      }

      for (auto& filter : m_gyro_lpf)
      {
         filter.get().reset();
      }

      for (auto& filter : m_pid_dterm_lpf)
      {
         filter.get().reset();
      }
   }

   void print_log()
   {
      static uint32_t m_counter{0};
      if ((m_counter++ % 125) == 0)
      {
         m_logger.printf("acc %.2f %.2f %.2f | gyr %.2f %.2f %.2f | est %.3f %.3f %.3f | rate %.2f %.2f %.2f | trq %.4f %.4f %.4f | t %.2f | pwm %.4f %.4f %.4f %.4f",
                         m_state_estimation.raw_accel_mps2[0],
                         m_state_estimation.raw_accel_mps2[1],
                         m_state_estimation.raw_accel_mps2[2],
                         m_state_estimation.raw_gyro_radps[0],
                         m_state_estimation.raw_gyro_radps[1],
                         m_state_estimation.raw_gyro_radps[2],
                         m_state_estimation.euler.roll(),
                         m_state_estimation.euler.pitch(),
                         m_state_estimation.euler.yaw(),
                         m_angular_rate_setpoints[0],
                         m_angular_rate_setpoints[1],
                         m_angular_rate_setpoints[2],
                         m_torque_setpoints[0],
                         m_torque_setpoints[1],
                         m_torque_setpoints[2],
                         m_flight_control_setpoints.throttle,
                         m_actuator_control.setpoints[0],
                         m_actuator_control.setpoints[1],
                         m_actuator_control.setpoints[2],
                         m_actuator_control.setpoints[3]);
      }
   }

   bool is_state_estimation_valid() const
   {
      const auto state_estimation_data_age_ms = m_current_time_ms - m_state_estimation.timestamp_ms;
      return (m_state_estimation.attitude_estimation_valid && (state_estimation_data_age_ms <= m_max_age_state_estimation_data_ms));
   }

   AttitudeController&                                           m_attitude_controller;
   RateController&                                               m_rate_controller;
   ControlAllocator&                                             m_control_allocator;
   ControlInput&                                                 m_control_input;
   std::array<std::reference_wrapper<FirstOrderLpf>, num_axis>   m_stick_input_lpf;
   std::array<std::reference_wrapper<FirstOrderLpf>, num_axis>   m_gyro_lpf;
   std::array<std::reference_wrapper<ButterworthLpf2>, num_axis> m_pid_dterm_lpf;
   ActuatorControlPublisher&                                     m_actuator_control_publisher;
   ControlHealthPublisher&                                       m_control_health_publisher;
   const SystemStateInfoSubscriber&                              m_system_state_info_subscriber;
   const aeromight_boundaries::StateEstimation&                  m_state_estimation_subscriber;
   Logger&                                                       m_logger;
   const uint32_t                                                m_max_age_state_estimation_data_ms;
   const float                                                   m_min_dt_s;
   const float                                                   m_max_dt_s;
   const bool                                                    m_run_attitude_controller;
   const float                                                   m_max_tilt_angle_rad;
   const math::Vector3                                           m_max_rate_radps;
   const float                                                   m_actuator_min;
   const float                                                   m_actuator_max;
   const float                                                   m_throttle_arming;
   const float                                                   m_throttle_gate_integrator;
   const float                                                   m_thrust_linearization_factor;
   aeromight_boundaries::ActuatorControl                         m_actuator_control{};
   aeromight_boundaries::ControlStatus                           m_control_status{};
   aeromight_boundaries::FlightControlSetpoints                  m_flight_control_setpoints{};
   aeromight_boundaries::SystemStateInfo                         m_system_state_info{};
   aeromight_boundaries::StateEstimation                         m_state_estimation{};
   math::Vector3                                                 m_gyro_radps{};
   math::Vector3                                                 m_dterm_gyro_radps{};
   math::Vector3                                                 m_previous_dterm_gyro_radps{};
   math::Vector3                                                 m_angular_rate_setpoints{};
   math::Vector3                                                 m_torque_setpoints{};
   float                                                         m_dt_s{0.0f};
   uint32_t                                                      m_current_time_ms{0};
   uint32_t                                                      m_current_time_us{0};
   uint32_t                                                      m_last_execution_time_us{0};
};

}   // namespace aeromight_control
