#pragma once

#include <algorithm>
#include <span>

#include "ControlParams.hpp"
#include "aeromight_boundaries/ControlAxis.hpp"
#include "aeromight_boundaries/ControlStatus.hpp"
#include "aeromight_boundaries/StateEstimation.hpp"
#include "aeromight_boundaries/SystemState.hpp"
#include "aeromight_boundaries/actuator.hpp"
#include "boundaries/SharedData.hpp"
#include "interfaces/IClockSource.hpp"
#include "math/Vector2.hpp"
#include "math/Vector4.hpp"
#include "motor/motor.hpp"
#include "rc/StickCommand.hpp"

namespace aeromight_control
{

template <typename AttitudeController,
          typename RateController,
          typename ControlAllocator,
          typename StickCommandSource,
          typename ActuatorControl,
          typename GyroFilter,
          typename DtermFilter,
          typename Led,
          interfaces::IClockSource ClockSource,
          typename Logger>
class Control
{
   using ControlHealthPublisher = boundaries::SharedData<aeromight_boundaries::ControlStatus>;
   using SystemStateSubscriber  = boundaries::SharedData<aeromight_boundaries::SystemState>;
   using Error                  = aeromight_boundaries::ControlStatus::Error;
   using Axis                   = aeromight_boundaries::ControlAxis;

public:
   static constexpr std::size_t num_axis      = aeromight_boundaries::num_axis;
   static constexpr std::size_t num_actuators = ControlAllocator::num_actuators;

   explicit Control(AttitudeController&                          attitude_controller,
                    RateController&                              rate_controller,
                    ControlAllocator&                            control_allocator,
                    StickCommandSource&                          stick_command_source,
                    ActuatorControl&                             actuator_control,
                    GyroFilter&                                  gyro_x_lpf,
                    GyroFilter&                                  gyro_y_lpf,
                    GyroFilter&                                  gyro_z_lpf,
                    DtermFilter&                                 pid_dterm_x_lpf,
                    DtermFilter&                                 pid_dterm_y_lpf,
                    DtermFilter&                                 pid_dterm_z_lpf,
                    Led&                                         led,
                    const math::Vec4<uint8_t>&                   motor_mapping,
                    ControlHealthPublisher&                      control_health_publisher,
                    const SystemStateSubscriber&                 system_state_subscriber,
                    const aeromight_boundaries::StateEstimation& state_estimation_subscriber,
                    Logger&                                      logger,
                    const ControlParams&                         params)
       : m_attitude_controller{attitude_controller},
         m_rate_controller{rate_controller},
         m_control_allocator{control_allocator},
         m_stick_command_source{stick_command_source},
         m_actuator_control{actuator_control},
         m_gyro_lpf{gyro_x_lpf, gyro_y_lpf, gyro_z_lpf},
         m_gyro_dterm_lpf{pid_dterm_x_lpf, pid_dterm_y_lpf, pid_dterm_z_lpf},
         m_led{led},
         m_motor_mapping{motor_mapping},
         m_control_health_publisher{control_health_publisher},
         m_system_state_subscriber{system_state_subscriber},
         m_state_estimation_subscriber{state_estimation_subscriber},
         m_logger{logger},
         m_params{params}
   {
      m_logger.enable();
   }

   void start()
   {
      m_control_status.enabled = true;
      get_time();
      publish_health();
      m_led.turn_off();
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

      get_stick_command();

      get_state_estimation();

      m_control_status.error.reset();

      if (!is_state_estimation_valid())
      {
         m_control_status.error.set(static_cast<types::ErrorBitsType>(Error::invalid_estimation_sample));
         reset();
         m_armed = false;
         m_led.toggle(control_led_period_ms);
      }
      else
      {
         update_attitude_control();

         update_rate_control();

         update_control_allocation();

         if (m_armed)
         {
            m_led.turn_on();

            if (!m_system_state_setpoints.armed)
            {
               reset();
               m_armed = false;
               m_logger.print("disarmed");
            }
         }
         else   // disarmed
         {
            if (m_system_state_setpoints.armed)
            {
               if (m_stick_command.throttle < m_params.throttle_arming)
               {
                  m_armed = true;
                  m_logger.print("armed");
               }
               else
               {
                  m_led.toggle(control_led_period_ms);
               }
            }
            else
            {
               m_led.turn_off();
            }

            reset();
         }
      }

      update_actuator();

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
      if ((m_dt_s < m_params.min_dt_s) || (m_params.max_dt_s < m_dt_s))
      {
         m_dt_s = std::clamp(m_dt_s, m_params.min_dt_s, m_params.max_dt_s);
         m_control_status.error.set(static_cast<types::ErrorBitsType>(Error::timing_jitter));
      }
   }

   void get_system_state_info()
   {
      m_system_state_setpoints = m_system_state_subscriber.get_latest().data;
   }

   void get_stick_command()
   {
      const auto raw_stick_command = m_stick_command_source.get_raw();
      m_stick_command              = m_stick_command_source.apply_filter(raw_stick_command, m_dt_s);
   }

   void get_state_estimation()
   {
      m_state_estimation = m_state_estimation_subscriber;
   }

   void update_attitude_control()
   {
      using namespace aeromight_boundaries;

      if (m_params.run_attitude_controller)
      {
         math::Vec2f stick_setpoints{m_stick_command.roll * m_params.max_tilt_angle_rad, m_stick_command.pitch * m_params.max_tilt_angle_rad};

         const float tilt_norm = stick_setpoints.norm();

         if (tilt_norm > m_params.max_tilt_angle_rad)
         {
            stick_setpoints *= (m_params.max_tilt_angle_rad / tilt_norm);
         }

         const math::Vec3f attitude_setpoints{stick_setpoints[idx(Axis::roll)], stick_setpoints[idx(Axis::pitch)], 0.0f};

         const math::Vec3f attitude_estimation{m_state_estimation.euler.roll(),
                                               m_state_estimation.euler.pitch(),
                                               m_state_estimation.euler.yaw()};

         m_rate_setpoints = m_attitude_controller.update(attitude_setpoints, attitude_estimation);

         m_rate_setpoints[idx(Axis::roll)]  = std::clamp(m_rate_setpoints[idx(Axis::roll)], -m_params.max_rate[idx(Axis::roll)], m_params.max_rate[idx(Axis::roll)]);
         m_rate_setpoints[idx(Axis::pitch)] = std::clamp(m_rate_setpoints[idx(Axis::pitch)], -m_params.max_rate[idx(Axis::pitch)], m_params.max_rate[idx(Axis::pitch)]);
      }
      else   // generate rate setpoints from sticks
      {
         m_rate_setpoints[idx(Axis::roll)]  = m_stick_command.roll * m_params.max_rate[idx(Axis::roll)];
         m_rate_setpoints[idx(Axis::pitch)] = m_stick_command.pitch * m_params.max_rate[idx(Axis::pitch)];
      }

      // yaw rate from stick setpoint
      m_rate_setpoints[idx(Axis::yaw)] = m_stick_command.yaw * m_params.max_rate[idx(Axis::yaw)];
   }

   void update_rate_control()
   {
      math::Vec3f gyro_rate{};
      math::Vec3f gyro_rate_dterm{};

      // gyro LPF
      for (std::size_t i = 0; i < num_axis; i++)
      {
         gyro_rate[i] = m_gyro_lpf[i].get().apply(m_state_estimation.raw_gyro_radps[i], m_dt_s);
      }

      // gyro d-term LPF
      for (std::size_t i = 0; i < num_axis; i++)
      {
         gyro_rate_dterm[i] = m_gyro_dterm_lpf[i].get().apply(gyro_rate[i]);
      }

      const bool run_integrator{m_armed &&
                                (m_stick_command.throttle > m_params.throttle_gate_integrator)};

      m_torque_setpoints = m_rate_controller.update(m_rate_setpoints,
                                                    gyro_rate,
                                                    gyro_rate_dterm,
                                                    m_dt_s,
                                                    run_integrator);
   }

   void update_control_allocation()
   {
      using namespace aeromight_boundaries;

      const math::Vec4f control_setpoints{m_torque_setpoints[idx(Axis::roll)],
                                          m_torque_setpoints[idx(Axis::pitch)],
                                          m_torque_setpoints[idx(Axis::yaw)],
                                          m_stick_command.throttle};

      m_actuator_setpoints = m_control_allocator.update(control_setpoints);

      // provide allocator saturation feedback to rate control
      m_control_allocator.estimate_saturation();
      m_rate_controller.set_saturation_status(m_control_allocator.get_actuator_saturation_positive(), m_control_allocator.get_actuator_saturation_negative());
   }

   void update_actuator()
   {
      math::Vec4f motor_values{};

      motor::apply_motor_permutation(motor_values, m_actuator_setpoints, m_motor_mapping);

      motor::apply_thrust_linearization(motor_values, m_params.thrust_linearization_factor,
                                        m_control_allocator.get_actuator_min(),
                                        m_control_allocator.get_actuator_max());

      m_actuator_control.update(motor_values.as_array(), false);
   }

   void publish_health()
   {
      m_control_health_publisher.update_latest(m_control_status, m_current_time_ms);
   }

   void reset()
   {
      m_rate_controller.reset();
      m_control_allocator.reset();

      for (auto& filter : m_gyro_lpf)
      {
         filter.get().reset();
      }

      for (auto& filter : m_gyro_dterm_lpf)
      {
         filter.get().reset();
      }

      m_rate_setpoints.zero();
      m_torque_setpoints.zero();
      m_actuator_setpoints.zero();
   }

   void print_log()
   {
      static uint32_t m_counter{0};
      if ((m_counter++ % 125u) == 0)
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
                         m_rate_setpoints[0],
                         m_rate_setpoints[1],
                         m_rate_setpoints[2],
                         m_torque_setpoints[0],
                         m_torque_setpoints[1],
                         m_torque_setpoints[2],
                         m_stick_command.throttle,
                         m_actuator_setpoints[0],
                         m_actuator_setpoints[1],
                         m_actuator_setpoints[2],
                         m_actuator_setpoints[3]);
      }
   }

   bool is_state_estimation_valid() const
   {
      const auto state_estimation_data_age_ms = m_current_time_ms - m_state_estimation.timestamp_ms;
      return (m_state_estimation.attitude_estimation_valid && (state_estimation_data_age_ms <= m_params.max_age_state_estimation_data_ms));
   }

   static constexpr uint32_t control_led_period_ms = 100u;

   AttitudeController&                                       m_attitude_controller;
   RateController&                                           m_rate_controller;
   ControlAllocator&                                         m_control_allocator;
   StickCommandSource&                                       m_stick_command_source;
   ActuatorControl&                                          m_actuator_control;
   std::array<std::reference_wrapper<GyroFilter>, num_axis>  m_gyro_lpf;
   std::array<std::reference_wrapper<DtermFilter>, num_axis> m_gyro_dterm_lpf;
   Led&                                                      m_led;
   const math::Vec4<uint8_t>&                                m_motor_mapping;
   ControlHealthPublisher&                                   m_control_health_publisher;
   const SystemStateSubscriber&                              m_system_state_subscriber;
   const aeromight_boundaries::StateEstimation&              m_state_estimation_subscriber;
   Logger&                                                   m_logger;
   const ControlParams&                                      m_params;
   aeromight_boundaries::ControlStatus                       m_control_status{};
   rc::StickCommand                                          m_stick_command{};
   aeromight_boundaries::SystemState                         m_system_state_setpoints{};
   aeromight_boundaries::StateEstimation                     m_state_estimation{};
   math::Vec3f                                               m_rate_setpoints{};
   math::Vec3f                                               m_torque_setpoints{};
   math::Vec4f                                               m_actuator_setpoints{};
   float                                                     m_dt_s{0.0f};
   bool                                                      m_armed{false};
   uint32_t                                                  m_current_time_ms{0};
   uint32_t                                                  m_current_time_us{0};
   uint32_t                                                  m_last_execution_time_us{0};
};

}   // namespace aeromight_control
