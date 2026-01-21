#pragma once

#include <algorithm>

#include "aeromight_boundaries/ActuatorSetpoints.hpp"
#include "aeromight_boundaries/ControlHealth.hpp"
#include "aeromight_boundaries/ControlState.hpp"
#include "aeromight_boundaries/FlightControlSetpoints.hpp"
#include "aeromight_boundaries/StateEstimation.hpp"
#include "boundaries/SharedData.hpp"
#include "interfaces/IClockSource.hpp"
#include "math/Vector2.hpp"

namespace aeromight_control
{

template <typename AttitudeController,
          typename RateController,
          typename ControlAllocator,
          typename FirstOrderLpf,
          typename ButterworthFilter,
          interfaces::IClockSource ClockSource,
          typename Logger>
class Control
{
   using ActuatorControl        = boundaries::SharedData<aeromight_boundaries::ActuatorControl>;
   using ControlHealth          = boundaries::SharedData<aeromight_boundaries::ControlHealth>;
   using FlightControlSetpoints = boundaries::SharedData<aeromight_boundaries::FlightControlSetpoints>;
   using StateEstimation        = aeromight_boundaries::StateEstimation;

public:
   static constexpr std::size_t num_axis = 3u;

   explicit Control(AttitudeController&                          attitude_controller,
                    RateController&                              rate_controller,
                    ControlAllocator&                            control_allocator,
                    FirstOrderLpf&                               gyro_x_lpf,
                    FirstOrderLpf&                               gyro_y_lpf,
                    FirstOrderLpf&                               gyro_z_lpf,
                    ButterworthFilter&                           angular_acceleration_x_lpf2,
                    ButterworthFilter&                           angular_acceleration_y_lpf2,
                    ButterworthFilter&                           angular_acceleration_z_lpf2,
                    ActuatorControl&                             actuator_control_storage,
                    ControlHealth&                               control_health_storage,
                    const FlightControlSetpoints&                flight_control_setpoint_storage,
                    const aeromight_boundaries::StateEstimation& state_estimation_data,
                    Logger&                                      logger,
                    const float                                  time_delta_lower_limit_s,
                    const float                                  time_delta_upper_limit_s,
                    const float                                  max_roll_rate_radps,
                    const float                                  max_pitch_rate_radps,
                    const float                                  max_yaw_rate_radps,
                    const float                                  max_tilt_angle_rad,
                    const float                                  lift_throttle,
                    const float                                  hover_throttle)
       : m_attitude_controller{attitude_controller},
         m_rate_controller{rate_controller},
         m_control_allocator{control_allocator},
         m_gyro_lpf{gyro_x_lpf, gyro_y_lpf, gyro_z_lpf},
         m_angular_acceleration_lpf2{angular_acceleration_x_lpf2, angular_acceleration_y_lpf2, angular_acceleration_z_lpf2},
         m_actuator_control_storage{actuator_control_storage},
         m_control_health_storage{control_health_storage},
         m_flight_control_setpoint_storage{flight_control_setpoint_storage},
         m_state_estimation{state_estimation_data},
         m_logger{logger},
         m_time_delta_lower_limit_s{time_delta_lower_limit_s},
         m_time_delta_upper_limit_s{time_delta_upper_limit_s},
         m_max_roll_rate_radps{max_roll_rate_radps},
         m_max_pitch_rate_radps{max_pitch_rate_radps},
         m_max_yaw_rate_radps{max_yaw_rate_radps},
         m_max_tilt_angle_rad{max_tilt_angle_rad},
         m_lift_throttle{lift_throttle},
         m_hover_throttle{hover_throttle}

   {
      m_logger.enable();
   }

   void start()
   {
      if (m_local_control_health.state == aeromight_boundaries::ControlState::inactive)
      {
         m_last_execution_time_ms = ClockSource::now_ms();
         get_time();
         move_to_disarmed();
         publish_health();
      }
   }

   void execute()
   {
      get_time();

      get_flight_control_setpoints();

      run_state_machine();

      publish_actuator_setpoints();

      publish_health();

      m_last_filtered_gyro_radps = m_filtered_gyro_radps;
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

         case aeromight_boundaries::ControlState::armed:
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

            run_control();
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
      m_current_time_ms = ClockSource::now_ms();

      m_dt_s = static_cast<float>(m_current_time_ms - m_last_execution_time_ms) / 1000.0f;
      m_dt_s = std::clamp(m_dt_s, m_time_delta_lower_limit_s, m_time_delta_upper_limit_s);

      m_last_execution_time_ms = m_current_time_ms;
   }

   void get_flight_control_setpoints()
   {
      m_last_flight_control_setpoints = m_flight_control_setpoint_storage.get_latest();

      // Pilot Command Mapping: perform pitch sign inversion (flight stick pullback -> pitch up)
      m_last_flight_control_setpoints.data.pitch *= -1.0f;
   }

   void move_to_killed()
   {
      reset();
      reset_filters();
      stop_actuator();
      m_local_control_health.state = aeromight_boundaries::ControlState::emergency_kill;
      m_logger.print("killed");
   }

   void move_to_disarmed()
   {
      reset();
      reset_filters();
      stop_actuator();
      m_local_control_health.state = aeromight_boundaries::ControlState::disarmed;
      m_logger.print("disarmed");
   }

   void move_to_armed()
   {
      reset();
      reset_filters();
      start_actuator();
      m_local_control_health.state = aeromight_boundaries::ControlState::armed;
      m_logger.print("armed");
   }

   void get_rate_setpoints(const bool run_attitude_controller)
   {
      if (run_attitude_controller)
      {
         math::Vector2 manual_setpoints{m_last_flight_control_setpoints.data.roll * m_max_tilt_angle_rad,
                                        m_last_flight_control_setpoints.data.pitch * m_max_tilt_angle_rad};

         const float tilt_norm = manual_setpoints.norm();

         if (tilt_norm > m_max_tilt_angle_rad)
         {
            manual_setpoints *= (m_max_tilt_angle_rad / tilt_norm);
         }

         const math::Vector3 angle_estimation_rad{m_state_estimation.euler.roll(),
                                                  m_state_estimation.euler.pitch(),
                                                  m_state_estimation.euler.yaw()};

         const math::Vector3 angle_setpoints{manual_setpoints[0], manual_setpoints[1], 0.0f};

         m_angular_rate_setpoints = m_attitude_controller.update(angle_setpoints, angle_estimation_rad);

         // @TODO: apply rate limits clamp to rate setpoints
      }
      else   // generate rate setpoints from sticks
      {
         //@TODO: scale values by acro rate max
         m_angular_rate_setpoints[aeromight_boundaries::ControlAxis::roll]  = (m_last_flight_control_setpoints.data.roll * m_max_roll_rate_radps);
         m_angular_rate_setpoints[aeromight_boundaries::ControlAxis::pitch] = (m_last_flight_control_setpoints.data.pitch * m_max_pitch_rate_radps);
      }

      // yaw always bypassed
      m_angular_rate_setpoints[aeromight_boundaries::ControlAxis::yaw] = (m_last_flight_control_setpoints.data.yaw * m_max_yaw_rate_radps);
   }

   void get_torque_setpoints(const bool run_integrator)
   {
      for (uint8_t i = 0; i < num_axis; i++)
      {
         m_filtered_gyro_radps[i] = m_gyro_lpf[i].get().apply(m_state_estimation.gyro_radps[i], m_dt_s);
      }

      const math::Vector3 gyro_rate_derivative = {(m_filtered_gyro_radps - m_last_filtered_gyro_radps) / m_dt_s};
      math::Vector3       filtered_angular_acceleration{};

      for (uint8_t i = 0; i < num_axis; i++)
      {
         filtered_angular_acceleration[i] = m_angular_acceleration_lpf2[i].get().apply(gyro_rate_derivative[i]);
      }

      m_torque_setpoints = m_rate_controller.update(m_angular_rate_setpoints, m_filtered_gyro_radps, filtered_angular_acceleration, m_dt_s, run_integrator);
   }

   void get_actuator_setpoints()
   {
      const math::Vector4 control_setpoints{m_torque_setpoints[aeromight_boundaries::ControlAxis::roll],
                                            m_torque_setpoints[aeromight_boundaries::ControlAxis::pitch],
                                            m_torque_setpoints[aeromight_boundaries::ControlAxis::yaw],
                                            m_last_flight_control_setpoints.data.throttle};

      m_control_allocator.set_control_setpoints(control_setpoints);

      m_control_allocator.allocate();

      m_control_allocator.apply_slew_rate_limits(m_dt_s);

      m_control_allocator.clip_actuator_setpoints();

      m_actuator_control.setpoints = m_control_allocator.get_actuator_setpoints();

      // determine allocator saturation
      m_control_allocator.estimate_saturation();
      m_rate_controller.set_saturation_status(m_control_allocator.get_actuator_saturation_positive(), m_control_allocator.get_actuator_saturation_negative());
   }

   void run_control()
   {
      get_rate_setpoints(true);
      get_torque_setpoints(m_last_flight_control_setpoints.data.throttle > m_hover_throttle);
      get_actuator_setpoints();

      print_log();
   }

   void start_actuator()
   {
      m_actuator_control.enabled = true;
      m_actuator_control.setpoints.zero();
   }

   void stop_actuator()
   {
      m_actuator_control.enabled = false;
      m_actuator_control.setpoints.zero();
   }

   void publish_actuator_setpoints()
   {
      m_actuator_control_storage.update_latest(m_actuator_control, m_current_time_ms);
   }

   void publish_health()
   {
      m_control_health_storage.update_latest(m_local_control_health, m_current_time_ms);
   }

   void reset()
   {
      m_rate_controller.reset();
      m_angular_rate_setpoints.zero();
      m_torque_setpoints.zero();
   }

   void reset_filters()
   {
      for (auto& filter : m_gyro_lpf)
      {
         filter.get().reset();
      }

      for (auto& filter : m_angular_acceleration_lpf2)
      {
         filter.get().reset();
      }
   }

   void print_log()
   {
      static uint32_t m_counter{0};
      if ((m_counter++ % 125) == 0)
      {
         m_logger.printf("%.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f | t=%.2f | 1=%.2f 2=%.2f 3=%.2f 4=%.2f",
                         m_filtered_gyro_radps[0],
                         m_filtered_gyro_radps[1],
                         m_filtered_gyro_radps[2],
                         m_state_estimation.euler.roll(),
                         m_state_estimation.euler.pitch(),
                         m_state_estimation.euler.yaw(),
                         m_angular_rate_setpoints[0],
                         m_angular_rate_setpoints[1],
                         m_angular_rate_setpoints[2],
                         m_torque_setpoints[0],
                         m_torque_setpoints[1],
                         m_torque_setpoints[2],
                         m_last_flight_control_setpoints.data.throttle,
                         m_actuator_control.setpoints[0],
                         m_actuator_control.setpoints[1],
                         m_actuator_control.setpoints[2],
                         m_actuator_control.setpoints[3]);
      }
   }

   bool armed() const noexcept
   {
      return m_last_flight_control_setpoints.data.armed;
   }

   bool kill() const noexcept
   {
      return m_last_flight_control_setpoints.data.kill;
   }

   AttitudeController&                                             m_attitude_controller;
   RateController&                                                 m_rate_controller;
   ControlAllocator&                                               m_control_allocator;
   std::array<std::reference_wrapper<FirstOrderLpf>, num_axis>     m_gyro_lpf;
   std::array<std::reference_wrapper<ButterworthFilter>, num_axis> m_angular_acceleration_lpf2;
   ActuatorControl&                                                m_actuator_control_storage;
   ControlHealth&                                                  m_control_health_storage;
   const FlightControlSetpoints&                                   m_flight_control_setpoint_storage;
   const StateEstimation&                                          m_state_estimation;
   Logger&                                                         m_logger;
   const float                                                     m_time_delta_lower_limit_s;
   const float                                                     m_time_delta_upper_limit_s;
   const float                                                     m_max_roll_rate_radps;
   const float                                                     m_max_pitch_rate_radps;
   const float                                                     m_max_yaw_rate_radps;
   const float                                                     m_max_tilt_angle_rad;
   const float                                                     m_lift_throttle;
   const float                                                     m_hover_throttle;
   aeromight_boundaries::ActuatorControl                           m_actuator_control{};
   aeromight_boundaries::ControlHealth                             m_local_control_health{};
   FlightControlSetpoints::Sample                                  m_last_flight_control_setpoints{};
   math::Vector3                                                   m_filtered_gyro_radps{};
   math::Vector3                                                   m_last_filtered_gyro_radps{};
   math::Vector3                                                   m_angular_rate_setpoints{};
   math::Vector3                                                   m_torque_setpoints{};
   float                                                           m_dt_s{0.0f};
   uint32_t                                                        m_current_time_ms{0};
   uint32_t                                                        m_last_execution_time_ms{0};
};

}   // namespace aeromight_control
