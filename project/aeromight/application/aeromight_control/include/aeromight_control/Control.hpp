#pragma once

#include <algorithm>

#include "StateEstimation.hpp"
#include "aeromight_boundaries/ActuatorSetpoints.hpp"
#include "aeromight_boundaries/ControlHealth.hpp"
#include "aeromight_boundaries/ControlState.hpp"
#include "aeromight_boundaries/FlightControlSetpoints.hpp"
#include "boundaries/SharedData.hpp"
#include "error/error_handler.hpp"
#include "interfaces/IClockSource.hpp"

namespace aeromight_control
{

template <typename AttitudeController, typename RateController, typename ControlAllocator, interfaces::IClockSource ClockSource, typename Logger>
class Control
{
   using ActuatorControl        = boundaries::SharedData<aeromight_boundaries::ActuatorControl>;
   using ControlHealth          = boundaries::SharedData<aeromight_boundaries::ControlHealth>;
   using FlightControlSetpoints = boundaries::SharedData<aeromight_boundaries::FlightControlSetpoints>;

public:
   explicit Control(AttitudeController&           attitude_controller,
                    RateController&               rate_controller,
                    ControlAllocator&             control_allocator,
                    ActuatorControl&              actuator_control_storage,
                    ControlHealth&                control_health_storage,
                    const FlightControlSetpoints& flight_control_setpoint_storage,
                    const StateEstimation&        state_estimation_data,
                    Logger&                       logger,
                    const float                   max_roll_rate_radps,
                    const float                   max_pitch_rate_radps,
                    const float                   max_yaw_rate_radps,
                    const float                   max_tilt_angle_rad,
                    const float                   actuator_min,
                    const float                   actuator_max,
                    const float                   lift_throttle,
                    const float                   thrust_model_factor)
       : m_attitude_controller{attitude_controller},
         m_rate_controller{rate_controller},
         m_control_allocator{control_allocator},
         m_actuator_control_storage{actuator_control_storage},
         m_control_health_storage{control_health_storage},
         m_flight_control_setpoint_storage{flight_control_setpoint_storage},
         m_state_estimation_data{state_estimation_data},
         m_logger{logger},
         m_max_roll_rate_radps{max_roll_rate_radps},
         m_max_pitch_rate_radps{max_pitch_rate_radps},
         m_max_yaw_rate_radps{max_yaw_rate_radps},
         m_max_tilt_angle_rad{max_tilt_angle_rad},
         m_actuator_min{actuator_min},
         m_actuator_max{actuator_max},
         m_lift_throttle{lift_throttle},
         m_thrust_model_factor{thrust_model_factor}

   {
      error::verify(actuator_min < actuator_max);
      error::verify(aeromight_boundaries::ActuatorParams::min <= actuator_min);
      error::verify(actuator_max <= aeromight_boundaries::ActuatorParams::max);

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

      get_flight_control_setpoints();

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

   void get_flight_control_setpoints()
   {
      m_last_flight_control_setpoints = m_flight_control_setpoint_storage.get_latest();
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
      const float thrust_signal = m_actuator_min + (m_last_flight_control_setpoints.data.throttle * (1.0f - m_actuator_min));
      m_thrust_setpoint         = (m_thrust_model_factor * (thrust_signal * thrust_signal)) + ((1.0f - m_thrust_model_factor) * thrust_signal);
   }

   void run_attitude_controller()
   {
      const math::Vector3 angle_setpoint_rad   = {m_last_flight_control_setpoints.data.roll * m_max_tilt_angle_rad,
                                                  m_last_flight_control_setpoints.data.pitch * m_max_tilt_angle_rad,
                                                  0.0f};
      const math::Vector3 angle_estimation_rad = {m_state_estimation_data.euler.roll(),
                                                  m_state_estimation_data.euler.pitch(),
                                                  m_state_estimation_data.euler.yaw()};

      m_desired_rate_radps.set(m_attitude_controller.update(angle_setpoint_rad, angle_estimation_rad));

      // yaw always rate-controlled
      m_desired_rate_radps.yaw(m_last_flight_control_setpoints.data.yaw * m_max_yaw_rate_radps);
   }

   math::Vector3 run_rate_controller(const bool run_integrator)
   {
      return m_rate_controller.update(m_desired_rate_radps.vector(), m_state_estimation_data.gyro_radps, m_time_delta_s, run_integrator);
   }

   void run_control(const bool run_integrator)
   {
      if (m_time_delta_s <= 0.0f)   // can't run rate controller
      {
         m_local_control_health.error.set(static_cast<uint8_t>(aeromight_boundaries::ControlHealth::Error::invalid_time_delta));
         return;
      }

      estimate_collective_thrust();

      run_attitude_controller();

      const math::Euler torque_cmd{run_rate_controller(run_integrator)};

      m_actuator_control.setpoints = m_control_allocator.allocate(torque_cmd, m_thrust_setpoint, m_actuator_min, m_actuator_max);

      publish_actuator_setpoints();

      // determine allocator saturation
      m_rate_controller.set_saturation_status(m_control_allocator.get_actuator_saturation_positive(), m_control_allocator.get_actuator_saturation_negative());

      if ((m_counter++ % 250) == 0)
      {
         m_logger.printf("r=%.2f, p=%.2f, y=%.2f, t=%.2f, r=%.2f, p=%.2f, y=%.2f, 1=%.2f, 2=%.2f, 3=%.2f, 4=%.2f",
                         m_state_estimation_data.euler.roll(),
                         m_state_estimation_data.euler.pitch(),
                         m_state_estimation_data.euler.yaw(),
                         m_thrust_setpoint,
                         torque_cmd.roll(),
                         torque_cmd.pitch(),
                         torque_cmd.yaw(),
                         m_actuator_control.setpoints.m1,
                         m_actuator_control.setpoints.m2,
                         m_actuator_control.setpoints.m3,
                         m_actuator_control.setpoints.m4);
      }
   }

   void start_actuator()
   {
      m_actuator_control.enabled = true;
   }

   void stop_actuator()
   {
      m_actuator_control.enabled = false;
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
      m_desired_rate_radps.zero();
      m_rate_controller.reset();
      m_thrust_setpoint  = 0.0f;
      m_actuator_control = {};
      m_control_saturation_positive.fill(false);
      m_control_saturation_negative.fill(false);
   }

   bool armed() const
   {
      return m_last_flight_control_setpoints.data.armed;
   }

   bool kill() const
   {
      return m_last_flight_control_setpoints.data.kill;
   }

   bool airborne() const
   {
      return (m_last_flight_control_setpoints.data.throttle > m_lift_throttle);
   }

   AttitudeController&                        m_attitude_controller;
   RateController&                            m_rate_controller;
   ControlAllocator&                          m_control_allocator;
   ActuatorControl&                           m_actuator_control_storage;
   ControlHealth&                             m_control_health_storage;
   const FlightControlSetpoints&              m_flight_control_setpoint_storage;
   const StateEstimation&                     m_state_estimation_data;
   Logger&                                    m_logger;
   const float                                m_max_roll_rate_radps;
   const float                                m_max_pitch_rate_radps;
   const float                                m_max_yaw_rate_radps;
   const float                                m_max_tilt_angle_rad;
   const float                                m_actuator_min;
   const float                                m_actuator_max;
   const float                                m_lift_throttle;
   const float                                m_thrust_model_factor;
   aeromight_boundaries::ActuatorControl      m_actuator_control{};
   aeromight_boundaries::ControlHealth        m_local_control_health{};
   FlightControlSetpoints::Sample             m_last_flight_control_setpoints{};
   math::Euler                                m_desired_rate_radps{};
   std::array<bool, RateController::num_axis> m_control_saturation_positive{};
   std::array<bool, RateController::num_axis> m_control_saturation_negative{};
   float                                      m_thrust_setpoint{};
   float                                      m_time_delta_s{0.0f};
   uint32_t                                   m_current_time_ms{0};
   uint32_t                                   m_last_execution_time_ms{0};

   uint32_t m_counter{};
};

}   // namespace aeromight_control
