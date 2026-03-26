#include "ControlTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_control/AttitudeController.hpp"
#include "aeromight_control/Control.hpp"
#include "aeromight_control/ControlAllocator.hpp"
#include "aeromight_control/ControlInput.hpp"
#include "aeromight_control/EstimationAndControl.hpp"
#include "aeromight_control/RateController.hpp"
#include "aeromight_estimation/AltitudeEkf.hpp"
#include "aeromight_estimation/Estimation.hpp"
#include "error/error_handler.hpp"
#include "estimation/AttitudeEstimator.hpp"
#include "event_handling/event_check.hpp"
#include "logging/LogClient.hpp"
#include "math/ButterworthFilter.hpp"
#include "math/FirstOrderLpf.hpp"
#include "math/constants.hpp"
#include "radio_control/ThrottleCurve.hpp"
#include "rtos/QueueSender.hpp"
#include "rtos/constants.hpp"
#include "rtos/periodic_task.hpp"
#include "sys_time/ClockSource.hpp"
#include "task_params.hpp"

namespace logging
{

extern rtos::QueueSender<params::LogBuffer> logging_queue_sender;

}   // namespace logging

extern "C"
{

   [[noreturn]] void control_task(void* const params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::ControlTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      // complimentary/mahony filter parameters
      constexpr float    accelerometer_weight                           = 0.2f;
      constexpr float    gyro_bias_weight                               = 0.1f;
      // altitude ekf2 parameters
      constexpr float    process_noise_z                                = 1.0f;
      constexpr float    process_noise_vz                               = 5.0f;
      constexpr float    process_noise_accel_bias                       = 0.0001f;
      constexpr float    measurement_noise_baro                         = 0.5f;
      constexpr float    tilt_gating_attitude_angle_rad                 = 0.2f;
      constexpr float    tilt_gating_accel_weight                       = 0.08f;
      // estimation parameters
      constexpr uint8_t  num_samples_reference_pressure                 = 10u;
      constexpr uint8_t  max_recovery_attempts                          = 3u;
      constexpr uint32_t wait_timeout_reference_pressure_acquisition_ms = controller::task::barometer_task_period_in_ms * 50u;
      constexpr uint32_t max_age_imu_data_ms                            = controller::task::imu_task_period_in_ms * 10u;
      constexpr uint32_t max_age_baro_data_ms                           = controller::task::barometer_task_period_in_ms * 10u;
      constexpr float    max_valid_imu_sample_dt_s                      = 0.02f;
      constexpr float    max_valid_barometer_sample_dt_s                = 10.0f;
      constexpr float    roll_trim                                      = -0.02f;
      constexpr float    pitch_trim                                     = -0.01f;
      // control parameters
      constexpr float    min_dt_s                                       = 0.002f;
      constexpr float    max_dt_s                                       = 0.010f;
      constexpr float    gyro_lpf_cutoff_hz                             = 80.0f;
      constexpr float    stick_input_lpf_cutoff_hz                      = 10.0f;
      constexpr float    pid_dterm_filter_cutoff_frequency_hz           = 25.0f;
      constexpr float    thrust_linearization_factor                    = 0.8f;
      // throttle and thrust
      constexpr float    thrust_limiting                                = 0.3f;
      constexpr float    actuator_min                                   = 0.0f;
      constexpr float    actuator_max                                   = 1.0f - thrust_limiting;
      constexpr float    actuator_idle                                  = 0.1f;
      constexpr float    throttle_min                                   = actuator_min;
      constexpr float    throttle_max                                   = actuator_max;
      constexpr float    throttle_arming                                = 0.01f;
      constexpr float    throttle_hover                                 = 0.3f;
      constexpr float    throttle_curve_factor                          = 0.5f;
      constexpr float    throttle_gate_integrator                       = 0.15f;
      // attitude controller
      constexpr bool     run_attitude_controller                        = false;
      constexpr float    max_tilt_angle_rad                             = 30 * math::constants::deg_to_rad;
      constexpr float    attitude_controller_roll_kp                    = 5.0f;
      constexpr float    attitude_controller_pitch_kp                   = 5.0f;
      constexpr float    attitude_controller_yaw_kp                     = 0.0f;
      constexpr float    max_roll_rate_radps                            = 3.0f;
      constexpr float    max_pitch_rate_radps                           = 3.0f;
      constexpr float    max_yaw_rate_radps                             = 3.0f;
      // rate controller
      constexpr float    torque_limit                                   = 1.0f;
      // p
      constexpr float    rate_controller_roll_kp                        = 0.04f;
      constexpr float    rate_controller_pitch_kp                       = 0.04f;
      constexpr float    rate_controller_yaw_kp                         = 0.06f;
      // i
      constexpr float    rate_controller_roll_ki                        = 0.0f;
      constexpr float    rate_controller_pitch_ki                       = 0.0f;
      constexpr float    rate_controller_yaw_ki                         = 0.0f;
      constexpr float    rate_controller_roll_integrator_limit          = 0.2f;
      constexpr float    rate_controller_pitch_integrator_limit         = 0.2f;
      constexpr float    rate_controller_yaw_integrator_limit           = 0.03f;
      // d
      constexpr float    rate_controller_roll_kd                        = 0.0f;
      constexpr float    rate_controller_pitch_kd                       = 0.0f;
      constexpr float    rate_controller_yaw_kd                         = 0.0f;
      // control allocator
      constexpr float    thrust_deadband                                = 0.01f;
      constexpr float    yaw_saturation_limit_factor                    = 0.25f;
      constexpr float    slew_rate_limit_s                              = 1.0f;

      LogClient logger_estimation{logging::logging_queue_sender, "estimation"};
      LogClient logger_control{logging::logging_queue_sender, "control"};

      estimation::AttitudeEstimator ahrs_filter{accelerometer_weight, gyro_bias_weight};

      aeromight_estimation::AltitudeEkf altitude_ekf{process_noise_z,
                                                     process_noise_vz,
                                                     process_noise_accel_bias,
                                                     measurement_noise_baro,
                                                     tilt_gating_attitude_angle_rad,
                                                     tilt_gating_accel_weight};

      aeromight_estimation::Estimation<decltype(ahrs_filter),
                                       decltype(altitude_ekf),
                                       sys_time::ClockSource,
                                       LogClient>
          estimation{ahrs_filter,
                     altitude_ekf,
                     aeromight_boundaries::aeromight_data.estimator_health_storage,
                     data->state_estimation,
                     aeromight_boundaries::aeromight_data.imu_data_storage,
                     aeromight_boundaries::aeromight_data.barometer_data_storage,
                     logger_estimation,
                     max_recovery_attempts,
                     num_samples_reference_pressure,
                     wait_timeout_reference_pressure_acquisition_ms,
                     max_age_imu_data_ms,
                     max_age_baro_data_ms,
                     max_valid_imu_sample_dt_s,
                     max_valid_barometer_sample_dt_s,
                     roll_trim,
                     pitch_trim};

      const math::Vector3 attitude_gains_p{attitude_controller_roll_kp, attitude_controller_pitch_kp, attitude_controller_yaw_kp};

      aeromight_control::AttitudeController attitude_controller{attitude_gains_p};

      const math::Vector3 rate_gains_p{rate_controller_roll_kp, rate_controller_pitch_kp, rate_controller_yaw_kp};
      const math::Vector3 rate_gains_i{rate_controller_roll_ki, rate_controller_pitch_ki, rate_controller_yaw_ki};
      const math::Vector3 rate_gains_d{rate_controller_roll_kd, rate_controller_pitch_kd, rate_controller_yaw_kd};
      const math::Vector3 rate_integrator_limits{rate_controller_roll_integrator_limit, rate_controller_pitch_integrator_limit, rate_controller_yaw_integrator_limit};

      aeromight_control::RateController rate_controller{rate_gains_p,
                                                        rate_gains_i,
                                                        rate_gains_d,
                                                        rate_integrator_limits,
                                                        torque_limit};

      aeromight_control::ControlAllocator control_allocator{actuator_min,
                                                            actuator_max,
                                                            actuator_idle,
                                                            thrust_deadband,
                                                            yaw_saturation_limit_factor,
                                                            slew_rate_limit_s};

      radio_control::ThrottleCurve throttle_curve{throttle_hover, throttle_curve_factor};

      aeromight_control::ControlInput<decltype(throttle_curve)> control_input{throttle_curve,
                                                                              aeromight_boundaries::aeromight_data.flight_control_setpoints,
                                                                              throttle_min,
                                                                              throttle_max};

      math::FirstOrderLpf roll_input_lpf{stick_input_lpf_cutoff_hz};
      math::FirstOrderLpf pitch_input_lpf{stick_input_lpf_cutoff_hz};
      math::FirstOrderLpf yaw_input_lpf{stick_input_lpf_cutoff_hz};

      math::FirstOrderLpf gyro_x_lpf{gyro_lpf_cutoff_hz};
      math::FirstOrderLpf gyro_y_lpf{gyro_lpf_cutoff_hz};
      math::FirstOrderLpf gyro_z_lpf{gyro_lpf_cutoff_hz};

      math::ButterworthFilter pid_dterm_x_lpf{pid_dterm_filter_cutoff_frequency_hz, controller::task::control_task_period_in_ms / 1000.0f};
      math::ButterworthFilter pid_dterm_y_lpf{pid_dterm_filter_cutoff_frequency_hz, controller::task::control_task_period_in_ms / 1000.0f};
      math::ButterworthFilter pid_dterm_z_lpf{pid_dterm_filter_cutoff_frequency_hz, controller::task::control_task_period_in_ms / 1000.0f};

      aeromight_control::Control<decltype(attitude_controller),
                                 decltype(rate_controller),
                                 decltype(control_allocator),
                                 decltype(control_input),
                                 math::FirstOrderLpf,
                                 math::ButterworthFilter,
                                 sys_time::ClockSource,
                                 LogClient>
          control{attitude_controller,
                  rate_controller,
                  control_allocator,
                  control_input,
                  roll_input_lpf,
                  pitch_input_lpf,
                  yaw_input_lpf,
                  gyro_x_lpf,
                  gyro_y_lpf,
                  gyro_z_lpf,
                  pid_dterm_x_lpf,
                  pid_dterm_y_lpf,
                  pid_dterm_z_lpf,
                  aeromight_boundaries::aeromight_data.actuator_control,
                  aeromight_boundaries::aeromight_data.control_health_storage,
                  aeromight_boundaries::aeromight_data.system_state_info,
                  data->state_estimation,
                  logger_control,
                  min_dt_s,
                  max_dt_s,
                  run_attitude_controller,
                  max_tilt_angle_rad,
                  math::Vector3{max_roll_rate_radps, max_pitch_rate_radps, max_yaw_rate_radps},
                  actuator_min,
                  actuator_max,
                  throttle_arming,
                  throttle_gate_integrator,
                  thrust_linearization_factor};

      aeromight_control::EstimationAndControl<decltype(estimation),
                                              decltype(control)>
          estimation_and_control{estimation,
                                 control,
                                 controller::task::control_task_period_in_ms};

      const auto event_bits = data->control_task_notification_waiter.wait(rtos::constants::max_delay);
      if ((event_bits != 0) &&
          (event_handling::has_event(event_bits, aeromight_boundaries::ControlTaskEvents::start)))
      {
         // start pwm
         data->master_pwm_timer.enable_interrupt();

         data->slave_pwm_timer.enable_channel(data->slave_pwm_timer_channels);
         data->master_pwm_timer.enable_channel(data->master_pwm_timer_channels);

         data->master_pwm_timer.enable_all_outputs();

         data->slave_pwm_timer.enable_counter();
         data->master_pwm_timer.enable_counter();

         estimation_and_control.start();
      }
      else
      {
         error::stop_operation();
      }

      rtos::run_periodic_task(estimation_and_control);
   }

}   // extern "C"

extern "C"
{
   constexpr uint32_t pwm_min_us                       = 1000u;                                                 // 1.0 ms
   constexpr uint32_t pwm_max_us                       = 2000u;                                                 // 2.0 ms
   constexpr uint32_t max_age_actuator_control_data_ms = controller::task::control_task_period_in_ms * 1250u;   // 5 seconds

   static inline uint32_t actuator_to_ccr(float x)
   {
      x = std::clamp(x, 0.0f, 1.0f);
      return pwm_min_us + static_cast<uint32_t>(x * static_cast<float>(pwm_max_us - pwm_min_us));
   }

   // PWM generator
   void TIM1_UP_TIM10_IRQHandler(void)
   {
      using namespace hw::timer;

      auto& data = controller::control_task_data;

      // Check update interrupt
      if (data.master_pwm_timer.is_update_flag_active())
      {
         data.master_pwm_timer.clear_update_flag();

         const auto& actuator_control_storage = aeromight_boundaries::aeromight_data.actuator_control;

         // Get latest actuator command (lock-free)
         const auto     sample        = actuator_control_storage.get_latest();
         const uint32_t sample_age_ms = (sys_time::ClockSource::now_ms() - sample.timestamp_ms);
         const bool     stale_sample  = (sample_age_ms > max_age_actuator_control_data_ms);
         const auto&    act           = sample.data;

         const bool enable_outputs = (!stale_sample) && (act.enabled);

         for (uint8_t i = 0; i < 4u; i++)
         {
            const uint32_t pwm = enable_outputs ? actuator_to_ccr(act.setpoints[i]) : pwm_min_us;

            auto& out = data.motor_output_map[i];
            out.timer.set_compare_value_for_channel(out.channel, pwm);
         }
      }
   }

}   // extern "C"
