#include "ControlTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_control/AttitudeController.hpp"
#include "aeromight_control/Control.hpp"
#include "aeromight_control/ControlAllocator.hpp"
#include "aeromight_control/EstimationAndControl.hpp"
#include "aeromight_control/RateController.hpp"
#include "aeromight_estimation/AltitudeEkf.hpp"
#include "aeromight_estimation/Estimation.hpp"
#include "error/error_handler.hpp"
#include "estimation/AttitudeEstimator.hpp"
#include "logging/LogClient.hpp"
#include "math/ButterworthFilter.hpp"
#include "math/FirstOrderLpf.hpp"
#include "physics/constants.hpp"
#include "rtos/QueueSender.hpp"
#include "rtos/periodic_task.hpp"
#include "rtos/utilities.hpp"
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
      // control parameters
      constexpr float    min_dt_s                                       = 0.002f;
      constexpr float    max_dt_s                                       = 0.010f;
      constexpr float    first_order_lpf_cutoff_frequency_hz            = 80.0f;
      constexpr float    butterworth_filter_cutoff_frequency_hz         = 30.0f;
      // throttle and thrust
      constexpr float    thrust_limiting                                = 0.3f;
      constexpr float    actuator_min                                   = 0.0f;
      constexpr float    actuator_max                                   = 1.0f - thrust_limiting;
      constexpr float    actuator_idle                                  = 0.2f;
      constexpr float    throttle_min                                   = actuator_min;
      constexpr float    throttle_max                                   = actuator_max;
      constexpr float    throttle_hover                                 = 0.50f;
      // attitude controller
      constexpr bool     run_attitude_controller                        = true;
      constexpr float    max_tilt_angle_rad                             = 50 * physics::constants::deg_to_rad;
      constexpr float    attitude_controller_roll_kp                    = 0.6f;
      constexpr float    attitude_controller_pitch_kp                   = 0.6f;
      constexpr float    attitude_controller_yaw_kp                     = 0.0f;
      constexpr float    max_roll_rate_radps                            = 1.5f;
      constexpr float    max_pitch_rate_radps                           = 1.5f;
      constexpr float    max_yaw_rate_radps                             = 1.0f;
      // rate controller
      constexpr float    torque_limit                                   = 1.0f;
      constexpr float    rate_controller_roll_kp                        = 0.04f;
      constexpr float    rate_controller_pitch_kp                       = 0.04f;
      constexpr float    rate_controller_yaw_kp                         = 0.04f;
      constexpr float    rate_controller_roll_ki                        = 0.0f;
      constexpr float    rate_controller_pitch_ki                       = 0.0f;
      constexpr float    rate_controller_yaw_ki                         = 0.0f;
      constexpr float    rate_controller_roll_kd                        = 0.0f;
      constexpr float    rate_controller_pitch_kd                       = 0.0f;
      constexpr float    rate_controller_yaw_kd                         = 0.0f;
      constexpr float    rate_controller_roll_integrator_limit          = 0.3f;
      constexpr float    rate_controller_pitch_integrator_limit         = 0.3f;
      constexpr float    rate_controller_yaw_integrator_limit           = 0.3f;
      // control allocator
      constexpr float    yaw_saturation_limit_factor                    = 0.15f;
      constexpr float    slew_rate_limit_s                              = 2.0f;

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
                     aeromight_boundaries::aeromight_data.imu_sensor_data_storage,
                     aeromight_boundaries::aeromight_data.barometer_sensor_data_storage,
                     logger_estimation,
                     max_recovery_attempts,
                     num_samples_reference_pressure,
                     wait_timeout_reference_pressure_acquisition_ms,
                     max_age_imu_data_ms,
                     max_age_baro_data_ms,
                     max_valid_imu_sample_dt_s,
                     max_valid_barometer_sample_dt_s};

      const math::Vector3 attitude_gains_p{attitude_controller_roll_kp, attitude_controller_pitch_kp, attitude_controller_yaw_kp};

      aeromight_control::AttitudeController attitude_controller{attitude_gains_p};

      const math::Vector3 rate_gains_p{rate_controller_roll_kp, rate_controller_pitch_kp, rate_controller_yaw_kp};
      const math::Vector3 rate_gains_i{rate_controller_roll_ki, rate_controller_pitch_ki, rate_controller_yaw_ki};
      const math::Vector3 rate_gains_d{rate_controller_roll_kd, rate_controller_pitch_kd, rate_controller_yaw_kd};
      const math::Vector3 rate_integrator_limits{rate_controller_roll_integrator_limit, rate_controller_pitch_integrator_limit, rate_controller_yaw_integrator_limit};

      math::ButterworthFilter gyro_derivative_x_lpf2{butterworth_filter_cutoff_frequency_hz, controller::task::control_task_period_in_ms / 1000.0f};
      math::ButterworthFilter gyro_derivative_y_lpf2{butterworth_filter_cutoff_frequency_hz, controller::task::control_task_period_in_ms / 1000.0f};
      math::ButterworthFilter gyro_derivative_z_lpf2{butterworth_filter_cutoff_frequency_hz, controller::task::control_task_period_in_ms / 1000.0f};

      aeromight_control::RateController rate_controller{rate_gains_p,
                                                        rate_gains_i,
                                                        rate_gains_d,
                                                        rate_integrator_limits,
                                                        torque_limit};

      aeromight_control::ControlAllocator control_allocator{actuator_min,
                                                            actuator_max,
                                                            actuator_idle,
                                                            yaw_saturation_limit_factor,
                                                            slew_rate_limit_s};

      math::FirstOrderLpf gyro_x_lpf{first_order_lpf_cutoff_frequency_hz};
      math::FirstOrderLpf gyro_y_lpf{first_order_lpf_cutoff_frequency_hz};
      math::FirstOrderLpf gyro_z_lpf{first_order_lpf_cutoff_frequency_hz};

      aeromight_control::Control<decltype(attitude_controller),
                                 decltype(rate_controller),
                                 decltype(control_allocator),
                                 decltype(gyro_x_lpf),
                                 decltype(gyro_derivative_x_lpf2),
                                 sys_time::ClockSource,
                                 LogClient>
          control{attitude_controller,
                  rate_controller,
                  control_allocator,
                  gyro_x_lpf,
                  gyro_y_lpf,
                  gyro_z_lpf,
                  gyro_derivative_x_lpf2,
                  gyro_derivative_y_lpf2,
                  gyro_derivative_z_lpf2,
                  aeromight_boundaries::aeromight_data.actuator_control,
                  aeromight_boundaries::aeromight_data.control_health_storage,
                  aeromight_boundaries::aeromight_data.flight_control_setpoints,
                  data->state_estimation,
                  logger_control,
                  min_dt_s,
                  max_dt_s,
                  run_attitude_controller,
                  max_tilt_angle_rad,
                  max_roll_rate_radps,
                  max_pitch_rate_radps,
                  max_yaw_rate_radps,
                  throttle_min,
                  throttle_max,
                  throttle_hover};

      aeromight_control::EstimationAndControl<decltype(estimation),
                                              decltype(control)>
          estimation_and_control{estimation,
                                 control,
                                 controller::task::control_task_period_in_ms};

      const auto flags = data->control_task_notification_waiter.wait(rtos::utilities::max_delay);
      if (flags.has_value())
      {
         if (flags.value().test(static_cast<uint8_t>(aeromight_boundaries::ControlTaskEvents::start)))
         {
            // start pwm
            data->control_task_pwm_timer.enable_interrupt();
            data->control_task_pwm_timer.start();
            data->control_task_pwm_timer.enable_all_outputs();

            estimation_and_control.start();
            rtos::run_periodic_task(estimation_and_control);
         }
      }

      // Won't reach here
      while (true)
      {
         error::stop_operation();
      }
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
      if (data.control_task_pwm_timer.is_update_flag_active())
      {
         data.control_task_pwm_timer.clear_update_flag();

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

            data.control_task_pwm_timer.set_compare_value_for_channel(data.motor_to_channel_map[i], pwm);
         }
      }
   }

}   // extern "C"
