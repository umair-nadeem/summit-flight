#include "ControlTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_control/AltitudeEkf.hpp"
#include "aeromight_control/AttitudeController.hpp"
#include "aeromight_control/Control.hpp"
#include "aeromight_control/ControlAllocator.hpp"
#include "aeromight_control/Estimation.hpp"
#include "aeromight_control/EstimationAndControl.hpp"
#include "aeromight_control/RateController.hpp"
#include "error/error_handler.hpp"
#include "estimation/AttitudeEstimator.hpp"
#include "logging/LogClient.hpp"
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

      // complimentary filter parameters
      constexpr float    accelerometer_weight                           = 0.2f;
      constexpr float    gyro_bias_weight                               = 0.1f;
      // altitude ekf2 parameters
      constexpr float    process_noise_z                                = 1.0f;
      constexpr float    process_noise_vz                               = 5.0f;
      constexpr float    process_noise_accel_bias                       = 0.0001f;
      constexpr float    measurement_noise_baro                         = 0.5f;
      constexpr float    tilt_gating_attitude_angle_rad                 = 0.2f;
      constexpr float    tilt_gating_accel_weight                       = 0.08f;
      // esimation parameters
      constexpr uint8_t  num_samples_reference_pressure                 = 10u;
      constexpr uint8_t  max_recovery_attempts                          = 3u;
      constexpr uint32_t wait_timeout_reference_pressure_acquisition_ms = controller::task::barometer_task_period_in_ms * 50u;
      constexpr uint32_t max_age_imu_data_ms                            = controller::task::imu_task_period_in_ms * 10u;
      constexpr uint32_t max_age_baro_data_ms                           = controller::task::barometer_task_period_in_ms * 10u;
      constexpr float    max_valid_imu_sample_dt_s                      = 0.02f;
      constexpr float    max_valid_barometer_sample_dt_s                = 10.0f;
      // control parameters
      constexpr float    time_delta_limit_s                             = 0.008f;
      constexpr float    actuator_min                                   = 0.05f;
      constexpr float    actuator_max                                   = 1.0f;
      constexpr float    lift_throttle                                  = 0.1f;
      constexpr float    attitude_controller_roll_kp                    = 4.0f;
      constexpr float    attitude_controller_pitch_kp                   = 4.0f;
      constexpr float    attitude_controller_yaw_kp                     = 0.0f;
      constexpr float    rate_controller_output_limit                   = 1.0f;
      constexpr float    rate_controller_roll_kp                        = 0.08f;
      constexpr float    rate_controller_pitch_kp                       = 0.08f;
      constexpr float    rate_controller_yaw_kp                         = 0.05f;
      constexpr float    rate_controller_roll_ki                        = 0.04f;
      constexpr float    rate_controller_pitch_ki                       = 0.04f;
      constexpr float    rate_controller_yaw_ki                         = 0.025f;
      constexpr float    rate_controller_roll_kd                        = 0.01f;
      constexpr float    rate_controller_pitch_kd                       = 0.01f;
      constexpr float    rate_controller_yaw_kd                         = 0.00f;
      constexpr float    rate_controller_roll_integrator_limit          = 0.3f;
      constexpr float    rate_controller_pitch_integrator_limit         = 0.3f;
      constexpr float    rate_controller_yaw_integrator_limit           = 0.3f;
      constexpr float    max_roll_rate_radps                            = 3.5f;
      constexpr float    max_pitch_rate_radps                           = 3.5f;
      constexpr float    max_yaw_rate_radps                             = 2.0f;
      constexpr float    max_tilt_angle_rad                             = 30 * physics::constants::deg_to_rad;   // 30 degrees
      constexpr float    yaw_saturation_limit_factor                    = 0.15f;

      LogClient logger_estimation{logging::logging_queue_sender, "estimation"};
      LogClient logger_control{logging::logging_queue_sender, "control"};

      estimation::AttitudeEstimator ahrs_filter{accelerometer_weight, gyro_bias_weight};

      aeromight_control::AltitudeEkf altitude_ekf{process_noise_z,
                                                  process_noise_vz,
                                                  process_noise_accel_bias,
                                                  measurement_noise_baro,
                                                  tilt_gating_attitude_angle_rad,
                                                  tilt_gating_accel_weight};

      aeromight_control::Estimation<decltype(ahrs_filter),
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

      aeromight_control::AttitudeController attitude_controller{math::Vector3{attitude_controller_roll_kp, attitude_controller_pitch_kp, attitude_controller_yaw_kp}};

      aeromight_control::RateController rate_controller{math::Vector3{rate_controller_roll_kp, rate_controller_pitch_kp, rate_controller_yaw_kp},
                                                        math::Vector3{rate_controller_roll_ki, rate_controller_pitch_ki, rate_controller_yaw_ki},
                                                        math::Vector3{rate_controller_roll_kd, rate_controller_pitch_kd, rate_controller_yaw_kd},
                                                        math::Vector3{rate_controller_roll_integrator_limit, rate_controller_pitch_integrator_limit, rate_controller_yaw_integrator_limit},
                                                        rate_controller_output_limit,
                                                        max_roll_rate_radps,
                                                        max_pitch_rate_radps,
                                                        max_yaw_rate_radps};

      aeromight_control::ControlAllocator control_allocator{actuator_min,
                                                            actuator_max,
                                                            yaw_saturation_limit_factor};

      aeromight_control::Control<decltype(attitude_controller),
                                 decltype(rate_controller),
                                 decltype(control_allocator),
                                 sys_time::ClockSource,
                                 LogClient>
          control{attitude_controller,
                  rate_controller,
                  control_allocator,
                  aeromight_boundaries::aeromight_data.actuator_control,
                  aeromight_boundaries::aeromight_data.control_health_storage,
                  aeromight_boundaries::aeromight_data.flight_control_setpoints,
                  data->state_estimation,
                  logger_control,
                  time_delta_limit_s,
                  max_roll_rate_radps,
                  max_pitch_rate_radps,
                  max_yaw_rate_radps,
                  max_tilt_angle_rad,
                  lift_throttle};

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

         if ((stale_sample) || (!act.enabled))
         {
            // Motors disabled → force minimum
            data.control_task_pwm_timer.set_compare_value_for_channel(Timer::OutputChannel::channel1, pwm_min_us);
            data.control_task_pwm_timer.set_compare_value_for_channel(Timer::OutputChannel::channel2, pwm_min_us);
            data.control_task_pwm_timer.set_compare_value_for_channel(Timer::OutputChannel::channel3, pwm_min_us);
            data.control_task_pwm_timer.set_compare_value_for_channel(Timer::OutputChannel::channel4, pwm_min_us);
            return;
         }

         // Enabled → write scaled PWM
         data.control_task_pwm_timer.set_compare_value_for_channel(Timer::OutputChannel::channel1, actuator_to_ccr(act.setpoints[0]));
         data.control_task_pwm_timer.set_compare_value_for_channel(Timer::OutputChannel::channel2, actuator_to_ccr(act.setpoints[1]));
         data.control_task_pwm_timer.set_compare_value_for_channel(Timer::OutputChannel::channel3, actuator_to_ccr(act.setpoints[2]));
         data.control_task_pwm_timer.set_compare_value_for_channel(Timer::OutputChannel::channel4, actuator_to_ccr(act.setpoints[3]));
      }
   }

}   // extern "C"
