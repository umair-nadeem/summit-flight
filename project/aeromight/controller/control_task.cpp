#include "ControlTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_control/Control.hpp"
#include "aeromight_control/ControlAllocator.hpp"
#include "aeromight_control/EstimationAndControl.hpp"
#include "aeromight_estimation/AltitudeEkf.hpp"
#include "aeromight_estimation/Estimation.hpp"
#include "control/attitude/AttitudeController.hpp"
#include "control/rate/RateController.hpp"
#include "error/error_handler.hpp"
#include "estimation/AttitudeEstimator.hpp"
#include "event_handling/event_check.hpp"
#include "led/Led.hpp"
#include "logging/LogClient.hpp"
#include "math/ButterworthFilter.hpp"
#include "math/FirstOrderLpf.hpp"
#include "math/NullFilter.hpp"
#include "math/constants.hpp"
#include "math/make_filter.hpp"
#include "rc/StickCommandSource.hpp"
#include "rc/ThrottleCurve.hpp"
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

      using LogClient       = logging::LogClient<decltype(logging::logging_queue_sender)>;
      using StickFilterType = math::FirstOrderLpf;
      using GyroFilterType  = math::FirstOrderLpf;
      using DtermFilterType = math::NullFilter;

      // altitude ekf2 parameters
      constexpr float    process_noise_z                      = 1.0f;
      constexpr float    process_noise_vz                     = 5.0f;
      constexpr float    process_noise_accel_bias             = 0.0001f;
      constexpr float    measurement_noise_baro               = 0.5f;
      constexpr float    tilt_gating_attitude_angle_rad       = 0.2f;
      constexpr float    tilt_gating_accel_weight             = 0.08f;
      // estimation parameters
      constexpr bool     run_altitude_estimation              = false;
      constexpr uint32_t max_age_imu_data_ms                  = controller::task::imu_task_period_in_ms * 10u;
      constexpr uint32_t max_age_baro_data_ms                 = controller::task::barometer_task_period_in_ms * 10u;
      constexpr float    max_valid_imu_sample_dt_s            = 0.02f;
      constexpr float    max_valid_barometer_sample_dt_s      = 10.0f;
      // control parameters
      constexpr uint32_t max_age_state_estimation_data_ms     = controller::task::control_task_period_in_ms * 10u;
      constexpr float    min_dt_s                             = 0.002f;
      constexpr float    max_dt_s                             = 0.010f;
      constexpr float    stick_input_lpf_cutoff_hz            = 10.0f;
      constexpr float    gyro_lpf_cutoff_hz                   = 101.0f;
      constexpr float    pid_dterm_filter_cutoff_frequency_hz = 35.0f;
      constexpr float    thrust_linearization_factor          = 0.8f;
      // throttle and thrust
      constexpr float    thrust_limiting                      = 0.3f;
      constexpr float    actuator_min                         = 0.0f;
      constexpr float    actuator_max                         = 1.0f - thrust_limiting;
      constexpr float    actuator_idle                        = 0.1f;
      constexpr float    throttle_arming                      = 0.01f;
      constexpr float    throttle_hover                       = 0.3f;
      constexpr float    throttle_curve_factor                = 0.7f;
      constexpr float    throttle_gate_integrator             = 0.15f;
      // attitude controller
      constexpr bool     run_attitude_controller              = true;
      constexpr float    max_tilt_angle_rad                   = 30 * math::constants::deg_to_rad;
      constexpr float    max_roll_rate_radps                  = 3.0f;
      constexpr float    max_pitch_rate_radps                 = 3.0f;
      constexpr float    max_yaw_rate_radps                   = 3.0f;

      // control allocator
      constexpr float thrust_deadband             = 0.01f;
      constexpr float yaw_saturation_limit_factor = 0.25f;
      constexpr float slew_rate_limit_s           = 1.0f;

      LogClient logger_estimation{logging::logging_queue_sender, "estimation"};
      LogClient logger_control{logging::logging_queue_sender, "control"};

      estimation::AttitudeEstimatorParams ahrs_params{};
      estimation::AttitudeEstimator       ahrs_filter{ahrs_params};

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
                     aeromight_boundaries::aeromight_data.estimator_health,
                     data->state_estimation,
                     aeromight_boundaries::aeromight_data.imu_data,
                     aeromight_boundaries::aeromight_data.barometer_data,
                     logger_estimation,
                     run_altitude_estimation,
                     max_age_imu_data_ms,
                     max_age_baro_data_ms,
                     max_valid_imu_sample_dt_s,
                     max_valid_barometer_sample_dt_s};

      control::attitude::AttitudeControllerParams attitude_controller_params{};
      control::attitude::AttitudeController       attitude_controller{attitude_controller_params};

      control::rate::RateControllerParams rate_controller_params{};
      control::rate::RateController       rate_controller{rate_controller_params};

      aeromight_control::ControlAllocator control_allocator{actuator_min,
                                                            actuator_max,
                                                            actuator_idle,
                                                            thrust_deadband,
                                                            yaw_saturation_limit_factor,
                                                            slew_rate_limit_s};

      rc::ThrottleCurve throttle_curve{throttle_hover, throttle_curve_factor};

      auto roll_input_lpf  = math::make_filter<StickFilterType>(stick_input_lpf_cutoff_hz);
      auto pitch_input_lpf = math::make_filter<StickFilterType>(stick_input_lpf_cutoff_hz);
      auto yaw_input_lpf   = math::make_filter<StickFilterType>(stick_input_lpf_cutoff_hz);

      rc::StickCommandSourceParams stick_command_source_params{actuator_min, actuator_max};
      rc::StickCommandSource<decltype(roll_input_lpf),
                             decltype(throttle_curve)>
          stick_command_source{roll_input_lpf,
                               pitch_input_lpf,
                               yaw_input_lpf,
                               throttle_curve,
                               aeromight_boundaries::aeromight_data.stick_command,
                               stick_command_source_params};

      auto gyro_x_lpf = math::make_filter<GyroFilterType>(gyro_lpf_cutoff_hz);
      auto gyro_y_lpf = math::make_filter<GyroFilterType>(gyro_lpf_cutoff_hz);
      auto gyro_z_lpf = math::make_filter<GyroFilterType>(gyro_lpf_cutoff_hz);

      auto pid_dterm_x_lpf = math::make_filter<DtermFilterType>(pid_dterm_filter_cutoff_frequency_hz,
                                                                controller::task::control_task_period_in_ms / 1000.0f);
      auto pid_dterm_y_lpf = math::make_filter<DtermFilterType>(pid_dterm_filter_cutoff_frequency_hz,
                                                                controller::task::control_task_period_in_ms / 1000.0f);
      auto pid_dterm_z_lpf = math::make_filter<DtermFilterType>(pid_dterm_filter_cutoff_frequency_hz,
                                                                controller::task::control_task_period_in_ms / 1000.0f);

      led::Led<hw::pcb_component::Led, sys_time::ClockSource> led{data->control_status_led};

      const math::Vec3f max_rate{max_roll_rate_radps, max_pitch_rate_radps, max_yaw_rate_radps};

      aeromight_control::Control<decltype(attitude_controller),
                                 decltype(rate_controller),
                                 decltype(control_allocator),
                                 decltype(stick_command_source),
                                 decltype(data->dshot),
                                 decltype(gyro_x_lpf),
                                 decltype(pid_dterm_x_lpf),
                                 decltype(led),
                                 sys_time::ClockSource,
                                 LogClient>
          control{attitude_controller,
                  rate_controller,
                  control_allocator,
                  stick_command_source,
                  data->dshot,
                  gyro_x_lpf,
                  gyro_y_lpf,
                  gyro_z_lpf,
                  pid_dterm_x_lpf,
                  pid_dterm_y_lpf,
                  pid_dterm_z_lpf,
                  led,
                  data->motor_output_map,
                  aeromight_boundaries::aeromight_data.control_health,
                  aeromight_boundaries::aeromight_data.system_state_info,
                  data->state_estimation,
                  logger_control,
                  max_age_state_estimation_data_ms,
                  min_dt_s,
                  max_dt_s,
                  run_attitude_controller,
                  max_tilt_angle_rad,
                  max_rate,
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
      if ((event_bits == 0) ||
          (!event_handling::has_event(event_bits, aeromight_boundaries::ControlTaskEvents::start)))
      {
         error::stop_operation();
      }

      // start pwm
      data->slave_pwm_timer.enable_channel(data->slave_pwm_timer_channels);
      data->master_pwm_timer.enable_channel(data->master_pwm_timer_channels);

      data->master_pwm_timer.enable_all_outputs();

      data->slave_pwm_timer.enable_counter();
      data->master_pwm_timer.enable_counter();

      estimation_and_control.start();

      rtos::run_periodic_task(estimation_and_control);
   }

}   // extern "C"
