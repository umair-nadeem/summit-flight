#include "ControlTaskData.hpp"
#include "aeromight_boundaries/AeromightSensorData.hpp"
#include "aeromight_control/AltitudeEkf.hpp"
#include "aeromight_control/Control.hpp"
#include "aeromight_control/Estimation.hpp"
#include "aeromight_control/EstimationAndControl.hpp"
#include "error/error_handler.hpp"
#include "estimation/MadgwickFilter.hpp"
#include "logging/LogClient.hpp"
#include "rtos/QueueSender.hpp"
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

      // madgwick parameters
      constexpr float ahrs_beta            = 0.3f;
      constexpr float gyro_bias_gain       = 0.5f;
      constexpr float accel_tolerance_mps2 = 3.0f;

      // altitude ekf2 parameters
      constexpr float process_noise_z          = 1e-3f;
      constexpr float process_noise_vz         = 1e-2f;
      constexpr float process_noise_accel_bias = 1e-4f;
      constexpr float measurement_noise_baro   = 1.0f;

      constexpr uint8_t     num_samples_pressure_reference                 = 10u;
      constexpr uint8_t     max_recovery_attempts                          = 3u;
      constexpr std::size_t wait_timeout_pressure_reference_acquisition_ms = controller::task::barometer_task_period_in_ms * 50u;
      constexpr std::size_t max_age_imu_data_ms                            = controller::task::imu_task_period_in_ms * 10u;
      constexpr std::size_t max_age_baro_data_ms                           = controller::task::barometer_task_period_in_ms * 10u;

      LogClient logger_estimation{logging::logging_queue_sender, "estimation"};
      LogClient logger_control{logging::logging_queue_sender, "control"};

      estimation::MadgwickFilter ahrs_filter{ahrs_beta, gyro_bias_gain, accel_tolerance_mps2, (controller::task::imu_task_period_in_ms / 1000.0f)};

      aeromight_control::AltitudeEkf altitude_ekf{process_noise_z,
                                                  process_noise_vz,
                                                  process_noise_accel_bias,
                                                  measurement_noise_baro,
                                                  (controller::task::imu_task_period_in_ms / 1000.0f)};

      aeromight_control::Estimation<decltype(ahrs_filter),
                                    decltype(altitude_ekf),
                                    sys_time::ClockSource,
                                    LogClient>
          estimation{ahrs_filter,
                     altitude_ekf,
                     aeromight_boundaries::aeromight_sensor_data.estimator_health_storage,
                     data->state_estimation,
                     aeromight_boundaries::aeromight_sensor_data.imu_sensor_data_storage,
                     aeromight_boundaries::aeromight_sensor_data.barometer_sensor_data_storage,
                     logger_estimation,
                     max_recovery_attempts,
                     num_samples_pressure_reference,
                     controller::task::control_task_period_in_ms,
                     wait_timeout_pressure_reference_acquisition_ms,
                     max_age_imu_data_ms,
                     max_age_baro_data_ms};

      aeromight_control::Control<LogClient> control{logger_control};

      aeromight_control::EstimationAndControl<decltype(estimation),
                                              decltype(control)>
          estimation_and_control{estimation,
                                 control,
                                 controller::task::control_task_period_in_ms};

      estimation_and_control.start();
      rtos::run_periodic_task(estimation_and_control);
   }

}   // extern "C"
