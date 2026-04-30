#include <chrono>
#include <cstdio>
#include <thread>

#include "MainLoopData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_control/Control.hpp"
#include "aeromight_control/ControlAllocator.hpp"
#include "aeromight_estimation/AltitudeEkf.hpp"
#include "aeromight_estimation/Estimation.hpp"
#include "aeromight_health/HealthMonitoring.hpp"
#include "aeromight_params.hpp"
#include "aeromight_system/SystemManager.hpp"
#include "control/attitude/AttitudeController.hpp"
#include "control/rate/RateController.hpp"
#include "estimation/AttitudeEstimator.hpp"
#include "imu/Imu.hpp"
#include "led/Led.hpp"
#include "logging/Logger.hpp"
#include "math/FirstOrderLpf.hpp"
#include "math/NullFilter.hpp"
#include "math/constants.hpp"
#include "math/make_filter.hpp"
#include "rc/StickCommandSource.hpp"
#include "rc/ThrottleCurve.hpp"
#include "sys_time/ClockSource.hpp"

namespace sitl
{

void run_main_loop(void* const params)
{
   error::verify(params != nullptr);
   auto* data = static_cast<MainLoopData*>(params);

   using StickFilterType = math::FirstOrderLpf;
   using GyroFilterType  = math::FirstOrderLpf;
   using DtermFilterType = math::NullFilter;

   constexpr float stick_input_lpf_cutoff_hz            = 10.0f;
   constexpr float gyro_lpf_cutoff_hz                   = 101.0f;
   constexpr float pid_dterm_filter_cutoff_frequency_hz = 35.0f;

   logging::Logger logger_imu{"imu"};
   logging::Logger logger_estimation{"estimation"};
   logging::Logger logger_control{"control"};
   logging::Logger logger_health_monitoring{"health"};
   logging::Logger logger_system_manager{"system"};

   imu::ImuParams                                   imu_params{};
   imu::Imu<sys_time::ClockSource, logging::Logger> imu{aeromight_boundaries::aeromight_data.imu_data,
                                                        aeromight_boundaries::aeromight_data.imu_health,
                                                        logger_imu,
                                                        imu_params};

   estimation::AttitudeEstimatorParams ahrs_params{};
   estimation::AttitudeEstimator       ahrs_filter{ahrs_params};

   aeromight_estimation::AltitudeEkfParams ekf2_params{};
   aeromight_estimation::AltitudeEkf       altitude_ekf{ekf2_params};

   aeromight_estimation::EstimationParams estimation_params{};
   aeromight_estimation::Estimation<decltype(ahrs_filter),
                                    decltype(altitude_ekf),
                                    sys_time::ClockSource,
                                    logging::Logger>
       estimation{ahrs_filter,
                  altitude_ekf,
                  aeromight_boundaries::aeromight_data.estimator_health,
                  data->state_estimation,
                  aeromight_boundaries::aeromight_data.imu_data,
                  aeromight_boundaries::aeromight_data.barometer_data,
                  logger_estimation,
                  estimation_params};

   control::attitude::AttitudeControllerParams attitude_controller_params{};
   control::attitude::AttitudeController       attitude_controller{attitude_controller_params};

   control::rate::RateControllerParams rate_controller_params{};
   control::rate::RateController       rate_controller{rate_controller_params};

   aeromight_control::ControlAllocatorParams allocator_params{};
   aeromight_control::ControlAllocator       control_allocator{allocator_params};

   rc::ThrottleCurveParams throttle_curve_params{};
   rc::ThrottleCurve       throttle_curve{throttle_curve_params};

   auto roll_input_lpf  = math::make_filter<StickFilterType>(stick_input_lpf_cutoff_hz);
   auto pitch_input_lpf = math::make_filter<StickFilterType>(stick_input_lpf_cutoff_hz);
   auto yaw_input_lpf   = math::make_filter<StickFilterType>(stick_input_lpf_cutoff_hz);

   rc::StickCommandSourceParams stick_command_source_params{allocator_params.actuator_min, allocator_params.actuator_max};
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
                                                             control_task_period_in_ms / 1000.0f);
   auto pid_dterm_y_lpf = math::make_filter<DtermFilterType>(pid_dterm_filter_cutoff_frequency_hz,
                                                             control_task_period_in_ms / 1000.0f);
   auto pid_dterm_z_lpf = math::make_filter<DtermFilterType>(pid_dterm_filter_cutoff_frequency_hz,
                                                             control_task_period_in_ms / 1000.0f);

   led::Led<sitl::pcb_component::Led, sys_time::ClockSource> led{data->control_led};

   aeromight_control::ControlParams control_params{};
   aeromight_control::Control<decltype(attitude_controller),
                              decltype(rate_controller),
                              decltype(control_allocator),
                              decltype(stick_command_source),
                              decltype(data->actuator_control),
                              decltype(gyro_x_lpf),
                              decltype(pid_dterm_x_lpf),
                              decltype(led),
                              sys_time::ClockSource,
                              logging::Logger>
       control{attitude_controller,
               rate_controller,
               control_allocator,
               stick_command_source,
               data->actuator_control,
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
               control_params};

   data->adc.set_raw_value(16u);
   aeromight_health::HealthMonitoringParams health_params{};
   aeromight_health::HealthMonitoring<decltype(data->battery),
                                      decltype(data->health_summary_queue),
                                      sys_time::ClockSource,
                                      logging::Logger>
       health_monitoring{data->battery,
                         data->health_summary_queue,
                         aeromight_boundaries::aeromight_data.battery_status,
                         aeromight_boundaries::aeromight_data.imu_health,
                         aeromight_boundaries::aeromight_data.barometer_health,
                         aeromight_boundaries::aeromight_data.estimator_health,
                         aeromight_boundaries::aeromight_data.control_health,
                         logger_health_monitoring,
                         health_params};

   led::Led<sitl::pcb_component::Led, sys_time::ClockSource> status_led{data->status_led};

   aeromight_system::SystemManagerParams system_params{};
   aeromight_system::SystemManager<decltype(data->health_summary_queue),
                                   rtos::Notification,
                                   decltype(status_led),
                                   sys_time::ClockSource,
                                   logging::Logger>
       system_manager{data->health_summary_queue,
                      data->control_task_start_notifier,
                      data->imu_task_calibrate_notifier,
                      status_led,
                      aeromight_boundaries::aeromight_data.system_state_info,
                      aeromight_boundaries::aeromight_data.system_control_setpoints,
                      aeromight_boundaries::aeromight_data.link_stats_actuals,
                      logger_system_manager,
                      system_params};

   constexpr std::chrono::microseconds period{imu_task_period_in_ms * 1000u};

   auto next_wake_time = std::chrono::steady_clock::now() + period;

   std::printf("starting aeromight sitl main loop\n");

   uint32_t tick = 0;

   while (true)
   {
      imu.execute(data->imu_sensor_data, data->imu_sensor_status);

      estimation.execute();

      control.execute();

      if ((tick % (health_monitoring_task_period_in_ms / imu_task_period_in_ms)) == 0)
      {
         health_monitoring.run_once();
      }

      if ((tick % (system_manager_task_period_in_ms / imu_task_period_in_ms)) == 0)
      {
         system_manager.run_once();
      }

      // Equivalent to vTaskDelayUntil
      std::this_thread::sleep_until(next_wake_time);
      next_wake_time += period;
   }
}

}   // namespace sitl
