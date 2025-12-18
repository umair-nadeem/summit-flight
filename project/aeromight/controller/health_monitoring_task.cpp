#include "HealthMonitoringTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_health/HealthMonitoring.hpp"
#include "error/error_handler.hpp"
#include "logging/LogClient.hpp"
#include "rtos/periodic_task.hpp"
#include "sys_time/ClockSource.hpp"
#include "task_params.hpp"

namespace logging
{

extern rtos::QueueSender<params::LogBuffer> logging_queue_sender;

}   // namespace logging

extern "C"
{
   [[noreturn]] void health_monitoring_task([[maybe_unused]] void* const params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::HealthMonitoringTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      constexpr uint32_t wait_before_first_summary_update_ms      = 1000u;
      constexpr uint32_t max_wait_sensors_readiness_ms            = 10'000u;
      constexpr uint32_t max_wait_estimation_control_readiness_ms = 2'000u;
      constexpr uint32_t max_age_stale_imu_sensor_health_ms       = controller::task::health_monitoring_task_period_in_ms * 5u;
      constexpr uint32_t max_age_barometer_sensor_health_ms       = controller::task::health_monitoring_task_period_in_ms * 10u;
      constexpr uint32_t max_age_estimation_health_ms             = controller::task::health_monitoring_task_period_in_ms * 15u;
      constexpr uint32_t max_age_control_health_ms                = controller::task::health_monitoring_task_period_in_ms * 5u;

      LogClient logger_health_monitoring{logging::logging_queue_sender, "health"};

      aeromight_health::HealthMonitoring<decltype(data->health_summary_queue_sender),
                                         sys_time::ClockSource,
                                         LogClient>
          health_monitoring{data->health_summary_queue_sender,
                            aeromight_boundaries::aeromight_data.imu_sensor_health_storage,
                            aeromight_boundaries::aeromight_data.barometer_sensor_health_storage,
                            aeromight_boundaries::aeromight_data.estimator_health_storage,
                            aeromight_boundaries::aeromight_data.control_health_storage,
                            logger_health_monitoring,
                            controller::task::health_monitoring_task_period_in_ms,
                            wait_before_first_summary_update_ms,
                            max_wait_sensors_readiness_ms,
                            max_wait_estimation_control_readiness_ms,
                            max_age_stale_imu_sensor_health_ms,
                            max_age_barometer_sensor_health_ms,
                            max_age_estimation_health_ms,
                            max_age_control_health_ms};

      rtos::run_periodic_task(health_monitoring);
   }

}   // extern "C"
