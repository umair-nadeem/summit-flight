#include "HealthMonitoringTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_health/HealthMonitoring.hpp"
#include "error/error_handler.hpp"
#include "logging/LogClient.hpp"
#include "power/battery/Battery.hpp"
#include "power/battery/BatteryPercentageModel.hpp"
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

      constexpr uint32_t max_age_stale_imu_sensor_health_ms = controller::task::health_monitoring_task_period_in_ms * 5u;
      constexpr uint32_t max_age_barometer_sensor_health_ms = controller::task::health_monitoring_task_period_in_ms * 10u;
      constexpr uint32_t max_age_estimation_health_ms       = controller::task::health_monitoring_task_period_in_ms * 15u;
      constexpr uint32_t max_age_control_health_ms          = controller::task::health_monitoring_task_period_in_ms * 5u;
      constexpr bool     evaluate_barometer_health          = false;

      // battery
      constexpr float    voltage_divider_r1_ohm               = 100'000.0f;
      constexpr float    voltage_divider_r2_ohm               = 22'000.0f;
      constexpr float    vref_v                               = 3.3f;
      constexpr uint16_t adc_resolution                       = 4095u;
      constexpr float    battery_voltage_calibration_constant = 0.988f;

      LogClient logger_health_monitoring{logging::logging_queue_sender, "health"};

      power::battery::VoltageSenseConfig battery_voltage_sense{voltage_divider_r1_ohm, voltage_divider_r2_ohm, vref_v, adc_resolution};

      power::battery::PercentageModelLipo4S percentage_convertor{};

      power::battery::Battery<decltype(data->voltage_sensing_adc),
                              decltype(percentage_convertor)>
          battery{data->voltage_sensing_adc,
                  percentage_convertor,
                  battery_voltage_sense,
                  battery_voltage_calibration_constant};

      aeromight_health::HealthMonitoring<decltype(battery),
                                         decltype(data->health_summary_queue_sender),
                                         sys_time::ClockSource,
                                         LogClient>
          health_monitoring{battery,
                            data->health_summary_queue_sender,
                            aeromight_boundaries::aeromight_data.battery_status,
                            aeromight_boundaries::aeromight_data.imu_health,
                            aeromight_boundaries::aeromight_data.barometer_health,
                            aeromight_boundaries::aeromight_data.estimator_health,
                            aeromight_boundaries::aeromight_data.control_health,
                            logger_health_monitoring,
                            controller::task::health_monitoring_task_period_in_ms,
                            max_age_stale_imu_sensor_health_ms,
                            max_age_barometer_sensor_health_ms,
                            max_age_estimation_health_ms,
                            max_age_control_health_ms,
                            evaluate_barometer_health};

      rtos::run_periodic_task(health_monitoring);
   }

}   // extern "C"
