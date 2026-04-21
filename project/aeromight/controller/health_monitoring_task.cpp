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

      // battery
      constexpr float battery_voltage_calibration_constant = 0.988f;

      LogClient logger_health_monitoring{logging::logging_queue_sender, "health"};

      power::battery::VoltageSenseConfig battery_voltage_sense{};

      power::battery::PercentageModelLipo4S percentage_convertor{};

      power::battery::Battery<decltype(data->voltage_sensing_adc),
                              decltype(percentage_convertor)>
          battery{data->voltage_sensing_adc,
                  percentage_convertor,
                  battery_voltage_sense,
                  battery_voltage_calibration_constant};

      aeromight_health::HealthMonitoringParams health_params{};
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
                            health_params};

      rtos::run_periodic_task(health_monitoring);
   }

}   // extern "C"
