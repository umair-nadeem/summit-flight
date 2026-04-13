#include "SystemManagerTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_system/SystemManager.hpp"
#include "led/Led.hpp"
#include "logging/LogClient.hpp"
#include "rtos/QueueReceiver.hpp"
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

   [[noreturn]] void system_manager_task(void* const params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::SystemManagerTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      constexpr float    stick_input_deadband_abs       = 0.1f;
      constexpr float    min_good_signal_rssi_dbm       = -110.0f;
      constexpr uint32_t max_age_stale_data_ms          = 1000u;
      constexpr uint32_t min_state_debounce_duration_ms = 100u;
      constexpr uint32_t timeout_sensors_readiness_ms   = 10'000u;
      constexpr uint32_t timeout_control_readiness_ms   = 2000u;

      LogClient logger_system_manager_task{logging::logging_queue_sender, "system"};

      led::Led<hw::pcb_component::Led, sys_time::ClockSource> led{data->system_status_led};

      aeromight_system::SystemManager<decltype(data->health_summary_queue_receiver),
                                      rtos::Notifier,
                                      decltype(led),
                                      sys_time::ClockSource,
                                      LogClient>
          system_manager{data->health_summary_queue_receiver,
                         data->control_task_start_notifier,
                         data->imu_task_calibrate_notifier,
                         led,
                         aeromight_boundaries::aeromight_data.system_state_info,
                         aeromight_boundaries::aeromight_data.system_control_setpoints,
                         aeromight_boundaries::aeromight_data.radio_link_actuals,
                         logger_system_manager_task,
                         stick_input_deadband_abs,
                         min_good_signal_rssi_dbm,
                         controller::task::system_manager_task_period_in_ms,
                         max_age_stale_data_ms,
                         min_state_debounce_duration_ms,
                         timeout_sensors_readiness_ms,
                         timeout_control_readiness_ms};

      rtos::run_periodic_task(system_manager);
   }

}   // extern "C"
