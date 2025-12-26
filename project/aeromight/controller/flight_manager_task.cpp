#include "FlightManagerTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_flight/FlightManager.hpp"
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

   [[noreturn]] void flight_manager_task(void* const params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::FlightManagerTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      constexpr float    stick_input_deadband_abs       = 0.1f;
      constexpr float    min_good_signal_rssi_dbm       = -110.0f;
      constexpr float    arming_throttle                = 0.05f;
      constexpr uint32_t max_age_stale_data_ms          = 1000u;
      constexpr uint32_t min_state_debounce_duration_ms = 200u;
      constexpr uint32_t timeout_sensors_readiness_ms   = 10'000u;
      constexpr uint32_t timeout_control_readiness_ms   = 2000u;
      constexpr uint32_t timeout_auto_land_ms           = 60'000u;   // 1 minute

      LogClient logger_flight_manager_task{logging::logging_queue_sender, "flight"};

      aeromight_flight::FlightManager<decltype(data->health_summary_queue_receiver),
                                      decltype(data->control_task_start_notifier),
                                      decltype(data->armed_status_led),
                                      sys_time::ClockSource,
                                      LogClient>
          flight_manager{data->health_summary_queue_receiver,
                         data->control_task_start_notifier,
                         data->armed_status_led,
                         aeromight_boundaries::aeromight_data.flight_control_setpoints,
                         aeromight_boundaries::aeromight_data.radio_control_setpoints,
                         aeromight_boundaries::aeromight_data.radio_link_actuals,
                         logger_flight_manager_task,
                         stick_input_deadband_abs,
                         min_good_signal_rssi_dbm,
                         arming_throttle,
                         controller::task::flight_manager_task_period_in_ms,
                         max_age_stale_data_ms,
                         min_state_debounce_duration_ms,
                         timeout_sensors_readiness_ms,
                         timeout_control_readiness_ms,
                         timeout_auto_land_ms};

      rtos::run_periodic_task(flight_manager);
   }

}   // extern "C"
