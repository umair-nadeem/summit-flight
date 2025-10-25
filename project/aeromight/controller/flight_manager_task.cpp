#include "FlightManagerTaskData.hpp"
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

      constexpr uint32_t max_wait_sensors_readiness_ms            = 10'000u;
      constexpr uint32_t max_wait_estimation_control_readiness_ms = 2'000u;

      LogClient logger_flight_manager_task{logging::logging_queue_sender, "flight"};

      aeromight_flight::FlightManager<decltype(data->health_summary_queue_receiver),
                                      decltype(data->control_task_start_notifier),
                                      sys_time::ClockSource,
                                      LogClient>
          flight_manager{data->health_summary_queue_receiver,
                         data->control_task_start_notifier,
                         logger_flight_manager_task,
                         controller::task::flight_manager_task_period_in_ms,
                         max_wait_sensors_readiness_ms,
                         max_wait_estimation_control_readiness_ms};

      rtos::run_periodic_task(flight_manager);
   }

}   // extern "C"
