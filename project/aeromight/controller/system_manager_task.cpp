#include "SystemManagerTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_params.hpp"
#include "aeromight_system/SystemManager.hpp"
#include "led/Led.hpp"
#include "logging/LogClient.hpp"
#include "rtos/QueueReceiver.hpp"
#include "rtos/QueueSender.hpp"
#include "rtos/periodic_task.hpp"
#include "sys_time/ClockSource.hpp"

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

      LogClient logger_system_manager_task{logging::logging_queue_sender, "system"};

      led::Led<hw::pcb_component::Led, sys_time::ClockSource> led{data->system_status_led};

      aeromight_system::SystemManagerParams system_params{};
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
                         aeromight_boundaries::aeromight_data.link_stats_actuals,
                         logger_system_manager_task,
                         system_params};

      rtos::run_periodic_task(system_manager);
   }

}   // extern "C"
