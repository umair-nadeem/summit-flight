#pragma once

#include "aeromight_boundaries/ControlTaskEvents.hpp"
#include "aeromight_boundaries/HealthSummary.hpp"
#include "rtos/Notifier.hpp"
#include "rtos/QueueReceiver.hpp"

extern "C"
{

   [[noreturn]] void flight_manager_task(void* const params);

}   // extern "C"

namespace controller
{

struct FlightManagerTaskData
{
   rtos::QueueReceiver<aeromight_boundaries::HealthSummary> health_summary_queue_receiver{};

   aeromight_boundaries::ControlTaskNotificationFlags                 control_task_start_notification{aeromight_boundaries::pos_to_value(aeromight_boundaries::ControlTaskEvents::start)};
   rtos::Notifier<aeromight_boundaries::ControlTaskNotificationFlags> control_task_start_notifier{control_task_start_notification};
};

extern FlightManagerTaskData flight_manager_task_data;

}   // namespace controller
