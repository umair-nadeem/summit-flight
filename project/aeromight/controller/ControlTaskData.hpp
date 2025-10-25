#pragma once

#include "aeromight_boundaries/ControlTaskEvents.hpp"
#include "aeromight_control/StateEstimation.hpp"
#include "rtos/NotificationWaiter.hpp"

extern "C"
{

   [[noreturn]] void control_task(void* const params);

}   // extern "C"

namespace controller
{

struct ControlTaskData
{
   aeromight_control::StateEstimation state_estimation{};

   rtos::NotificationWaiter<aeromight_boundaries::ControlTaskNotificationFlags> control_task_notification_waiter{};
};

extern ControlTaskData control_task_data;
}   // namespace controller
