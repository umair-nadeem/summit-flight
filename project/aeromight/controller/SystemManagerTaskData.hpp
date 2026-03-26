#pragma once

#include "aeromight_boundaries/ControlTaskEvents.hpp"
#include "aeromight_boundaries/HealthSummary.hpp"
#include "hardware_bindings.hpp"
#include "hw/pcb_component/Led.hpp"
#include "rtos/Notifier.hpp"
#include "rtos/QueueReceiver.hpp"
#include "utilities/enum_to_bit_mask.hpp"

extern "C"
{

   [[noreturn]] void system_manager_task(void* const params);

}   // extern "C"

namespace controller
{

struct SystemManagerTaskData
{
   // armed status led
   hw::pcb_component::Led armed_status_led{global_data.gpios.armed_status_led};

   rtos::QueueReceiver<aeromight_boundaries::HealthSummary> health_summary_queue_receiver{};

   // task notifications
   rtos::Notifier control_task_start_notifier{utilities::enum_to_bit_mask<aeromight_boundaries::ControlTaskEvents::start>()};
};

extern SystemManagerTaskData system_manager_task_data;

}   // namespace controller
