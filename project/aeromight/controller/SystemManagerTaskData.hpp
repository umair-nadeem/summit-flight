#pragma once

#include "aeromight_boundaries/ControlTaskEvents.hpp"
#include "aeromight_boundaries/HealthSummary.hpp"
#include "aeromight_boundaries/ImuTaskEvents.hpp"
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
   hw::pcb_component::Led system_status_led{global_data.gpios.system_led};

   rtos::QueueReceiver<aeromight_boundaries::HealthSummary> health_summary_queue_receiver{};

   // task notifications
   rtos::Notifier control_task_start_notifier{utilities::enum_to_bit_mask<aeromight_boundaries::ControlTaskEvents::start>()};
   rtos::Notifier imu_task_calibrate_notifier{utilities::enum_to_bit_mask<aeromight_boundaries::ImuTaskEvents::calibrate>()};
};

extern SystemManagerTaskData system_manager_task_data;

}   // namespace controller
