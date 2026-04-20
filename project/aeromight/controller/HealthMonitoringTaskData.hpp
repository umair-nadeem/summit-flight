#pragma once

#include "aeromight_boundaries/HealthSummary.hpp"
#include "hardware_bindings.hpp"
#include "hw/adc/Adc.hpp"
#include "rtos/QueueSender.hpp"

extern "C"
{
   [[noreturn]] void health_monitoring_task(void* const params);
}   // extern "C"

namespace controller
{

struct HealthMonitoringTaskData
{
   rtos::QueueSender<aeromight_boundaries::HealthSummary> health_summary_queue_sender{};

   // battery voltage sensing adc
   hw::adc::Adc voltage_sensing_adc{global_data.adc.adc1_config};
};

extern HealthMonitoringTaskData health_monitoring_task_data;

}   // namespace controller
