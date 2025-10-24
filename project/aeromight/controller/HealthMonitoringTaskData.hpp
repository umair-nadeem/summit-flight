#pragma once

#include "aeromight_boundaries/HealthSummary.hpp"
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
};

extern HealthMonitoringTaskData health_monitoring_task_data;

}   // namespace controller
