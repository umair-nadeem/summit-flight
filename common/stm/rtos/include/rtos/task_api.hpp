#pragma once

#include "rtos/RtosTaskConfig.hpp"
#include "task.h"

namespace rtos
{

TaskHandle_t create_task(const RtosTaskConfig& config);

void start_scheduler();

}   // namespace rtos
