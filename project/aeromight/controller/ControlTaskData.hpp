#pragma once

#include "aeromight_control/StateEstimation.hpp"

extern "C"
{

   [[noreturn]] void control_task(void* const params);

}   // extern "C"

namespace controller
{

struct ControlTaskData
{
   aeromight_control::StateEstimation state_estimation{};
};

extern ControlTaskData control_task_data;
}   // namespace controller
