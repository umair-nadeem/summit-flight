#pragma once

extern "C"
{

   [[noreturn]] void control_task(void* const params);

}   // extern "C"

namespace controller
{

struct ControlTaskData
{
};

extern ControlTaskData control_task_data;
}   // namespace controller
