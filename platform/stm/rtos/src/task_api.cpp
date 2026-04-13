#include "rtos/task_api.hpp"

#include <cassert>
#include <type_traits>

namespace rtos
{

TaskHandle_t create_task(const RtosTaskConfig& config)
{
   static_assert(std::is_pointer_v<decltype(config.func)>);
   static_assert(std::is_pointer_v<decltype(config.params)>);
   static_assert(std::is_pointer_v<decltype(config.stack_buffer)>);

   TaskHandle_t task_handle = nullptr;

   task_handle = xTaskCreateStatic(config.func,
                                   config.name,
                                   config.stack_depth_in_words,
                                   config.params,
                                   config.priority,
                                   config.stack_buffer,
                                   &config.task_block);

   assert(task_handle != nullptr);

   return task_handle;
}

void start_scheduler()
{
   vTaskStartScheduler();
}

}   // namespace rtos
