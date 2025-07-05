#include <stdlib.h>
#include <cstdio>
#include <cstring>

#include "rtos/task_interface.hpp"
#include "rtos/RtosTaskConfig.h"

#include "main.h"
#include "SensorAcquisitionTaskData.h"

namespace controller
{

// task data
SensorAcquisitionTaskData sensor_acq_task_data{};

// task control blocks
rtos::TCB sensor_acq_task_tcb{};

// task stack
uint32_t sensor_acq_task_stack_buffer[controller::sensor_acq_task_stack_depth_in_words];

void register_tasks()
{
   std::memset(sensor_acq_task_stack_buffer, 0, sizeof(sensor_acq_task_stack_buffer));

   rtos::RtosTaskConfig sensor_acq_task_config{
      .func = sensor_acquisition_task,
      .name = controller::sensor_acq_task_name,
      .stack_depth_in_words = controller::sensor_acq_task_stack_size_in_bytes / sizeof(uint32_t),
      .params = static_cast<void*>(&sensor_acq_task_data),
      .priority = controller::sensor_acq_task_priority,
      .stack_buffer = sensor_acq_task_stack_buffer,
      .task_block = sensor_acq_task_tcb};

   rtos::create_task(sensor_acq_task_config);
}

}  // namespace controller

extern "C"
{

void controller_register_rtos_objects()
{
   controller::register_tasks();
}

void controller_start_scheduler()
{
   rtos::start_scheduler();
}

}
