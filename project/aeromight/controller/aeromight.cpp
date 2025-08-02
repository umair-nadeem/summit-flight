#include <cstddef>
#include <cstdio>
#include <cstring>
#include <stdlib.h>

#include "SensorAcquisitionTaskData.h"
#include "error/error_record.h"
#include "hw/uart/uart.hpp"
#include "main.h"
#include "rtos/Queue.hpp"
#include "rtos/RtosTaskConfig.h"
#include "rtos/Semaphore.hpp"
#include "rtos/task_api.h"

namespace controller
{

// task data
SensorAcquisitionTaskData sensor_acq_task_data{};

// task control blocks
rtos::TCB sensor_acq_task_tcb{};

// task stack
uint32_t sensor_acq_task_stack_buffer[controller::task::sensor_acq_task_stack_depth_in_words];

// queue
alignas(std::max_align_t) std::byte logging_queue_storage[10u * 64u];

// semaphore
rtos::Semaphore logging_uart_semaphore{};

// tasks
void register_sensor_acquisition_task()
{
   std::memset(sensor_acq_task_stack_buffer, 0, sizeof(sensor_acq_task_stack_buffer));

   rtos::RtosTaskConfig sensor_acq_task_config{
       .func                 = sensor_acquisition_task,
       .name                 = controller::task::sensor_acq_task_name,
       .stack_depth_in_words = controller::task::sensor_acq_task_stack_size_in_bytes / sizeof(uint32_t),
       .params               = static_cast<void*>(&sensor_acq_task_data),
       .priority             = controller::task::sensor_acq_task_priority,
       .stack_buffer         = sensor_acq_task_stack_buffer,
       .task_block           = sensor_acq_task_tcb};

   rtos::create_task(sensor_acq_task_config);
}

void setup_rtos_objects()
{
   // logging uart
   auto logging_uart_sem_handle = logging_uart_semaphore.create(true);
   sensor_acq_task_data.logging_uart.transmitter_sem_taker.set_handle(logging_uart_sem_handle);
   sensor_acq_task_data.logging_uart.isr_sem_giver.set_handle(logging_uart_sem_handle);

   hw::uart::prepare_for_communication(sensor_acq_task_data.logging_uart.config,
                                       std::as_bytes(std::span{sensor_acq_task_data.logging_uart.dma_tx_buffer}),
                                       std::as_bytes(std::span{sensor_acq_task_data.logging_uart.dummy_dma_rx_buffer}));
}

}   // namespace controller

extern "C"
{
   void controller_initialize_hardware_and_start_scheduler()
   {
      if (error::has_no_error())
      {
         // tasks
         controller::register_sensor_acquisition_task();

         // rtos
         controller::setup_rtos_objects();

         // start scheduler
         rtos::start_scheduler();
      }
      else
      {
         __asm volatile("BKPT #0");
      }
   }
}
