#include <cstddef>
#include <cstdio>
#include <cstring>
#include <stdlib.h>

#include "LoggingTaskData.hpp"
#include "SensorAcquisitionTaskData.hpp"
#include "aeromight_boundaries/AeromightSensorData.hpp"
#include "error/error_record.hpp"
#include "hw/uart/uart.hpp"
#include "rtos/Queue.hpp"
#include "rtos/QueueSender.hpp"
#include "rtos/RtosTaskConfig.hpp"
#include "rtos/Semaphore.hpp"
#include "rtos/task_api.hpp"
#include "task_params.hpp"

namespace logging
{
rtos::QueueSender<params::LogBuffer> logging_queue_sender{};
}   // namespace logging

namespace aeromight_boundaries
{
AeromightSensorData aeromight_sensor_data{};
}   // namespace aeromight_boundaries

namespace controller
{

// all data
GlobalData                global_data{};
SensorAcquisitionTaskData sensor_acq_task_data{};
LoggingTaskData           logging_task_data{};

// task control blocks
rtos::TCB sensor_acq_task_tcb{};
rtos::TCB logging_task_tcb{};

// task stack
alignas(std::max_align_t) uint32_t sensor_acq_task_stack_buffer[controller::task::sensor_acq_task_stack_depth_in_words];
alignas(std::max_align_t) uint32_t logging_task_stack_buffer[controller::task::logging_task_stack_depth_in_words];

// queue
rtos::Queue<logging::params::logging_queue_len, sizeof(logging::params::LogBuffer)> logging_queue{};

// semaphore
rtos::Semaphore logging_uart_semaphore{};

// tasks
void register_tasks()
{
   // sensor acquisition task
   std::memset(sensor_acq_task_stack_buffer, 0, sizeof(sensor_acq_task_stack_buffer));

   rtos::RtosTaskConfig sensor_acq_task_config{
       .func                 = sensor_acquisition_task,
       .name                 = controller::task::sensor_acq_task_name,
       .stack_depth_in_words = controller::task::sensor_acq_task_stack_depth_in_words,
       .params               = static_cast<void*>(&sensor_acq_task_data),
       .priority             = controller::task::sensor_acq_task_priority,
       .stack_buffer         = sensor_acq_task_stack_buffer,
       .task_block           = sensor_acq_task_tcb};

   rtos::create_task(sensor_acq_task_config);

   // logging task
   std::memset(logging_task_stack_buffer, 0, sizeof(logging_task_stack_buffer));

   rtos::RtosTaskConfig logging_task_config{
       .func                 = logging_task,
       .name                 = controller::task::logging_task_name,
       .stack_depth_in_words = controller::task::logging_task_stack_depth_in_words,
       .params               = static_cast<void*>(&logging_task_data),
       .priority             = controller::task::logging_task_priority,
       .stack_buffer         = logging_task_stack_buffer,
       .task_block           = logging_task_tcb};

   rtos::create_task(logging_task_config);
}

void setup_uart()
{
   // logging uart (No need to call start_rx() for logging uart)
   auto logging_uart_sem_handle = logging_uart_semaphore.create();
   logging_task_data.logging_uart.transmitter_sem_taker.set_handle(logging_uart_sem_handle);
   logging_task_data.logging_uart.isr_sem_giver.set_handle(logging_uart_sem_handle);

   hw::uart::prepare_for_communication(logging_task_data.logging_uart.config,
                                       std::as_bytes(std::span{logging_task_data.logging_uart.dma_tx_buffer}),
                                       std::as_bytes(std::span{logging_task_data.logging_uart.dummy_dma_rx_buffer}));
}

void setup_spi()
{
   sensor_acq_task_data.spi1_master.prepare_for_communication();
}

void setup_queues()
{
   auto logging_queue_handle = logging_queue.create();
   ::logging::logging_queue_sender.set_handle(logging_queue_handle);
   logging_task_data.logging_queue_receiver.set_handle(logging_queue_handle);
}

}   // namespace controller

extern "C"
{
   void controller_initialize_hardware_and_start_scheduler()
   {
      if (error::has_no_error())
      {
         // hw & rtos objects
         controller::setup_uart();
         controller::setup_queues();

         controller::setup_spi();

         // tasks
         controller::register_tasks();

         // start scheduler
         rtos::start_scheduler();
      }
      else
      {
         __asm volatile("BKPT #0");
      }
   }
}   // extern "C"
