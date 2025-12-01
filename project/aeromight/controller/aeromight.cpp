#include <cstddef>
#include <cstdio>
#include <cstring>
#include <stdlib.h>

#include "BarometerTaskData.hpp"
#include "ControlTaskData.hpp"
#include "FlightManagerTaskData.hpp"
#include "HealthMonitoringTaskData.hpp"
#include "ImuTaskData.hpp"
#include "LoggingTaskData.hpp"
#include "RadioLinkTaskData.hpp"
#include "SysClockData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "error/error_record.hpp"
#include "hw/uart/uart.hpp"
#include "rtos/Queue.hpp"
#include "rtos/QueueSender.hpp"
#include "rtos/RtosTaskConfig.hpp"
#include "rtos/Semaphore.hpp"
#include "rtos/task_api.hpp"
#include "sys_time/ClockSource.hpp"
#include "task_params.hpp"

namespace logging
{
rtos::QueueSender<params::LogBuffer> logging_queue_sender{};
}   // namespace logging

namespace aeromight_boundaries
{
AeromightData aeromight_data{};
}   // namespace aeromight_boundaries

namespace controller
{

// all data
GlobalData               global_data{};
ImuTaskData              imu_task_data{};
ControlTaskData          control_task_data{};
FlightManagerTaskData    flight_manager_task_data{};
RadioLinkTaskData        radio_link_task_data{};
HealthMonitoringTaskData health_monitoring_task_data{};
BarometerTaskData        barometer_task_data{};
LoggingTaskData          logging_task_data{};
SysClockData             sys_clock_data{};

// task control blocks
rtos::TCB imu_task_tcb{};
rtos::TCB control_task_tcb{};
rtos::TCB flight_manager_task_tcb{};
rtos::TCB radio_link_task_tcb{};
rtos::TCB health_monitoring_task_tcb{};
rtos::TCB barometer_task_tcb{};
rtos::TCB logging_task_tcb{};

// task stack
alignas(std::max_align_t) uint32_t imu_task_stack_buffer[controller::task::imu_task_stack_depth_in_words];
alignas(std::max_align_t) uint32_t control_task_stack_buffer[controller::task::control_task_stack_depth_in_words];
alignas(std::max_align_t) uint32_t flight_manager_task_stack_buffer[controller::task::flight_manager_task_stack_depth_in_words];
alignas(std::max_align_t) uint32_t radio_link_task_stack_buffer[controller::task::radio_link_task_stack_depth_in_words];
alignas(std::max_align_t) uint32_t health_monitoring_task_stack_buffer[controller::task::health_monitoring_task_stack_depth_in_words];
alignas(std::max_align_t) uint32_t barometer_task_stack_buffer[controller::task::barometer_task_stack_depth_in_words];
alignas(std::max_align_t) uint32_t logging_task_stack_buffer[controller::task::logging_task_stack_depth_in_words];

// task handles
TaskHandle_t imu_task_handle;
TaskHandle_t control_task_handle;
TaskHandle_t flight_manager_task_handle;
TaskHandle_t radio_link_task_handle;
TaskHandle_t health_monitoring_task_handle;
TaskHandle_t barometer_task_handle;
TaskHandle_t logging_task_handle;

// queues
rtos::Queue<logging::params::logging_queue_depth, sizeof(logging::params::LogBuffer)>                      logging_queue{};
rtos::Queue<aeromight_boundaries::health_summary_queue_depth, sizeof(aeromight_boundaries::HealthSummary)> health_summary_queue{};
rtos::Queue<radio_link_task_data.queue_depth, sizeof(RadioLinkTaskData::RadioLinkUart::Message)>           radio_input_queue{};
rtos::Queue<radio_link_task_data.queue_depth, sizeof(std::size_t)>                                         radio_task_buffer_index_queue{};

// semaphore
rtos::Semaphore logging_uart_semaphore{};
rtos::Semaphore radio_input_semaphore{};

// tasks
void register_tasks()
{
   // imu task
   std::memset(imu_task_stack_buffer, 0, sizeof(imu_task_stack_buffer));

   rtos::RtosTaskConfig imu_task_config{
       .func                 = imu_task,
       .name                 = controller::task::imu_task_name,
       .stack_depth_in_words = controller::task::imu_task_stack_depth_in_words,
       .params               = static_cast<void*>(&imu_task_data),
       .priority             = controller::task::imu_task_priority,
       .stack_buffer         = imu_task_stack_buffer,
       .task_block           = imu_task_tcb};

   imu_task_handle = rtos::create_task(imu_task_config);

   // control task
   std::memset(control_task_stack_buffer, 0, sizeof(control_task_stack_buffer));

   rtos::RtosTaskConfig control_task_config{
       .func                 = control_task,
       .name                 = controller::task::control_task_name,
       .stack_depth_in_words = controller::task::control_task_stack_depth_in_words,
       .params               = static_cast<void*>(&control_task_data),
       .priority             = controller::task::control_task_priority,
       .stack_buffer         = control_task_stack_buffer,
       .task_block           = control_task_tcb};

   control_task_handle = rtos::create_task(control_task_config);

   // flight manager task
   std::memset(flight_manager_task_stack_buffer, 0, sizeof(flight_manager_task_stack_buffer));

   rtos::RtosTaskConfig flight_manager_task_config{
       .func                 = flight_manager_task,
       .name                 = controller::task::flight_manager_task_name,
       .stack_depth_in_words = controller::task::flight_manager_task_stack_depth_in_words,
       .params               = static_cast<void*>(&flight_manager_task_data),
       .priority             = controller::task::flight_manager_task_priority,
       .stack_buffer         = flight_manager_task_stack_buffer,
       .task_block           = flight_manager_task_tcb};

   flight_manager_task_handle = rtos::create_task(flight_manager_task_config);

   // radio link task
   std::memset(radio_link_task_stack_buffer, 0, sizeof(radio_link_task_stack_buffer));

   rtos::RtosTaskConfig radio_link_task_config{
       .func                 = radio_link_task,
       .name                 = controller::task::radio_link_task_name,
       .stack_depth_in_words = controller::task::radio_link_task_stack_depth_in_words,
       .params               = static_cast<void*>(&radio_link_task_data),
       .priority             = controller::task::radio_link_task_priority,
       .stack_buffer         = radio_link_task_stack_buffer,
       .task_block           = radio_link_task_tcb};

   radio_link_task_handle = rtos::create_task(radio_link_task_config);

   // health monitoring task
   std::memset(health_monitoring_task_stack_buffer, 0, sizeof(health_monitoring_task_stack_buffer));

   rtos::RtosTaskConfig health_monitoring_task_config{
       .func                 = health_monitoring_task,
       .name                 = controller::task::health_monitoring_task_name,
       .stack_depth_in_words = controller::task::health_monitoring_task_stack_depth_in_words,
       .params               = static_cast<void*>(&health_monitoring_task_data),
       .priority             = controller::task::health_monitoring_task_priority,
       .stack_buffer         = health_monitoring_task_stack_buffer,
       .task_block           = health_monitoring_task_tcb};

   health_monitoring_task_handle = rtos::create_task(health_monitoring_task_config);

   // barometer task
   std::memset(barometer_task_stack_buffer, 0, sizeof(barometer_task_stack_buffer));

   rtos::RtosTaskConfig barometer_task_config{
       .func                 = barometer_task,
       .name                 = controller::task::barometer_task_name,
       .stack_depth_in_words = controller::task::barometer_task_stack_depth_in_words,
       .params               = static_cast<void*>(&barometer_task_data),
       .priority             = controller::task::barometer_task_priority,
       .stack_buffer         = barometer_task_stack_buffer,
       .task_block           = barometer_task_tcb};

   barometer_task_handle = rtos::create_task(barometer_task_config);

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

   logging_task_handle = rtos::create_task(logging_task_config);
}

void setup_semaphores()
{
   // logging uart semaphore
   auto logging_uart_sem_handle = logging_uart_semaphore.create();
   logging_task_data.logging_uart.transmitter_sem_taker.set_handle(logging_uart_sem_handle);
   logging_task_data.logging_uart.isr_sem_giver.set_handle(logging_uart_sem_handle);

   // dummy radio link semaphore
   auto radio_input_sem_handle = radio_input_semaphore.create();
   radio_link_task_data.radio_link_uart.transmitter_sem_taker.set_handle(radio_input_sem_handle);
   radio_link_task_data.radio_link_uart.isr_sem_giver.set_handle(radio_input_sem_handle);
}

void setup_queues()
{
   // logging queue
   auto logging_queue_handle = logging_queue.create();
   ::logging::logging_queue_sender.set_handle(logging_queue_handle);
   logging_task_data.logging_queue_receiver.set_handle(logging_queue_handle);

   // health summary queue
   auto health_summary_queue_handle = health_summary_queue.create();
   health_monitoring_task_data.health_summary_queue_sender.set_handle(health_summary_queue_handle);
   flight_manager_task_data.health_summary_queue_receiver.set_handle(health_summary_queue_handle);

   // radio link task queues
   auto radio_input_queue_handle = radio_input_queue.create();
   radio_link_task_data.radio_link_uart.radio_input_sender_from_isr.set_handle(radio_input_queue_handle);
   radio_link_task_data.radio_link_uart.radio_input_receiver.set_handle(radio_input_queue_handle);

   auto radio_task_buffer_index_queue_handle = radio_task_buffer_index_queue.create();
   radio_link_task_data.radio_link_uart.radio_queue_buffer_index_sender.set_handle(radio_task_buffer_index_queue_handle);
   radio_link_task_data.radio_link_uart.radio_queue_buffer_index_receiver_from_isr.set_handle(radio_task_buffer_index_queue_handle);

   // make all buffers available by pushing all buffer indices to queue
   for (std::size_t i = 0; i < radio_link_task_data.queue_depth; i++)
   {
      radio_link_task_data.radio_link_uart.radio_queue_buffer_index_sender.send_blocking(i);
   }
}

void setup_task_notifications()
{
   // imu task
   imu_task_data.imu_task_tick_notifier_from_isr.set_task_to_notify(imu_task_handle);
   imu_task_data.imu_task_rx_complete_notifier_from_isr.set_task_to_notify(imu_task_handle);

   // barometer task
   barometer_task_data.barometer_task_rx_complete_notifier_from_isr.set_task_to_notify(barometer_task_handle);

   // control task
   flight_manager_task_data.control_task_start_notifier.set_task_to_notify(control_task_handle);
}

void init_hardware()
{
   imu_task_data.spi1_chip_select.disable();
}

void setup_uart()
{
   // USART1, logging uart (No need to call start_rx() for logging uart)
   hw::uart::prepare_for_communication(logging_task_data.logging_uart.config,
                                       std::as_bytes(std::span{logging_task_data.logging_uart.dma_tx_buffer}),
                                       std::as_bytes(std::span{logging_task_data.logging_uart.dummy_dma_rx_buffer}));

   // USART2
   hw::uart::prepare_for_communication(radio_link_task_data.radio_link_uart.config,
                                       radio_link_task_data.radio_link_uart.dma_tx_buffer,
                                       radio_link_task_data.radio_link_uart.dma_rx_buffer);
   hw::uart::start_rx(radio_link_task_data.radio_link_uart.config, radio_link_task_data.radio_link_uart.dma_rx_buffer);
}

void setup_spi()
{
   imu_task_data.spi1_master.prepare_for_communication();
}

void setup_i2c()
{
   barometer_task_data.i2c_driver.prepare_for_communication();
}

void start_sys_clock()
{
   sys_time::ClockSource::init();
   sys_clock_data.syc_clock_timer.start();
}

}   // namespace controller

extern "C"
{
   void controller_initialize_hardware_and_start_scheduler()
   {
      if (error::has_no_error())
      {
         // rtos objects
         controller::setup_semaphores();
         controller::setup_queues();

         // hw objects
         controller::init_hardware();
         controller::setup_uart();
         controller::setup_spi();
         controller::setup_i2c();

         // start sys clock
         controller::start_sys_clock();

         // tasks
         controller::register_tasks();

         // task notifications
         controller::setup_task_notifications();

         // start scheduler
         rtos::start_scheduler();
      }
      else
      {
         __asm volatile("BKPT #0");
      }
   }
}   // extern "C"
