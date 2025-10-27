#include "BarometerTaskData.hpp"
#include "aeromight_barometer/BarometerDriverExecutor.hpp"
#include "aeromight_boundaries/AeromightSensorData.hpp"
#include "bmp390/Bmp390.hpp"
#include "logging/LogClient.hpp"
#include "rtos/QueueSender.hpp"
#include "rtos/periodic_task.hpp"
#include "sys_time/ClockSource.hpp"
#include "task_params.hpp"

namespace logging
{

extern rtos::QueueSender<params::LogBuffer> logging_queue_sender;

}   // namespace logging

extern "C"
{

   [[noreturn]] void barometer_task(void* const params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::BarometerTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      constexpr uint32_t bmp390_receive_wait_timeout_ms = 2u * controller::task::barometer_task_period_in_ms;
      constexpr uint32_t notification_wait_period_in_ms = controller::task::barometer_task_period_in_ms / 4u;
      constexpr uint8_t  bmp390_read_failures_limit     = 5u;
      constexpr uint8_t  bmp390_max_recovery_attempts   = 3u;

      LogClient logger_bmp390{logging::logging_queue_sender, "bmp390"};

      bmp390::Bmp390<sys_time::ClockSource,
                     decltype(data->i2c_driver),
                     LogClient>
          bmp390{aeromight_boundaries::aeromight_sensor_data.barometer_sensor_data_storage,
                 aeromight_boundaries::aeromight_sensor_data.barometer_sensor_health_storage,
                 data->i2c_driver,
                 logger_bmp390,
                 bmp390_read_failures_limit,
                 bmp390_max_recovery_attempts,
                 controller::task::barometer_task_period_in_ms,
                 bmp390_receive_wait_timeout_ms};

      aeromight_barometer::BarometerDriverExecutor<decltype(bmp390),
                                                   decltype(data->barometer_task_notification_waiter)>
          barometer_driver_executor{bmp390,
                                    data->barometer_task_notification_waiter,
                                    controller::task::barometer_task_period_in_ms,
                                    notification_wait_period_in_ms};

      data->i2c_driver.register_receive_complete_callback(&i2c1_receive_complete_callback, nullptr);
      barometer_driver_executor.start();
      rtos::run_periodic_task(barometer_driver_executor);
   }
}

// I2C1 rx complete (called from ISR)
void i2c1_receive_complete_callback([[maybe_unused]] void* p)
{
   auto& data = controller::barometer_task_data;
   data.barometer_task_rx_complete_notifier_from_isr.notify_from_isr();
}

extern "C"
{
   // I2C1 Event
   void I2C1_EV_IRQHandler(void)
   {
      auto& data = controller::barometer_task_data;
      data.i2c_driver.handle_i2c_event_interrupt();
   }

   // I2C1 Error
   void I2C1_ER_IRQHandler(void)
   {
      auto& data = controller::barometer_task_data;
      data.i2c_driver.handle_i2c_error_interrupt();
   }

}   // extern "C"
