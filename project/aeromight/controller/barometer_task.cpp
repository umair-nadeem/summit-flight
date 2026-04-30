#include "BarometerTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_params.hpp"
#include "bmp390/Bmp390.hpp"
#include "event_handling/EventBinding.hpp"
#include "event_handling/EventDispatcher.hpp"
#include "logging/LogClient.hpp"
#include "rtos/QueueSender.hpp"
#include "rtos/periodic_task.hpp"
#include "sys_time/ClockSource.hpp"

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

      constexpr uint32_t notification_wait_period_in_ms = barometer_task_period_in_ms / 4u;

      LogClient logger_bmp390{logging::logging_queue_sender, "bmp390"};

      bmp390::Bmp390Params bmp390_params{};
      bmp390::Bmp390<decltype(data->i2c_driver),
                     LogClient>
          bmp390{data->i2c_driver,
                 logger_bmp390,
                 data->baro_sensor_data,
                 data->baro_sensor_status,
                 bmp390_params};

      using TickBinding       = event_handling::EventBinding<decltype(bmp390), &decltype(bmp390)::execute>;
      using RxCompleteBinding = event_handling::EventBinding<decltype(bmp390), &decltype(bmp390)::notify_receive_complete>;

      event_handling::EventDispatcher barometer_event_dispatcher{data->barometer_task_notification_waiter,
                                                                 notification_wait_period_in_ms,
                                                                 TickBinding{bmp390, data->event_tick_bit_mask},
                                                                 RxCompleteBinding{bmp390, data->event_rx_complete_bit_mask}};

      data->i2c_driver.register_receive_complete_callback(&i2c1_receive_complete_callback, nullptr);
      bmp390.start();

      while (true)
      {
         barometer_event_dispatcher.execute();
      }
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
