#include "ImuTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "error/error_handler.hpp"
#include "event_handling/EventBinding.hpp"
#include "event_handling/EventDispatcher.hpp"
#include "hw/uart/uart.hpp"
#include "imu/Imu.hpp"
#include "logging/LogClient.hpp"
#include "mpu6500/Mpu6500.hpp"
#include "rtos/QueueSender.hpp"
#include "sys_time/ClockSource.hpp"
#include "task_params.hpp"

namespace logging
{

extern rtos::QueueSender<params::LogBuffer> logging_queue_sender;

}   // namespace logging

extern "C"
{

   [[noreturn]] void imu_task(void* const params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::ImuTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      LogClient logger_mpu6500{logging::logging_queue_sender, "mpu6500"};
      LogClient logger_imu{logging::logging_queue_sender, "imu"};

      mpu6500::Mpu6500Params mpu6500_params{};
      mpu6500::Mpu6500<decltype(data->spi1_master),
                       LogClient>
          mpu6500{data->spi1_master,
                  logger_mpu6500,
                  data->imu_sensor_data,
                  data->imu_sensor_status,
                  mpu6500_params};

      imu::ImuParams                             imu_params{};
      imu::Imu<sys_time::ClockSource, LogClient> imu{aeromight_boundaries::aeromight_data.imu_data,
                                                     aeromight_boundaries::aeromight_data.imu_health,
                                                     logger_imu,
                                                     imu_params};

      using TickBinding        = event_handling::EventBinding<decltype(mpu6500), &decltype(mpu6500)::execute>;
      using RxCompleteBinding  = event_handling::EventBinding<decltype(mpu6500), &decltype(mpu6500)::notify_receive_complete>;
      using CalibrationBinding = event_handling::EventBinding<decltype(imu), &decltype(imu)::start_calibration>;

      event_handling::EventDispatcher imu_event_dispatcher{data->imu_task_notification_waiter,
                                                           controller::task::imu_task_period_in_ms,
                                                           TickBinding{mpu6500, data->event_tick_bit_mask},
                                                           RxCompleteBinding{mpu6500, data->event_rx_complete_bit_mask},
                                                           CalibrationBinding{imu, data->event_calibrate_bit_mask}};

      mpu6500.start();
      data->imu_task_tick_timer.enable_interrupt();
      data->imu_task_tick_timer.enable_counter();

      while (true)
      {
         imu_event_dispatcher.execute();
         imu.execute(data->imu_sensor_data, data->imu_sensor_status);
      }
   }
}

extern "C"
{
   // SPI1 RX DMA
   void DMA2_Stream0_IRQHandler()
   {
      auto& data = controller::imu_task_data;
      data.spi1_master.handle_spi_transfer_complete_interrupt();
      data.imu_task_rx_complete_notifier_from_isr.notify_from_isr();
   }

   // TIM3 (imu task tick timer)
   void TIM3_IRQHandler(void)
   {
      auto& data = controller::imu_task_data;
      if (data.imu_task_tick_timer.is_update_flag_active())
      {
         data.imu_task_tick_timer.clear_update_flag();
         data.imu_task_tick_notifier_from_isr.notify_from_isr();
      }
   }
}   // extern "C"
