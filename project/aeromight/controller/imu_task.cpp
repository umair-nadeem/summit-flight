#include "ImuTaskData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "aeromight_imu/ImuDriverExecutor.hpp"
#include "error/error_handler.hpp"
#include "hw/uart/uart.hpp"
#include "logging/LogClient.hpp"
#include "mpu6500/Mpu6500.hpp"
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

   [[noreturn]] void imu_task(void* const params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::ImuTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      constexpr uint32_t mpu6500_receive_wait_timeout_ms              = 2u * controller::task::imu_task_period_in_ms;
      constexpr uint8_t  mpu6500_read_failures_limit                  = 5u;
      constexpr uint8_t  mpu6500_sample_rate_divider                  = 0x03;
      constexpr uint8_t  mpu6500_dlpf_config                          = 0x03;
      constexpr uint8_t  mpu6500_gyro_full_scale                      = 0x03;
      constexpr uint8_t  mpu6500_accel_full_scale                     = 0x03;
      constexpr uint8_t  mpu6500_accel_a_dlpf_config                  = 0x03;
      constexpr float    mpu6500_gyro_range_plausibility_margin_radps = 6.0f;
      constexpr float    mpu6500_accel_range_plausibility_margin_mps2 = 20.0f;
      constexpr uint16_t num_samples_self_test                        = 400u;
      constexpr float    gyro_tolerance_radps                         = 0.1f;
      constexpr float    accel_tolerance_mps2                         = 1.5f;

      LogClient logger_imu_driver_executor{logging::logging_queue_sender, "imu_runner"};
      LogClient logger_mpu6500{logging::logging_queue_sender, "mpu6500"};

      mpu6500::Mpu6500<sys_time::ClockSource,
                       decltype(data->spi1_master),
                       decltype(logger_mpu6500)>
          mpu6500{aeromight_boundaries::aeromight_data.imu_sensor_data_storage,
                  aeromight_boundaries::aeromight_data.imu_sensor_health_storage,
                  data->spi1_master,
                  logger_mpu6500,
                  mpu6500_read_failures_limit,
                  controller::task::imu_task_period_in_ms,
                  mpu6500_receive_wait_timeout_ms,
                  mpu6500_sample_rate_divider,
                  mpu6500_dlpf_config,
                  mpu6500_gyro_full_scale,
                  mpu6500_accel_full_scale,
                  mpu6500_accel_a_dlpf_config,
                  mpu6500_gyro_range_plausibility_margin_radps,
                  mpu6500_accel_range_plausibility_margin_mps2,
                  num_samples_self_test,
                  gyro_tolerance_radps,
                  accel_tolerance_mps2};

      aeromight_imu::ImuDriverExecutor<
          decltype(mpu6500),
          decltype(data->imu_task_notification_waiter),
          decltype(data->blue_led),
          LogClient>
          imu_driver_executor{mpu6500,
                              data->imu_task_notification_waiter,
                              data->blue_led,
                              logger_imu_driver_executor,
                              controller::task::imu_task_period_in_ms};

      imu_driver_executor.start();
      data->imu_task_tick_timer.start();

      while (true)
      {
         imu_driver_executor.run_once();
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
