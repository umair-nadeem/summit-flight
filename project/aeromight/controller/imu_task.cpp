#include "ImuTaskData.hpp"
#include "aeromight_boundaries/AeromightSensorData.hpp"
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

   [[noreturn]] void imu_task(void* params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::ImuTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      constexpr std::size_t task_execution_period_ms                     = controller::task::imu_task_period_in_ms;
      constexpr std::size_t mpu6500_receive_wait_timeout_ms              = 2u * task_execution_period_ms;
      constexpr uint8_t     mpu6500_read_failures_limit                  = 5;
      constexpr uint8_t     mpu6500_sample_rate_divider                  = 0x03;
      constexpr uint8_t     mpu6500_dlpf_config                          = 0x02;
      constexpr uint8_t     mpu6500_gyro_full_scale                      = 0x03;
      constexpr uint8_t     mpu6500_accel_full_scale                     = 0x03;
      constexpr uint8_t     mpu6500_accel_a_dlpf_config                  = 0x03;
      constexpr float       mpu6500_gyro_range_plausibility_margin_radps = 6.0f;
      constexpr float       mpu6500_accel_range_plausibility_margin_mps2 = 20.0f;

      LogClient logger_imu_driver_executor{logging::logging_queue_sender, "imu_task"};

      mpu6500::Mpu6500<sys_time::ClockSource,
                       decltype(data->spi1_master)>
          mpu6500{aeromight_boundaries::aeromight_sensor_data.imu_sensor_data_storage,
                  aeromight_boundaries::aeromight_sensor_data.imu_sensor_health_storage,
                  data->spi1_master,
                  mpu6500_read_failures_limit,
                  task_execution_period_ms,
                  mpu6500_receive_wait_timeout_ms,
                  mpu6500_sample_rate_divider,
                  mpu6500_dlpf_config,
                  mpu6500_gyro_full_scale,
                  mpu6500_accel_full_scale,
                  mpu6500_accel_a_dlpf_config,
                  mpu6500_gyro_range_plausibility_margin_radps,
                  mpu6500_accel_range_plausibility_margin_mps2};

      aeromight_imu::ImuDriverExecutor<
          decltype(mpu6500),
          decltype(data->blue_led),
          LogClient>
          imu_driver_executor{mpu6500,
                              data->blue_led,
                              logger_imu_driver_executor,
                              controller::task::imu_task_period_in_ms};

      rtos::run_periodic_task(imu_driver_executor);
   }
}

extern "C"
{
   void DMA2_Stream0_IRQHandler()
   {
      auto& data = controller::imu_task_data;
      data.spi1_master.handle_spi_transfer_complete_interrupt();
   }
}
