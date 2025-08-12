#include "SensorAcquisitionTaskData.hpp"
#include "aeromight_boundaries/AeromightSensorData.hpp"
#include "aeromight_sensors/SensorAcquisition.hpp"
#include "error/error_handler.hpp"
#include "hw/uart/uart.hpp"
#include "logging/LogClient.hpp"
#include "mpu6500/Mpu6500.hpp"
#include "rtos/QueueSender.hpp"
#include "rtos/periodic_task.hpp"
#include "task_params.hpp"

namespace logging
{

extern rtos::QueueSender<params::LogBuffer> logging_queue_sender;

}   // namespace logging

extern "C"
{

   [[noreturn]] void sensor_acquisition_task(void* params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::SensorAcquisitionTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      LogClient logger_sensor_acq{logging::logging_queue_sender, "snsr_acq"};

      mpu6500::Mpu6500<sys_time::ClockSource,
                       decltype(data->spi1_master)>
          mpu6500{aeromight_boundaries::aeromight_sensor_data.imu_sensor_data_storage,
                  data->spi1_master};

      aeromight_sensors::SensorAcquisition<
          decltype(mpu6500),
          decltype(data->blue_led),
          LogClient>
          sensor_acquisition{mpu6500,
                             data->blue_led,
                             logger_sensor_acq,
                             controller::task::sensor_acq_task_period_in_ms};

      rtos::run_periodic_task(sensor_acquisition);
   }
}

extern "C"
{
   void DMA2_Stream0_IRQHandler()
   {
      auto& data = controller::sensor_acq_task_data;
      data.spi1_master.handle_spi_transfer_complete_interrupt();
   }
}
