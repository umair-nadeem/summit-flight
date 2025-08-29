#include "BarometerTaskData.hpp"
#include "aeromight_barometer/BarometerDriverExecutor.hpp"
#include "aeromight_boundaries/AeromightSensorData.hpp"
#include "bmp390/Bmp390.hpp"
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

   [[noreturn]] void barometer_task(void* params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::BarometerTaskData*>(params);

      using LogClient = logging::LogClient<decltype(logging::logging_queue_sender)>;

      constexpr uint8_t mpu6500_read_failures_limit = 5;

      LogClient logger_bmp390{logging::logging_queue_sender, "bmp390"};

      bmp390::Bmp390<sys_time::ClockSource,
                     decltype(data->spi2_master),
                     LogClient>
          bmp390{aeromight_boundaries::aeromight_sensor_data.barometer_sensor_data_storage,
                 aeromight_boundaries::aeromight_sensor_data.barometer_sensor_health_storage,
                 data->spi2_master,
                 logger_bmp390,
                 mpu6500_read_failures_limit,
                 controller::task::barometer_task_period_in_ms};

      aeromight_barometer::BarometerDriverExecutor<decltype(bmp390)> barometer_driver_executor{bmp390,
                                                                                               controller::task::barometer_task_period_in_ms};

      barometer_driver_executor.start();
      rtos::run_periodic_task(barometer_driver_executor);
   }
}
