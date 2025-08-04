#include "SensorAcquisitionTaskData.h"
#include "aeromight_sensors/SensorAcquisition.hpp"
#include "error/error_handler.h"
#include "hw/uart/uart.hpp"
#include "logging/LogClient.hpp"
#include "rtos/QueueSender.hpp"
#include "rtos/periodic_task.hpp"
#include "task_params.h"

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

      aeromight_sensors::SensorAcquisition<decltype(data->blue_led), LogClient> sensor_acquisition{data->blue_led,
                                                                                                   logger_sensor_acq,
                                                                                                   controller::task::sensor_acq_task_period_in_ms};

      rtos::run_periodic_task(sensor_acquisition);
   }
}
