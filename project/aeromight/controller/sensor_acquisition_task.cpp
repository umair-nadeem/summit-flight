#include "SensorAcquisitionTaskData.h"
#include "aeromight_sensors/SensorAcquisition.hpp"
#include "rtos/periodic_task.hpp"

extern "C"
{

   [[noreturn]] void sensor_acquisition_task(void* params)
   {
      assert(params != nullptr);

      auto* data = static_cast<controller::SensorAcquisitionTaskData*>(params);

      aeromight_sensors::SensorAcquisition<decltype(data->blue_led)> sensor_acquisition{data->blue_led,
                                                                                        controller::task::sensor_acq_task_period_in_ms};

      rtos::run_periodic_task(sensor_acquisition);
   }
}

extern "C"
{
   void USART1_IRQHandler()
   {
   }

   void DMA2_Stream7_IRQHandler()
   {
   }
}
