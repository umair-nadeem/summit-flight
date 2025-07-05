#include "SensorAcquisitionTaskData.h"
#include "aeromight_sensors/SensorAcquisition.hpp"
#include "rtos/periodic_task.hpp"

extern "C"
{

   void sensor_acquisition_task(void* params)
   {
      assert(params != nullptr);

      auto* data = static_cast<controller::SensorAcquisitionTaskData*>(params);

      aeromight_sensors::SensorAcquisition<decltype(data->x)>
          sensor_acquisition{data->x,
                             controller::sensor_acq_task_period_in_ms};

      rtos::run_periodic_task(sensor_acquisition);
   }
}
