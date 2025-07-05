#pragma once

#include "rtos/RtosTaskConfig.h"
#include "task_params.h"

extern "C"
{
   void sensor_acquisition_task(void* params);
}

namespace controller
{

struct SensorAcquisitionTaskData
{
   int x = 1;   // dummy data
};

}   // namespace controller
