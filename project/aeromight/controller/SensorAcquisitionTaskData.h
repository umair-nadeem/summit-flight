#pragma once

#include "hw_abstraction/DigitalOutput.hpp"
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
   hw_abstraction::DigitalOutput blue_led{GPIOC, GPIO_PIN_13};
};

}   // namespace controller
