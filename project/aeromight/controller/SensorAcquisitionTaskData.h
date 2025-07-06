#pragma once

#include "hw/gpio/DigitalOutput.hpp"
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
   hw::gpio::DigitalOutput blue_led{GPIOC, GPIO_PIN_13, true};
};

}   // namespace controller
