#pragma once

#include <array>

#include "hw/gpio/DigitalOutput.hpp"

extern "C"
{
   [[noreturn]] void sensor_acquisition_task(void* params);
}

namespace controller
{

struct SensorAcquisitionTaskData
{
   hw::gpio::DigitalOutput blue_led{GPIOC, LL_GPIO_PIN_13, true};
};

extern SensorAcquisitionTaskData sensor_acq_task_data;

}   // namespace controller
