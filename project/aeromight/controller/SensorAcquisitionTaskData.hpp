#pragma once

#include <array>

#include "hardware_bindings.hpp"
#include "hw/gpio/DigitalOutput.hpp"

extern "C"
{
   [[noreturn]] void sensor_acquisition_task(void* params);
}

namespace controller
{

struct SensorAcquisitionTaskData
{
   hw::gpio::DigitalOutput blue_led{global_data.gpios.blue_led, true};

   hw::gpio::DigitalOutput chip_select_spi1{global_data.spi.chip_select_spi1, true};
};

extern SensorAcquisitionTaskData sensor_acq_task_data;

}   // namespace controller
