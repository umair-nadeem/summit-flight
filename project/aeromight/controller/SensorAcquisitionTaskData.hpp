#pragma once

#include <array>

#include "hardware_bindings.hpp"
#include "hw/gpio/DigitalOutput.hpp"
#include "hw/spi/SpiMasterWithDma.hpp"

extern "C"
{
   [[noreturn]] void sensor_acquisition_task(void* params);
}

namespace controller
{

struct SensorAcquisitionTaskData
{
   hw::gpio::DigitalOutput blue_led{global_data.gpios.blue_led, true};

   // SPI 1
   hw::gpio::DigitalOutput spi1_chip_select{global_data.spi.spi1_chip_select, true};

   hw::spi::SpiMasterWithDma<decltype(spi1_chip_select)> spi1_master{global_data.spi.spi1_config, spi1_chip_select};
};

extern SensorAcquisitionTaskData sensor_acq_task_data;

}   // namespace controller
