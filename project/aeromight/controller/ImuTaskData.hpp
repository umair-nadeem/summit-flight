#pragma once

#include <array>

#include "hardware_bindings.hpp"
#include "hw/gpio/DigitalOutput.hpp"
#include "hw/pcb_component/Enabler.hpp"
#include "hw/pcb_component/Led.hpp"
#include "hw/spi/SpiMasterWithDma.hpp"

extern "C"
{
   [[noreturn]] void imu_task(void* params);
}

namespace controller
{

struct ImuTaskData
{
   hw::pcb_component::Led blue_led{global_data.gpios.blue_led, true};

   // SPI 1
   hw::gpio::DigitalOutput                                     spi1_chip_select_gpio{global_data.spi.spi1_chip_select, true};
   hw::pcb_component::Enabler<decltype(spi1_chip_select_gpio)> spi1_chip_select{spi1_chip_select_gpio};

   hw::spi::SpiMasterWithDma<decltype(spi1_chip_select)> spi1_master{global_data.spi.spi1_config, spi1_chip_select};
};

extern ImuTaskData imu_task_data;

}   // namespace controller
