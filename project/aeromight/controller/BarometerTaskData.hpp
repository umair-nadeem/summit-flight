#pragma once

#include "hardware_bindings.hpp"
#include "hw/gpio/DigitalOutput.hpp"
#include "hw/pcb_component/Enabler.hpp"
#include "hw/spi/SpiMaster.hpp"
#include "hw/timer/CpuCycleCounter.hpp"
#include "task_params.hpp"

extern "C"
{
   [[noreturn]] void barometer_task(void* params);
}

namespace controller
{

struct BarometerTaskData
{
   // spi timeout evaluator
   hw::timer::CpuCycleCounter spi_timeout_evaluator{task::system_core_clock};

   // SPI2
   hw::gpio::DigitalOutput                                                         spi2_chip_select_gpio{global_data.spi.spi2_chip_select, true};
   hw::pcb_component::Enabler<decltype(spi2_chip_select_gpio)>                     spi2_chip_select{spi2_chip_select_gpio};
   hw::spi::SpiMaster<decltype(spi2_chip_select), decltype(spi_timeout_evaluator)> spi2_master{global_data.spi.spi2_config,
                                                                                               spi2_chip_select,
                                                                                               spi_timeout_evaluator};
};

extern BarometerTaskData barometer_task_data;

}   // namespace controller
