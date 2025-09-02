#pragma once

#include "hardware_bindings.hpp"
#include "hw/i2c/I2c.hpp"
#include "hw/timer/CpuCycleTimer.hpp"
#include "task_params.hpp"

extern "C"
{
   [[noreturn]] void barometer_task(void* params);
}

namespace controller
{

struct BarometerTaskData
{
   // i2c timeout evaluator
   hw::timer::CpuCycleTimer i2c_timeout_evaluator{task::system_core_clock};

   // I2C1
   hw::i2c::I2c<decltype(i2c_timeout_evaluator)> i2c_driver{global_data.i2c.i2c1_config, i2c_timeout_evaluator};
};

extern BarometerTaskData barometer_task_data;

}   // namespace controller
