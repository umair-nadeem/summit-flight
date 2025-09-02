#pragma once

#include "hardware_bindings.hpp"
#include "hw/i2c/I2c.hpp"
#include "task_params.hpp"

extern "C"
{
   [[noreturn]] void barometer_task(void* params);
}

namespace controller
{

struct BarometerTaskData
{
   // I2C1
   hw::i2c::I2c i2c_driver{global_data.i2c.i2c1_config};
};

extern BarometerTaskData barometer_task_data;

}   // namespace controller
