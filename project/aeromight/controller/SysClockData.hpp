#pragma once

#include "hardware_bindings.hpp"
#include "hw/timer/Timer.hpp"

namespace controller
{

struct SysClockData
{
   hw::timer::Timer syc_clock_timer{global_data.timer.tim2_config};
};

extern SysClockData sys_clock_data;

}   // namespace controller
