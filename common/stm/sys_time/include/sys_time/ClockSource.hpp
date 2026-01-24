#pragma once

#include "hw/timer/TimerConfig.hpp"
#include "interfaces/IClockSource.hpp"

namespace sys_time
{

struct ClockSource
{
   static uint32_t now_us() noexcept
   {
      return timer_config.timer_handle->CNT;
   }

   static uint32_t now_ms() noexcept
   {
      return static_cast<uint32_t>(static_cast<float>(now_us()) * 0.001f);
   }

   static uint32_t now_s() noexcept
   {
      return static_cast<uint32_t>(static_cast<float>(now_us()) * 0.000001f);
   }

   static hw::timer::TimerConfig timer_config;
};

static_assert(interfaces::IClockSource<ClockSource>);

}   // namespace sys_time
