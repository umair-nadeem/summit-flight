#pragma once

#include "interfaces/IClockSource.hpp"

namespace sys_time
{

struct ClockSource
{
   static uint32_t now_ms() noexcept
   {
      return 0;
      // return static_cast<uint64_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
   }

   static uint32_t now_us() noexcept
   {
      return 0;
      // Convert FreeRTOS ticks to microseconds
      // return static_cast<uint64_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS * 1000ULL;
   }

   static uint32_t now_s() noexcept
   {
      return 0;
      // return static_cast<uint64_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS / 1000ULL;
   }
};

static_assert(interfaces::IClockSource<ClockSource>);

}   // namespace sys_time
