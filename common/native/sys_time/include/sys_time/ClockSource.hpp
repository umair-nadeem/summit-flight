#pragma once

#include "interfaces/IClockSource.hpp"

namespace sys_time
{

struct ClockSource
{
   static void init() noexcept
   {
      // No init needed for std::chrono
   }

   static uint32_t now_us() noexcept
   {
      using namespace std::chrono;
      return static_cast<uint32_t>(duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
   }

   static uint32_t now_ms() noexcept
   {
      using namespace std::chrono;
      return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
   }

   static uint32_t now_s() noexcept
   {
      using namespace std::chrono;
      return static_cast<uint32_t>(duration_cast<seconds>(steady_clock::now().time_since_epoch()).count());
   }
};

static_assert(interfaces::IClockSource<ClockSource>);

}   // namespace sys_time
