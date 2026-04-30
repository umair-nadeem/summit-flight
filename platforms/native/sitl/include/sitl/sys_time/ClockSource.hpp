#pragma once

#include "interfaces/IClockSource.hpp"

namespace sitl::sys_time
{

struct ClockSource
{
   static inline uint64_t s_now_us = 0;

   static void init() noexcept
   {
      s_now_us = 0;
   }

   static void set_us(const uint64_t us) noexcept
   {
      s_now_us = us;
   }

   static void advance_us(const uint64_t dt_us) noexcept
   {
      s_now_us += dt_us;
   }

   static uint32_t now_us() noexcept
   {
      return static_cast<uint32_t>(s_now_us);
   }

   static uint32_t now_ms() noexcept
   {
      return static_cast<uint32_t>(s_now_us / 1000ull);
   }

   static uint32_t now_s() noexcept
   {
      return static_cast<uint32_t>(s_now_us / 1000000ull);
   }
};

static_assert(interfaces::IClockSource<ClockSource>);

}   // namespace sitl::sys_time
