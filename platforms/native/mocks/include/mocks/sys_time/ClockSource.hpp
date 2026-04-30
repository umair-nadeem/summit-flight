#pragma once

#include "interfaces/IClockSource.hpp"

namespace mocks::sys_time
{

struct ClockSource
{
   static inline uint32_t u_sec = 0;
   static inline uint32_t m_sec = 0;
   static inline uint32_t sec   = 0;

   static uint32_t now_us()
   {
      return u_sec;
   }

   static uint32_t now_ms()
   {
      return m_sec;
   }

   static uint32_t now_s()
   {
      return sec;
   }

   static void reset()
   {
      u_sec = 0;
      m_sec = 0;
      sec   = 0;
   }
};

static_assert(interfaces::IClockSource<ClockSource>);

}   // namespace mocks::sys_time
