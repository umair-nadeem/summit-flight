#pragma once

#include <gmock/gmock.h>

#include "interfaces/IClockSource.hpp"

namespace mocks::common
{

class ClockSource
{
public:
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
      m_sec = 0;
      sec   = 0;
   }

   inline static uint32_t m_sec = 0;
   inline static uint32_t sec   = 0;
};

static_assert(interfaces::IClockSource<ClockSource>);

}   // namespace mocks::common
