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

   static uint32_t m_sec;
   static uint32_t sec;
};

uint32_t ClockSource::m_sec = 0;
uint32_t ClockSource::sec   = 0;

static_assert(interfaces::IClockSource<ClockSource>);

}   // namespace mocks::common
