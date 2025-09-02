#pragma once

#include <cstdint>

#include "stm32f4xx.h"

namespace hw::timer
{

class CpuCycleTimer
{
public:
   explicit CpuCycleTimer(const uint32_t system_core_clock_hz);

   void set_timeout_us(const uint32_t delay_us);
   void start_measurement();

   bool timeout_occurred() const;

private:
   static constexpr uint32_t cycles_per_mhz = 1'000'000u;

   const uint32_t m_system_core_clock_hz;
   const uint32_t m_cycles_per_us;
   uint32_t       m_delay_cycle_count{0};
   uint32_t       m_start_cycle_count{0};
};

}   // namespace hw::timer
