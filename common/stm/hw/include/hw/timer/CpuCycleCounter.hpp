#pragma once

#include <cstdint>

#include "stm32f4xx.h"

namespace hw::timer
{

class CpuCycleCounter
{
public:
   explicit CpuCycleCounter(const uint32_t system_core_clock)
       : m_system_core_clock{system_core_clock},
         m_cycles_per_us{m_system_core_clock / 1'000'000}
   {
   }

   void set_timeout(const uint32_t delay_us)
   {
      m_delay_cycle_count = delay_us * m_cycles_per_us;
   }

   void start_measurement()
   {
      m_start_cycle_count = DWT->CYCCNT;
   }

   bool timeout_occurred() const
   {
      return (static_cast<uint32_t>(DWT->CYCCNT - m_start_cycle_count) > m_delay_cycle_count);
   }

private:
   const uint32_t m_system_core_clock;
   const uint32_t m_cycles_per_us;
   uint32_t       m_delay_cycle_count{0};
   uint32_t       m_start_cycle_count{0};
};

}   // namespace hw::timer
