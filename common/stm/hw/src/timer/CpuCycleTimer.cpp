#include "hw/timer/CpuCycleTimer.hpp"

#include "error/error_handler.hpp"

namespace hw::timer
{

CpuCycleTimer::CpuCycleTimer(const uint32_t system_core_clock_hz)
    : m_system_core_clock_hz{system_core_clock_hz},
      m_cycles_per_us{m_system_core_clock_hz / cycles_per_mhz}
{
   error::verify(m_cycles_per_us > 0);
}

void CpuCycleTimer::set_timeout_us(const uint32_t delay_us)
{
   error::verify(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);
   m_delay_cycle_count = delay_us * m_cycles_per_us;
}

void CpuCycleTimer::start_measurement()
{
   m_start_cycle_count = DWT->CYCCNT;
}

bool CpuCycleTimer::timeout_occurred() const
{
   const uint32_t elapsed_cycles = static_cast<uint32_t>(DWT->CYCCNT - m_start_cycle_count);
   return (elapsed_cycles > m_delay_cycle_count);
}

}   // namespace hw::timer
