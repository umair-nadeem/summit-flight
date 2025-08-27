#include "SysClockData.hpp"
#include "sys_time/ClockSource.hpp"

std::atomic<uint32_t> sys_time::ClockSource::clock_ms{0};

extern "C"
{
   void TIM2_IRQHandler(void)
   {
      auto& data = controller::sys_clock_data;
      if (data.syc_clock_timer.is_update_flag_active())
      {
         data.syc_clock_timer.clear_update_flag();
         sys_time::ClockSource::tick_clock();   // increment 1 ms
      }
   }

}   // extern "C"
