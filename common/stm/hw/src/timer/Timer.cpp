#include "hw/timer/Timer.hpp"

namespace hw::timer
{

void Timer::start()
{
   LL_TIM_EnableIT_UPDATE(m_config.timer_handle);
   LL_TIM_EnableCounter(m_config.timer_handle);
}

void Timer::clear_update_flag()
{
   LL_TIM_ClearFlag_UPDATE(m_config.timer_handle);
}

bool Timer::is_update_flag_active() const
{
   return (LL_TIM_IsActiveFlag_UPDATE(m_config.timer_handle) != 0);
}

}   // namespace hw::timer
