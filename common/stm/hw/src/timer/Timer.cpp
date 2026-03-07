#include "hw/timer/Timer.hpp"

namespace
{

uint32_t get_timer_channel(const hw::timer::OutputChannel channel)
{
   switch (channel)
   {
      case hw::timer::OutputChannel::channel1:
         return LL_TIM_CHANNEL_CH1;
         break;

      case hw::timer::OutputChannel::channel2:
         return LL_TIM_CHANNEL_CH2;
         break;

      case hw::timer::OutputChannel::channel3:
         return LL_TIM_CHANNEL_CH3;
         break;

      case hw::timer::OutputChannel::channel4:
         return LL_TIM_CHANNEL_CH4;
         break;

      default:
         error::stop_operation();
         break;
   }

   return 0;
}

}   // namespace

namespace hw::timer
{

void Timer::enable_interrupt()
{
   LL_TIM_EnableIT_UPDATE(m_config.timer_handle);
}

void Timer::clear_update_flag()
{
   LL_TIM_ClearFlag_UPDATE(m_config.timer_handle);
}

bool Timer::is_update_flag_active() const
{
   return (LL_TIM_IsActiveFlag_UPDATE(m_config.timer_handle) != 0);
}

void Timer::set_compare_value_for_channel(const OutputChannel channel, const uint32_t value)
{
   switch (channel)
   {
      case OutputChannel::channel1:
         LL_TIM_OC_SetCompareCH1(m_config.timer_handle, value);
         break;

      case OutputChannel::channel2:
         LL_TIM_OC_SetCompareCH2(m_config.timer_handle, value);
         break;

      case OutputChannel::channel3:
         LL_TIM_OC_SetCompareCH3(m_config.timer_handle, value);
         break;

      case OutputChannel::channel4:
         LL_TIM_OC_SetCompareCH4(m_config.timer_handle, value);
         break;

      default:
         error::stop_operation();
         break;
   }
}

void Timer::enable_counter()
{
   LL_TIM_EnableCounter(m_config.timer_handle);
}

void Timer::enable_channel(const ChannelType channel)
{
   uint32_t all_channels = 0;
   for (uint8_t i = 0; i < 4u; i++)
   {
      if (channel.test(i))
      {
         all_channels = all_channels | get_timer_channel(static_cast<OutputChannel>(i));
      }
   }

   LL_TIM_CC_EnableChannel(m_config.timer_handle, all_channels);
}

void Timer::enable_all_channels()
{
   LL_TIM_CC_EnableChannel(m_config.timer_handle, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH4);
}

void Timer::enable_all_outputs()
{
   LL_TIM_EnableAllOutputs(m_config.timer_handle);
}

}   // namespace hw::timer
