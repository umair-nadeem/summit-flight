#pragma once

#include <bitset>

#include "TimerConfig.hpp"
#include "error/error_handler.hpp"
#include "stm32f4xx_ll_tim.h"

namespace hw::timer
{

using ChannelType = std::bitset<4u>;

enum class OutputChannel : uint8_t
{
   channel1 = 0,
   channel2,
   channel3,
   channel4
};

class Timer
{
public:
   explicit Timer(TimerConfig& config)
       : m_config{config}
   {
      error::verify(m_config.timer_handle != nullptr);
   }

   void enable_interrupt();
   void clear_update_flag();
   bool is_update_flag_active() const;
   void set_compare_value_for_channel(const OutputChannel channel, const uint32_t value);

   void enable_counter();
   void enable_channel(ChannelType channel);
   void enable_all_channels();
   void enable_all_outputs();

private:
   TimerConfig& m_config;
};

}   // namespace hw::timer
