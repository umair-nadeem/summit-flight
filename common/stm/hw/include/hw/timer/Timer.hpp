#pragma once

#include "TimerConfig.hpp"
#include "error/error_handler.hpp"
#include "stm32f4xx_ll_tim.h"

namespace hw::timer
{

class Timer
{
public:
   explicit Timer(TimerConfig& config)
       : m_config{config}
   {
      error::verify(m_config.timer_handle != nullptr);
   }

   void start();
   void enable_interrupt();
   void clear_update_flag();
   bool is_update_flag_active() const;

   enum class OutputChannel : uint8_t
   {
      channel1 = 0,
      channel2,
      channel3,
      channel4
   };

   void set_compare_value_for_channel(const OutputChannel channel, const uint32_t value);
   void enable_all_outputs();

private:
   TimerConfig& m_config;
};

}   // namespace hw::timer
