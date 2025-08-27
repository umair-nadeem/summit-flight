#pragma once

#include "TimerConfig.hpp"
#include "stm32f4xx_ll_tim.h"
#include "error/error_handler.hpp"

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
   void clear_update_flag();
   bool is_update_flag_active() const;

private:
   TimerConfig& m_config;
};

}   // namespace hw::timer
