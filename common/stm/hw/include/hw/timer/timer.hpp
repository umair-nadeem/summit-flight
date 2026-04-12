#pragma once

#include "error/error_handler.hpp"
#include "stm32f4xx_ll_tim.h"

namespace hw::timer
{

inline void enable_timer_dma(TIM_TypeDef* timer_handle, const uint32_t timer_channel) noexcept
{
   switch (timer_channel)
   {
      case LL_TIM_CHANNEL_CH1:
         LL_TIM_EnableDMAReq_CC1(timer_handle);
         break;

      case LL_TIM_CHANNEL_CH2:
         LL_TIM_EnableDMAReq_CC2(timer_handle);
         break;

      case LL_TIM_CHANNEL_CH3:
         LL_TIM_EnableDMAReq_CC3(timer_handle);
         break;

      case LL_TIM_CHANNEL_CH4:
         LL_TIM_EnableDMAReq_CC4(timer_handle);
         break;

      default:
         error::stop_operation();
         break;
   }
}

}   // namespace hw::timer
