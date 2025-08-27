#pragma once

#include "stm32f4xx_ll_tim.h"

namespace hw::timer
{

struct TimerConfig
{
   TIM_TypeDef* timer_handle;
};

}   // namespace hw::timer
