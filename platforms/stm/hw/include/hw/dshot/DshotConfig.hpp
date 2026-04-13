#pragma once

#include "stm32f4xx_ll_dma.h"

namespace hw::dshot
{

struct DshotConfig
{
   DMA_TypeDef* dma;
   uint32_t     stream;

   TIM_TypeDef* timer_handle;
   uint32_t     timer_channel;

   volatile uint32_t* ccr;
};

}   // namespace hw::dshot
