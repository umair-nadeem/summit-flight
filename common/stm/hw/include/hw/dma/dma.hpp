#pragma once

#include "stm32f4xx_ll_dma.h"

namespace hw::dma
{

void clear_dma_tc_flag(DMA_TypeDef* dma_handle, const uint32_t dma_stream);
void clear_dma_te_flag(DMA_TypeDef* dma_handle, const uint32_t dma_stream);

bool is_dma_tc_flag_active(DMA_TypeDef* dma_handle, const uint32_t dma_stream);

}   // namespace hw::dma
