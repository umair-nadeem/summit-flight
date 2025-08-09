#pragma once

#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_spi.h"

namespace hw::spi
{

struct SpiConfig
{
   SPI_TypeDef* spi_handle;
   DMA_TypeDef* dma_handle;
   uint32_t     tx_dma_stream;
   uint32_t     rx_dma_stream;
};

}   // namespace hw::spi
