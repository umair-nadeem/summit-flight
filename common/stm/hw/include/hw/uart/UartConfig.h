#pragma once

#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_usart.h"

namespace hw::uart
{

struct UartConfig
{
   USART_TypeDef* uart_handle;
   DMA_TypeDef*   dma_handle;
   uint32_t       tx_dma_stream;
   uint32_t       rx_dma_stream;
};

}   // namespace hw::uart
