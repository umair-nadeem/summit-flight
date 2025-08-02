#include "hw/uart/uart.hpp"

#include "error/error_handler.h"

namespace hw::uart
{

void prepare_for_communication(UartConfig& config, std::span<const std::byte> tx_buffer, std::span<const std::byte> rx_buffer)
{
   LL_DMA_EnableIT_TC(config.dma_handle, config.rx_dma_stream);
   LL_USART_EnableIT_IDLE(config.uart_handle);
   LL_USART_EnableIT_TC(config.uart_handle);

   LL_DMA_ConfigTransfer(config.dma_handle, config.rx_dma_stream,
                         LL_DMA_PRIORITY_MEDIUM |
                             LL_DMA_MDATAALIGN_BYTE |
                             LL_DMA_PDATAALIGN_BYTE |
                             LL_DMA_MEMORY_INCREMENT |
                             LL_DMA_PERIPH_NOINCREMENT |
                             LL_DMA_MODE_NORMAL |
                             LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

   LL_DMA_ConfigTransfer(config.dma_handle, config.tx_dma_stream,
                         LL_DMA_PRIORITY_MEDIUM |
                             LL_DMA_MDATAALIGN_BYTE |
                             LL_DMA_PDATAALIGN_BYTE |
                             LL_DMA_MEMORY_INCREMENT |
                             LL_DMA_PERIPH_NOINCREMENT |
                             LL_DMA_MODE_NORMAL |
                             LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

   LL_DMA_SetPeriphAddress(config.dma_handle, config.tx_dma_stream, LL_USART_DMA_GetRegAddr(config.uart_handle));
   LL_DMA_SetPeriphAddress(config.dma_handle, config.rx_dma_stream, LL_USART_DMA_GetRegAddr(config.uart_handle));

   LL_DMA_SetMemoryAddress(config.dma_handle, config.tx_dma_stream, reinterpret_cast<uint32_t>(tx_buffer.data()));
   LL_DMA_SetMemoryAddress(config.dma_handle, config.rx_dma_stream, reinterpret_cast<uint32_t>(rx_buffer.data()));

   LL_USART_EnableDMAReq_TX(config.uart_handle);
   LL_USART_EnableDMAReq_RX(config.uart_handle);
}

void start_rx(UartConfig& config, std::span<const std::byte> rx_buffer)
{
   LL_DMA_SetDataLength(config.dma_handle, config.rx_dma_stream, rx_buffer.size());
   LL_DMA_EnableStream(config.dma_handle, config.rx_dma_stream);
}

void clear_dma_tc_flag(DMA_TypeDef* dma_handle, const uint32_t dma_stream)
{
   switch (dma_stream)
   {
      case LL_DMA_STREAM_0:
         LL_DMA_ClearFlag_TC0(dma_handle);
         break;

      case LL_DMA_STREAM_1:
         LL_DMA_ClearFlag_TC1(dma_handle);
         break;

      case LL_DMA_STREAM_2:
         LL_DMA_ClearFlag_TC2(dma_handle);
         break;

      case LL_DMA_STREAM_3:
         LL_DMA_ClearFlag_TC3(dma_handle);
         break;

      case LL_DMA_STREAM_4:
         LL_DMA_ClearFlag_TC4(dma_handle);
         break;

      case LL_DMA_STREAM_5:
         LL_DMA_ClearFlag_TC5(dma_handle);
         break;

      case LL_DMA_STREAM_6:
         LL_DMA_ClearFlag_TC6(dma_handle);
         break;

      case LL_DMA_STREAM_7:
         LL_DMA_ClearFlag_TC7(dma_handle);
         break;

      default:
         error::stop_operation();
         break;
   }
}

bool is_dma_tc_flag_active(DMA_TypeDef* dma_handle, const uint32_t dma_stream)
{
   switch (dma_stream)
   {
      case LL_DMA_STREAM_0:
         return (LL_DMA_IsActiveFlag_TC0(dma_handle) == 1u);

      case LL_DMA_STREAM_1:
         return (LL_DMA_IsActiveFlag_TC1(dma_handle) == 1u);

      case LL_DMA_STREAM_2:
         return (LL_DMA_IsActiveFlag_TC2(dma_handle) == 1u);

      case LL_DMA_STREAM_3:
         return (LL_DMA_IsActiveFlag_TC3(dma_handle) == 1u);

      case LL_DMA_STREAM_4:
         return (LL_DMA_IsActiveFlag_TC4(dma_handle) == 1u);

      case LL_DMA_STREAM_5:
         return (LL_DMA_IsActiveFlag_TC5(dma_handle) == 1u);

      case LL_DMA_STREAM_6:
         return (LL_DMA_IsActiveFlag_TC6(dma_handle) == 1u);

      case LL_DMA_STREAM_7:
         return (LL_DMA_IsActiveFlag_TC7(dma_handle) == 1u);

      default:
         error::stop_operation();
         break;
   }

   return false;
}

}   // namespace hw::uart
