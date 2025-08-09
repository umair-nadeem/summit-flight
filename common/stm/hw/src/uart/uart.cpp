#include "hw/uart/uart.hpp"

#include "error/error_handler.hpp"

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

}   // namespace hw::uart
