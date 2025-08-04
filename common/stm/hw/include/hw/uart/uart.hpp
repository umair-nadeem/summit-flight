#pragma once

#include <span>

#include "UartConfig.h"

namespace hw::uart
{

void prepare_for_communication(UartConfig& config, std::span<const std::byte> tx_buffer, std::span<const std::byte> rx_buffer);
void start_rx(UartConfig& config, std::span<const std::byte> rx_buffer);

void clear_dma_tc_flag(DMA_TypeDef* dma_handle, const uint32_t dma_stream);
bool is_dma_tc_flag_active(DMA_TypeDef* dma_handle, const uint32_t dma_stream);

template <typename SemaphoreGiverFromIsr>
inline void handle_uart_global_interrupt(UartConfig& config, SemaphoreGiverFromIsr& isr_sem_giver)
{
   // uart is idle
   if (LL_USART_IsActiveFlag_IDLE(config.uart_handle) == 1u)
   {
      LL_USART_ClearFlag_IDLE(config.uart_handle);
      LL_DMA_DisableStream(config.dma_handle, config.rx_dma_stream);   // -> triggers tc flag on dma rx stream
   }

   // dma tx complete
   if (LL_USART_IsActiveFlag_TC(config.uart_handle) == 1u)
   {
      LL_USART_ClearFlag_TC(config.uart_handle);
      clear_dma_tc_flag(config.dma_handle, config.tx_dma_stream);

      isr_sem_giver.give_from_isr(false);
   }
}

template <typename QueueSenderFromIsr>
inline void handle_dma_rx_global_interrupt(UartConfig&                config,
                                           QueueSenderFromIsr&        queue_sender_from_isr,
                                           std::span<const std::byte> rx_buffer,
                                           std::span<std::byte>       rx_user_buffer)
{
   if (is_dma_tc_flag_active(config.dma_handle, config.rx_dma_stream))
   {
      clear_dma_tc_flag(config.dma_handle, config.tx_dma_stream);

      const std::size_t length = rx_buffer.size() - LL_DMA_GetDataLength(config.dma_handle, config.rx_dma_stream);
      std::copy(rx_buffer.begin(), rx_buffer.begin() + static_cast<std::ptrdiff_t>(length), rx_user_buffer.begin());

      std::span<const std::byte> received_data{rx_user_buffer.data(), length};

      queue_sender_from_isr.send_from_isr(received_data, false);

      start_rx(config, rx_buffer);
   }
}

}   // namespace hw::uart
