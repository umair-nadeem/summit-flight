#pragma once

#include <span>

#include "UartConfig.hpp"
#include "boundaries/BufferWithOwnershipIndex.hpp"
#include "error/error_handler.hpp"
#include "hw/dma/dma.hpp"

namespace hw::uart
{

void prepare_for_communication(UartConfig& config, std::span<const std::byte> tx_buffer, std::span<const std::byte> rx_buffer);
void start_rx(UartConfig& config, std::span<const std::byte> rx_buffer);

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
      ::hw::dma::clear_dma_tc_flag(config.dma_handle, config.tx_dma_stream);

      isr_sem_giver.give_from_isr();
   }
}

template <typename QueueSenderFromIsr, typename BufferPoolIndexQueueReceiverFromIsr, std::size_t N>
inline void handle_uart_dma_rx_global_interrupt(UartConfig&                         config,
                                                QueueSenderFromIsr&                 queue_sender_from_isr,
                                                BufferPoolIndexQueueReceiverFromIsr buffer_pool_index_queue_receiver_from_isr,
                                                const std::array<std::byte, N>&     dma_rx_buffer,
                                                std::span<std::array<std::byte, N>> rx_buffer_pool)
{
   if (::hw::dma::is_dma_tc_flag_active(config.dma_handle, config.rx_dma_stream))
   {
      ::hw::dma::clear_dma_tc_flag(config.dma_handle, config.rx_dma_stream);

      std::size_t index      = 0;
      const bool  have_space = buffer_pool_index_queue_receiver_from_isr.receive_from_isr(index);

      if (have_space)
      {
         auto&             buffer = rx_buffer_pool[index];
         const std::size_t length = dma_rx_buffer.size() - LL_DMA_GetDataLength(config.dma_handle, config.rx_dma_stream);

         error::verify(length <= buffer.size());
         std::copy(dma_rx_buffer.begin(), dma_rx_buffer.begin() + static_cast<std::ptrdiff_t>(length), buffer.begin());

         boundaries::BufferWithOwnershipIndex<std::byte> msg{{buffer.data(), length},
                                                             index};

         [[maybe_unused]] bool result = queue_sender_from_isr.send_from_isr(msg);
      }

      start_rx(config, dma_rx_buffer);   // re-enable dma receive
   }
}

}   // namespace hw::uart
