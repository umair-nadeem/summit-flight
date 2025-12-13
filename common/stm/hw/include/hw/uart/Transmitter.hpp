#pragma once

#include <functional>
#include <span>

#include "UartConfig.hpp"
#include "interfaces/hw/IUartTransmitter.hpp"

namespace hw::uart
{

template <typename SemaphoreTaker>
class Transmitter
{
public:
   explicit Transmitter(UartConfig&          uart_config,
                        std::span<std::byte> tx_buffer,
                        SemaphoreTaker&      semaphore_taker)
       : m_uart_config(uart_config),
         m_tx_buffer{tx_buffer},
         m_semaphore_taker{semaphore_taker}
   {
   }

   void send_blocking(const uint32_t size)
   {
      start_tx(size);
      m_tx_in_progress = true;
      wait_for_tx_to_complete();
   }

   std::span<std::byte> get_buffer() const
   {
      return m_tx_buffer;
   }

private:
   void wait_for_tx_to_complete()
   {
      if (m_tx_in_progress)
      {
         m_semaphore_taker.take();   // blocks until transfer is complete
         m_tx_in_progress = false;
      }
   }

   void start_tx(const uint32_t size)
   {
      wait_for_tx_to_complete();

      LL_DMA_SetDataLength(m_uart_config.dma_handle, m_uart_config.tx_dma_stream, size);
      LL_DMA_EnableStream(m_uart_config.dma_handle, m_uart_config.tx_dma_stream);
   }

   UartConfig&          m_uart_config;
   std::span<std::byte> m_tx_buffer;
   SemaphoreTaker&      m_semaphore_taker;
   volatile bool        m_tx_in_progress{false};
};

static_assert(interfaces::hw::IUartTransmitter<Transmitter<int>>);

}   // namespace hw::uart
