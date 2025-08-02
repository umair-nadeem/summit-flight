#include "hw/uart/Transmitter.h"

#include "error/error_handler.h"

namespace hw::uart
{

Transmitter::Transmitter(UartConfig&                 uart_config,
                         std::span<std::byte>        tx_buffer,
                         const FunctionTakeSemaphore function_take_semaphore)
    : m_uart_config(uart_config),
      m_tx_buffer{tx_buffer},
      m_function_take_semaphore{std::move(function_take_semaphore)}
{
   error::verify(m_function_take_semaphore != nullptr);
}

void Transmitter::send(const uint32_t size)
{
   start_tx(size);
   m_tx_in_progress = true;
}

void Transmitter::blocking_send(const uint32_t size)
{
   send(size);
   wait_for_tx_to_complete();
}

void Transmitter::wait_for_tx_to_complete()
{
   if (m_tx_in_progress)
   {
      m_function_take_semaphore();   // blocks until transfer is complete
      m_tx_in_progress = false;
   }
}

void Transmitter::start_tx(const uint32_t size)
{
   wait_for_tx_to_complete();

   LL_DMA_SetDataLength(m_uart_config.dma_handle, m_uart_config.tx_dma_stream, size);
   LL_DMA_EnableStream(m_uart_config.dma_handle, m_uart_config.tx_dma_stream);
}

}   // namespace hw::uart
