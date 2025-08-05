#pragma once

#include <functional>
#include <span>

#include "UartConfig.hpp"
#include "interfaces/IUartTransmitter.hpp"

namespace hw::uart
{

class Transmitter
{
public:
   using FunctionTakeSemaphore = std::function<void()>;

   explicit Transmitter(UartConfig&                 uart_config,
                        std::span<std::byte>        tx_buffer,
                        const FunctionTakeSemaphore function_take_semaphore);

   void                 send_blocking(const uint32_t size);
   void                 send_and_return(const uint32_t size);
   std::span<std::byte> get_buffer() const;

private:
   void wait_for_tx_to_complete();
   void start_tx(const uint32_t size);

   UartConfig&                 m_uart_config;
   std::span<std::byte>        m_tx_buffer;
   const FunctionTakeSemaphore m_function_take_semaphore;
   volatile bool               m_tx_in_progress{false};
};

static_assert(interfaces::IUartTransmitter<Transmitter>);

}   // namespace hw::uart
