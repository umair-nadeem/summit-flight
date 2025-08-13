#pragma once

#include <span>

#include "error/error_handler.hpp"
#include "interfaces/ILogChannel.hpp"
#include "interfaces/hw/IUartTransmitter.hpp"

namespace logging
{

template <interfaces::hw::IUartTransmitter UartTransmitter>
class LogUart
{
public:
   explicit LogUart(UartTransmitter& uart_transmitter)
       : m_uart_transmitter{uart_transmitter}

   {
   }

   void publish_log(std::span<const char> msg)
   {
      std::span<std::byte> tx_buffer = m_uart_transmitter.get_buffer();
      error::verify(tx_buffer.size() >= msg.size());

      const auto msg_len            = std::min(static_cast<uint32_t>(msg.size()), static_cast<uint32_t>(tx_buffer.size()));
      uint32_t   length_to_transmit = 0;

      for (; length_to_transmit < msg_len; length_to_transmit++)
      {
         tx_buffer[length_to_transmit] = static_cast<std::byte>(msg[length_to_transmit]);
         if (msg[length_to_transmit] == '\0')
         {
            break;
         }
      }

      m_uart_transmitter.send_and_return(std::min(length_to_transmit, msg_len));
   }

private:
   UartTransmitter& m_uart_transmitter;

   static_assert(interfaces::ILogChannel<LogUart<UartTransmitter>>);
};

}   // namespace logging
