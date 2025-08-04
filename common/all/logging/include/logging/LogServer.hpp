#pragma once

#include "interfaces/ILogChannel.hpp"

namespace logging
{

template <typename QueueReceiver, interfaces::ILogChannel LogUart>
class LogServer
{
public:
   explicit LogServer(QueueReceiver& queue_receiver, LogUart& log_uart)
       : m_queue_receiver{queue_receiver},
         m_log_uart{log_uart}
   {
   }

   void run_once()
   {
      // blocking call
      auto msg = m_queue_receiver.receive();

      m_log_uart.publish_log(msg);
   }

private:
   QueueReceiver& m_queue_receiver;
   LogUart&       m_log_uart;
};

}   // namespace logging
