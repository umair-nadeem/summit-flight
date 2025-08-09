#pragma once

#include "interfaces/ILogChannel.hpp"
#include "interfaces/rtos/IQueueReceiver.hpp"

namespace logging
{

template <interfaces::rtos::IQueueReceiver<params::LogBuffer> QueueReceiver, interfaces::ILogChannel LogUart>
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
      auto msg = m_queue_receiver.receive_blocking();

      m_log_uart.publish_log(msg);
   }

private:
   QueueReceiver& m_queue_receiver;
   LogUart&       m_log_uart;
};

}   // namespace logging
