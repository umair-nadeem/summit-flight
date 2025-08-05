#include "LoggingTaskData.hpp"
#include "error/error_handler.hpp"
#include "hw/uart/uart.hpp"
#include "logging/LogServer.hpp"
#include "logging/LogUart.hpp"
#include "task_params.hpp"

extern "C"
{

   [[noreturn]] void logging_task(void* params)
   {
      error::verify(params != nullptr);
      auto* data = static_cast<controller::LoggingTaskData*>(params);

      logging::LogUart<decltype(data->logging_uart.transmitter)> log_uart{data->logging_uart.transmitter};

      logging::LogServer<decltype(data->logging_queue_receiver),
                         decltype(log_uart)>
          log_server{data->logging_queue_receiver,
                     log_uart};

      while (true)
      {
         log_server.run_once();
      }
   }
}

extern "C"
{
   void USART1_IRQHandler()
   {
      auto& data = controller::logging_task_data;
      hw::uart::handle_uart_global_interrupt(data.logging_uart.config, data.logging_uart.isr_sem_giver);
   }

   void DMA2_Stream2_IRQHandler()
   {
      // do nothing, because rx is not applicable
   }
}
