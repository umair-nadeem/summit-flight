#pragma once

#include "hw/uart/Transmitter.hpp"
#include "hw/uart/UartConfig.hpp"
#include "logging/log_params.hpp"
#include "rtos/QueueReceiver.hpp"
#include "rtos/SemaphoreGiver.hpp"
#include "rtos/SemaphoreTaker.hpp"

extern "C"
{
   [[noreturn]] void logging_task(void* params);
}

namespace controller
{

struct LoggingTaskData
{
   // logging uart
   struct LoggingUart
   {
      hw::uart::UartConfig config{
          .uart_handle   = USART1,
          .dma_handle    = DMA2,
          .tx_dma_stream = LL_DMA_STREAM_7,
          .rx_dma_stream = LL_DMA_STREAM_2};

      std::array<uint8_t, logging::params::max_log_len> dma_tx_buffer{};
      std::array<uint8_t, 1u>                           dummy_dma_rx_buffer{};

      rtos::SemaphoreTaker  transmitter_sem_taker{};
      rtos::SemaphoreGiver  isr_sem_giver{};
      hw::uart::Transmitter transmitter{config, std::as_writable_bytes(std::span{dma_tx_buffer}),
                                        [&]()
                                        { transmitter_sem_taker.take(); }};

   } logging_uart{};

   rtos::QueueReceiver<logging::params::LogBuffer> logging_queue_receiver{};
};

extern LoggingTaskData logging_task_data;

}   // namespace controller
