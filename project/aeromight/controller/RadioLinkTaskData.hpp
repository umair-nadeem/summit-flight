#pragma once

#include "boundaries/BufferWithOwnershipIndex.hpp"
#include "crsf/params.hpp"
#include "hw/uart/Transmitter.hpp"
#include "hw/uart/UartConfig.hpp"
#include "rtos/QueueReceiver.hpp"
#include "rtos/QueueSender.hpp"
#include "rtos/SemaphoreGiver.hpp"
#include "rtos/SemaphoreTaker.hpp"

extern "C"
{

   [[noreturn]] void radio_link_task(void* const params);

}   // extern "C"

namespace controller
{

struct RadioLinkTaskData
{
   static constexpr std::size_t queue_depth = 8u;

   struct RadioLinkUart
   {
      using Message = boundaries::BufferWithOwnershipIndex<std::byte>;

      hw::uart::UartConfig config{
          .uart_handle   = USART2,
          .dma_handle    = DMA1,
          .tx_dma_stream = LL_DMA_STREAM_6,
          .rx_dma_stream = LL_DMA_STREAM_5};

      std::array<std::byte, crsf::max_buffer_size>                          dma_tx_buffer{};
      std::array<std::byte, crsf::max_buffer_size>                          dma_rx_buffer{};
      std::array<std::array<std::byte, crsf::max_buffer_size>, queue_depth> user_rx_buffer_pool{};

      rtos::SemaphoreTaker                                   transmitter_sem_taker{};
      rtos::SemaphoreGiver                                   isr_sem_giver{};
      hw::uart::Transmitter<decltype(transmitter_sem_taker)> transmitter{config,
                                                                         std::as_writable_bytes(std::span{dma_tx_buffer}),
                                                                         transmitter_sem_taker};

      // queue's task endpoints
      rtos::QueueReceiver<Message>   radio_input_receiver{};
      rtos::QueueSender<std::size_t> radio_queue_buffer_index_sender{};

      // queue's isr endpoints
      rtos::QueueSender<Message>       radio_input_sender_from_isr{};
      rtos::QueueReceiver<std::size_t> radio_queue_buffer_index_receiver_from_isr{};

   } radio_link_uart{};
};

extern RadioLinkTaskData radio_link_task_data;

}   // namespace controller
