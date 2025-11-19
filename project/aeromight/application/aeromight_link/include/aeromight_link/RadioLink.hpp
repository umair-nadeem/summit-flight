#pragma once

#include "boundaries/BufferWithOwnershipIndex.hpp"
#include "interfaces/IClockSource.hpp"

namespace aeromight_link
{

template <typename QueueReceiver, typename QueueSender, interfaces::IClockSource ClockSource, typename Logger>
class RadioLink
{
public:
   explicit RadioLink(QueueReceiver& radio_input_queue_receiver,
                      QueueSender&   free_index_queue_sender,
                      Logger&        logger,
                      uint32_t       period_in_ms)
       : m_radio_input_queue_receiver{radio_input_queue_receiver},
         m_free_index_queue_sender(free_index_queue_sender),
         m_logger(logger),
         m_period_in_ms(period_in_ms)
   {
      m_logger.enable();
   }

   void run_once()
   {
      const auto radio_input = m_radio_input_queue_receiver.receive_if_available();
      if (radio_input.has_value())
      {
         const std::span<const uint8_t> bytes = std::span{reinterpret_cast<const uint8_t*>(radio_input.value().buffer.data()), radio_input.value().buffer.size()};

         m_counter++;
         if (m_counter % 100 == 0)
         {
            m_logger.printf("size: %u, index: %u, data: %u, %u, %u", bytes.size(), radio_input.value().index, bytes[0], bytes[1], bytes[2]);
         }
         m_free_index_queue_sender.send_blocking(radio_input.value().index);
      }
   }

   uint32_t get_period_ms() const
   {
      return m_period_in_ms;
   }

private:
   QueueReceiver& m_radio_input_queue_receiver;
   QueueSender&   m_free_index_queue_sender;
   Logger&        m_logger;
   const uint32_t m_period_in_ms;
   std::size_t    m_counter{};
};

}   // namespace aeromight_link
