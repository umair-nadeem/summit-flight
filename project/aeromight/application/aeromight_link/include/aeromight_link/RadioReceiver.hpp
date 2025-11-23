#pragma once

#include "boundaries/BufferWithOwnershipIndex.hpp"
#include "interfaces/IClockSource.hpp"

namespace aeromight_link
{

template <typename QueueReceiver, typename QueueSender, interfaces::IClockSource ClockSource, typename Logger>
class RadioReceiver
{
public:
   explicit RadioReceiver(QueueReceiver& radio_input_queue_receiver,
                          QueueSender&   free_index_queue_sender,
                          Logger&        logger)
       : m_radio_input_queue_receiver{radio_input_queue_receiver},
         m_free_index_queue_sender(free_index_queue_sender),
         m_logger(logger)
   {
      m_logger.enable();
   }

   void execute()
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

private:
   QueueReceiver& m_radio_input_queue_receiver;
   QueueSender&   m_free_index_queue_sender;
   Logger&        m_logger;
   std::size_t    m_counter{};
};

}   // namespace aeromight_link
