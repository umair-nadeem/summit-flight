#pragma once

#include "boundaries/BufferWithOwnershipIndex.hpp"
#include "crsf/crsf.h"
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
      const auto queue_item = m_radio_input_queue_receiver.receive_if_available();
      if (queue_item.has_value())
      {
         const auto                     radio_input = queue_item.value();
         const std::span<const uint8_t> bytes       = std::span{reinterpret_cast<const uint8_t*>(radio_input.buffer.data()), radio_input.buffer.size()};
         if (crsf::crsf_parse(bytes, m_rc_channels))
         {
            m_counter++;
            if (m_counter % 10 == 0)
            {
               m_logger.printf("size: %u, data: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u", bytes.size(),
                               m_rc_channels[0], m_rc_channels[1], m_rc_channels[2],
                               m_rc_channels[3], m_rc_channels[4], m_rc_channels[5],
                               m_rc_channels[6], m_rc_channels[7], m_rc_channels[8],
                               m_rc_channels[9], m_rc_channels[10], m_rc_channels[11],
                               m_rc_channels[12], m_rc_channels[12], m_rc_channels[14]);
            }
         }

         m_free_index_queue_sender.send_blocking(radio_input.index);
      }
   }

private:
   QueueReceiver&                                 m_radio_input_queue_receiver;
   QueueSender&                                   m_free_index_queue_sender;
   Logger&                                        m_logger;
   std::array<uint16_t, crsf::total_num_channels> m_rc_channels{};
   std::size_t                                    m_counter{};
};

}   // namespace aeromight_link
