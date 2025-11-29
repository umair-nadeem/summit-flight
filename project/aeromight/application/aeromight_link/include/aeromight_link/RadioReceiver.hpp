#pragma once

#include "boundaries/BufferWithOwnershipIndex.hpp"
#include "crsf/CrsfPacket.hpp"
#include "interfaces/IClockSource.hpp"

namespace aeromight_link
{

template <typename QueueReceiver, typename QueueSender, typename CrsfRcChannelsParser, typename Crsf, interfaces::IClockSource ClockSource, typename Logger>
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
         const auto radio_input = queue_item.value();

         process_radio_input(std::span{reinterpret_cast<const uint8_t*>(radio_input.buffer.data()), radio_input.buffer.size()});

         m_free_index_queue_sender.send_blocking(radio_input.index);   // send index to queue to indicate buffer is free
      }
   }

private:
   void process_radio_input(std::span<const uint8_t> buffer)
   {
      const bool result = Crsf::parse_buffer(buffer, m_crsf_packet);

      if (!result)
      {
         m_logger.printf("received corrupted frame of len %u", buffer.size());
         return;
      }

      switch (m_crsf_packet.type)   // type
      {
         case crsf::FrameType::rc_channels_packed:
            print_rc_channels(std::get<crsf::CrsfRcChannels>(m_crsf_packet.data).channels);
            break;

         case crsf::FrameType::link_statistics:
            print_link_stats(std::get<crsf::CrsfLinkStatistics>(m_crsf_packet.data));
            break;

         case crsf::FrameType::link_statistics_tx:
            break;

         case crsf::FrameType::airspeed:
         case crsf::FrameType::attitude:
         case crsf::FrameType::battery_sensor:
         case crsf::FrameType::gps:
         case crsf::FrameType::heartbeat:
         case crsf::FrameType::link_statistics_rx:
         default:
            break;
      }
   }

   void print_rc_channels(std::array<uint16_t, crsf::rc_channel_count>& m_rc_channels)
   {
      m_counter++;
      if (m_counter % 10 == 0)
      {
         m_logger.printf("data: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u",
                         m_rc_channels[0], m_rc_channels[1], m_rc_channels[2],
                         m_rc_channels[3], m_rc_channels[4], m_rc_channels[5],
                         m_rc_channels[6], m_rc_channels[7], m_rc_channels[8],
                         m_rc_channels[9], m_rc_channels[10], m_rc_channels[11],
                         m_rc_channels[12], m_rc_channels[12], m_rc_channels[14]);
      }
   }

   void print_link_stats(crsf::CrsfLinkStatistics& stats)
   {
      m_counter++;
      if (m_counter % 10 == 0)
      {
         m_logger.printf("data: %u, %u, %u, %d, %u, %u, %u, %u, %u, %d",
                         stats.uplink_rssi_1,
                         stats.uplink_rssi_2,
                         stats.uplink_link_quality,
                         stats.uplink_snr,
                         stats.active_antenna,
                         stats.rf_profile,
                         stats.uplink_rf_power,
                         stats.downlink_rssi,
                         stats.downlink_link_quality,
                         stats.downlink_snr);
      }
   }

   QueueReceiver&   m_radio_input_queue_receiver;
   QueueSender&     m_free_index_queue_sender;
   Logger&          m_logger;
   crsf::CrsfPacket m_crsf_packet{};
   std::size_t      m_counter{};
};

}   // namespace aeromight_link
