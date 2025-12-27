#include "crsf/Crsf.hpp"

#include <optional>

#include "error/error_handler.hpp"

namespace
{

enum class ParserState
{
   sync_byte_check,
   length_and_type_check,
   crc_validation
};

static uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xff, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

uint8_t crc8(std::span<const uint8_t> buff)
{
   const std::size_t len = buff.size();
   uint8_t           crc = 0;
   for (uint8_t i = 0; i < len; i++)
   {
      crc = crc8tab[crc ^ buff[i]];
   }
   return crc;
}

bool packet_type_is_supported(const uint8_t packet_type)
{
   if ((packet_type == static_cast<uint8_t>(crsf::FrameType::rc_channels_packed)) ||
       (packet_type == static_cast<uint8_t>(crsf::FrameType::link_statistics)) ||
       (packet_type == static_cast<uint8_t>(crsf::FrameType::link_statistics_tx)))
   {
      return true;
   }

   return false;
}

std::optional<uint8_t> get_expected_payload_size(const uint8_t packet_type)
{
   if (packet_type == static_cast<uint8_t>(crsf::FrameType::rc_channels_packed))
   {
      return static_cast<uint8_t>(crsf::PayloadSize::rc_channels_packed);
   }

   if (packet_type == static_cast<uint8_t>(crsf::FrameType::link_statistics))
   {
      return static_cast<uint8_t>(crsf::PayloadSize::link_statistics);
   }
   if (packet_type == static_cast<uint8_t>(crsf::FrameType::link_statistics_tx))
   {
      return static_cast<uint8_t>(crsf::PayloadSize::link_statistics_tx);
   }

   return std::nullopt;
}

}   // namespace

namespace crsf
{

CrsfRcChannels       Crsf::rc_channels{};
CrsfLinkStatistics   Crsf::link_statistics{};
CrsfLinkStatisticsTx Crsf::link_statistics_tx{};

// returns the number of bytes processed
bool Crsf::parse_buffer(std::span<const uint8_t> buffer, CrsfPacket& packet)
{
   const std::size_t      buffer_size           = buffer.size();
   ParserState            state                 = ParserState::sync_byte_check;
   uint8_t                current_byte          = 0;
   uint8_t                frame_len             = 0;
   std::optional<uint8_t> expected_payload_size = std::nullopt;
   uint8_t                type                  = 0;
   uint8_t                crc                   = 0;

   while (current_byte < buffer_size)
   {
      switch (state)
      {
         case ParserState::sync_byte_check:
            if ((buffer_size - current_byte) <= min_valid_frame_size)
            {
               return false;
            }

            if (buffer[current_byte] == sync_byte)
            {
               state = ParserState::length_and_type_check;
            }
            current_byte++;
            break;

         case ParserState::length_and_type_check:
            frame_len = buffer[current_byte++];
            type      = buffer[current_byte];

            if (packet_type_is_supported(type))
            {
               expected_payload_size = get_expected_payload_size(type);
               if ((!expected_payload_size.has_value()) ||
                   (frame_len != (expected_payload_size.value() + num_bytes_frame_type + num_bytes_frame_crc)) ||
                   ((frame_len + current_byte) > buffer_size))
               {
                  return false;
               }

               packet.type = static_cast<FrameType>(type);
               state       = ParserState::crc_validation;
            }
            else
            {
               if (frame_len + current_byte < buffer_size)
               {
                  // scan again for next frame (if available)
                  current_byte += frame_len;
                  state = ParserState::sync_byte_check;
               }
               else
               {
                  return false;
               }
            }

            break;

         case ParserState::crc_validation:
            crc = buffer[current_byte + expected_payload_size.value() + 1u];
            if (crc8(std::span{buffer.data() + current_byte, expected_payload_size.value() + 1u}) == crc)
            {
               process_packet(std::span{buffer.data() + current_byte + 1u, expected_payload_size.value()}, packet);
               return true;
            }

            return false;

         default:
            error::stop_operation();
            break;
      }
   }

   return false;
}

uint32_t Crsf::serialize_battery_telemetry(const CrsfBattery& packet, std::span<uint8_t> out)
{

   static constexpr std::size_t total_bytes_required = num_bytes_sync_byte +
                                                       num_bytes_frame_size +
                                                       num_bytes_frame_type +
                                                       num_bytes_frame_crc +
                                                       static_cast<uint8_t>(crsf::PayloadSize::battery);
   static constexpr std::size_t frame_len = total_bytes_required - 2u;
   static constexpr std::size_t crc_len   = num_bytes_frame_type + static_cast<uint8_t>(crsf::PayloadSize::battery);

   error::verify(out.size() >= total_bytes_required);

   uint32_t bytes_written = 0;
   out[bytes_written++]   = sync_byte;
   out[bytes_written++]   = frame_len;
   out[bytes_written++]   = static_cast<uint8_t>(FrameType::battery_sensor);

   // voltage
   const int16_t voltage = packet.voltage_10uv;
   out[bytes_written++]  = static_cast<uint8_t>(voltage >> 8u) & 0xff;
   out[bytes_written++]  = static_cast<uint8_t>(voltage) & 0xff;

   // current
   const int16_t current = packet.current_10ua;
   out[bytes_written++]  = static_cast<uint8_t>(current >> 8u) & 0xff;
   out[bytes_written++]  = static_cast<uint8_t>(current) & 0xff;

   // capacity uint24_t
   const uint32_t capacity = packet.capacity_used_mah & 0xffffff;
   out[bytes_written++]    = static_cast<uint8_t>(capacity >> 16u) & 0xff;
   out[bytes_written++]    = static_cast<uint8_t>(capacity >> 8u) & 0xff;
   out[bytes_written++]    = static_cast<uint8_t>(capacity) & 0xff;

   out[bytes_written++] = packet.remaining_pct;

   const uint8_t crc = crc8(std::span{out.data() + num_bytes_sync_byte + num_bytes_frame_size, crc_len});

   out[bytes_written++] = crc;

   return bytes_written;
}

void Crsf::process_rc_channels(std::span<const uint8_t> frame, crsf::CrsfPacket& packet)
{
   error::verify(frame.size() == static_cast<uint8_t>(crsf::PayloadSize::rc_channels_packed));

   rc_channels.channels[0]  = static_cast<uint16_t>((frame[0] | frame[1u] << 8u) & 0x07ff);
   rc_channels.channels[1]  = static_cast<uint16_t>((frame[1u] >> 3u | frame[2u] << 5u) & 0x07ff);
   rc_channels.channels[2]  = static_cast<uint16_t>((frame[2u] >> 6u | frame[3u] << 2u | frame[4] << 10u) & 0x07ff);
   rc_channels.channels[3]  = static_cast<uint16_t>((frame[4u] >> 1u | frame[5u] << 7u) & 0x07ff);
   rc_channels.channels[4]  = static_cast<uint16_t>((frame[5u] >> 4u | frame[6u] << 4u) & 0x07ff);
   rc_channels.channels[5]  = static_cast<uint16_t>((frame[6u] >> 7u | frame[7u] << 1u | frame[8] << 9u) & 0x07ff);
   rc_channels.channels[6]  = static_cast<uint16_t>((frame[8u] >> 2u | frame[9u] << 6u) & 0x07ff);
   rc_channels.channels[7]  = static_cast<uint16_t>((frame[9u] >> 5u | frame[10u] << 3u) & 0x07ff);
   rc_channels.channels[8]  = static_cast<uint16_t>((frame[11u] | frame[12u] << 8u) & 0x07ff);
   rc_channels.channels[9]  = static_cast<uint16_t>((frame[12u] >> 3u | frame[13u] << 5u) & 0x07ff);
   rc_channels.channels[10] = static_cast<uint16_t>((frame[13u] >> 6u | frame[14u] << 2u | frame[15] << 10u) & 0x07ff);
   rc_channels.channels[11] = static_cast<uint16_t>((frame[15u] >> 1u | frame[16u] << 7u) & 0x07ff);
   rc_channels.channels[12] = static_cast<uint16_t>((frame[16u] >> 4u | frame[17u] << 4u) & 0x07ff);
   rc_channels.channels[13] = static_cast<uint16_t>((frame[17u] >> 7u | frame[18u] << 1u | frame[19] << 9u) & 0x07ff);
   rc_channels.channels[14] = static_cast<uint16_t>((frame[19u] >> 2u | frame[20u] << 6u) & 0x07ff);
   rc_channels.channels[15] = static_cast<uint16_t>((frame[20u] >> 5u | frame[21u] << 3u) & 0x07ff);

   packet.data = rc_channels;
}

void Crsf::process_link_statistics(std::span<const uint8_t> frame, crsf::CrsfPacket& packet)
{
   error::verify(frame.size() == static_cast<uint8_t>(crsf::PayloadSize::link_statistics));

   link_statistics.uplink_rssi_1         = frame[0];
   link_statistics.uplink_rssi_2         = frame[1];
   link_statistics.uplink_link_quality   = frame[2];
   link_statistics.uplink_snr            = static_cast<int8_t>(frame[3]);
   link_statistics.active_antenna        = frame[4];
   link_statistics.rf_profile            = frame[5];
   link_statistics.uplink_rf_power       = frame[6];
   link_statistics.downlink_rssi         = frame[7];
   link_statistics.downlink_link_quality = frame[8];
   link_statistics.downlink_snr          = static_cast<int8_t>(frame[9]);

   packet.data = link_statistics;
}

void Crsf::process_link_statistics_tx(std::span<const uint8_t> frame, crsf::CrsfPacket& packet)
{
   error::verify(frame.size() == static_cast<uint8_t>(crsf::PayloadSize::link_statistics_tx));

   link_statistics_tx.uplink_rssi_db       = frame[0];
   link_statistics_tx.uplink_rssi_pct      = frame[1];
   link_statistics_tx.uplink_link_quality  = frame[2];
   link_statistics_tx.uplink_snr           = static_cast<int8_t>(frame[3]);
   link_statistics_tx.downlink_rf_power_db = frame[4];
   link_statistics_tx.uplink_fps           = frame[5];

   packet.data = link_statistics_tx;
}

void Crsf::process_packet(std::span<const uint8_t> frame, crsf::CrsfPacket& packet)
{
   switch (packet.type)
   {
      case crsf::FrameType::rc_channels_packed:
         process_rc_channels(frame, packet);
         break;

      case crsf::FrameType::link_statistics:
         process_link_statistics(frame, packet);
         break;

      case crsf::FrameType::link_statistics_tx:
         process_link_statistics_tx(frame, packet);
         break;

      case crsf::FrameType::airspeed:
      case crsf::FrameType::attitude:
      case crsf::FrameType::battery_sensor:
      case crsf::FrameType::gps:
      case crsf::FrameType::heartbeat:
      case crsf::FrameType::link_statistics_rx:
      default:
         error::stop_operation();
         break;
   }
}

}   // namespace crsf
