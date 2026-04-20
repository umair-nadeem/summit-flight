#include "rc/crsf/CrsfDecoder.hpp"

#include <algorithm>
#include <cmath>

namespace
{

float normalize_channel(const uint16_t raw, const float min_norm, const float max_norm)
{
   constexpr float raw_min = static_cast<float>(::crsf::rc_channel_value_min);
   constexpr float raw_max = static_cast<float>(::crsf::rc_channel_value_max);

   float t = (static_cast<float>(raw) - raw_min) / (raw_max - raw_min);
   t       = std::clamp(t, 0.0f, 1.0f);

   return min_norm + (t * (max_norm - min_norm));
}

float apply_deadband(const float x, const float deadband)
{
   if (deadband <= 0.0f)
   {
      return x;
   }

   if (std::fabs(x) < deadband)
   {
      return 0.0f;
   }

   return (x > 0.0f) ? (x - deadband) / (1.0f - deadband) : (x + deadband) / (1.0f - deadband);
}

rc::crsf::Bistate decode_bistate(const float norm)
{
   return (norm > 0.0f) ? rc::crsf::Bistate::high : rc::crsf::Bistate::low;
}

rc::crsf::Tristate decode_tristate(const float norm)
{
   if (norm < -0.33f)
   {
      return rc::crsf::Tristate::low;
   }
   if (norm > 0.33f)
   {
      return rc::crsf::Tristate::high;
   }

   return rc::crsf::Tristate::mid;
}

}   // namespace

namespace rc::crsf
{

CrsfDecoder::CrsfDecoder(const ChannelConfigs& channel_configs)
    : m_channel_configs{channel_configs}
{
   for (const auto& config : m_channel_configs)
   {
      error::verify((0.0f <= config.deadband) && (config.deadband < 1.0f));
      error::verify((config.min_norm < config.max_norm));
   }
}

CrsfDecoderResult CrsfDecoder::decode_crsf(std::span<const uint8_t> buffer)
{
   using namespace ::crsf;
   const bool result = Crsf::parse_buffer(buffer, m_crsf_packet);

   if (!result)
   {
      return CrsfDecoderResult::invalid_frame;
   }

   switch (m_crsf_packet.type)
   {
      case FrameType::rc_channels_packed:
         decode_rc_channels(std::get<CrsfRcChannels>(m_crsf_packet.data));
         return CrsfDecoderResult::rc_channels_update;

      case FrameType::link_statistics:
         decode_link_stats(std::get<CrsfLinkStatistics>(m_crsf_packet.data));
         return CrsfDecoderResult::link_stats_update;

      case FrameType::airspeed:
      case FrameType::attitude:
      case FrameType::battery_sensor:
      case FrameType::gps:
      case FrameType::heartbeat:
      case FrameType::link_statistics_tx:
      case FrameType::link_statistics_rx:
      default:
         return CrsfDecoderResult::other_packet;
   }
}

void CrsfDecoder::decode_rc_channels(const ::crsf::CrsfRcChannels& rc_channels)
{
   for (std::size_t i = 0; i < ::crsf::rc_channel_count; i++)
   {
      const uint16_t raw    = rc_channels.channels[i];
      const auto&    config = m_channel_configs[i];

      switch (config.type)
      {
         case ChannelType::unused:
            m_rc_channels[i] = std::monostate{};
            break;

         case ChannelType::raw_u16:
            m_rc_channels[i] = raw;
            break;

         case ChannelType::normalized_float:
         {
            const float norm = normalize_channel(raw, config.min_norm, config.max_norm);
            m_rc_channels[i] = apply_deadband(norm, config.deadband);
            break;
         }

         case ChannelType::bistate:
         {
            const float norm = normalize_channel(raw, -1.0f, 1.0f);
            m_rc_channels[i] = decode_bistate(norm);
            break;
         }

         case ChannelType::tristate:
         {
            const float norm = normalize_channel(raw, -1.0f, 1.0f);
            m_rc_channels[i] = decode_tristate(norm);
            break;
         }

         default:
            error::stop_operation();
            break;
      }
   }
}

void CrsfDecoder::decode_link_stats(const ::crsf::CrsfLinkStatistics& link_stats)
{
   m_link_stats.uplink_rssi_1_dbm  = -static_cast<int8_t>(link_stats.uplink_rssi_1);
   m_link_stats.uplink_rssi_2_dbm  = -static_cast<int8_t>(link_stats.uplink_rssi_2);
   m_link_stats.uplink_quality_pct = link_stats.uplink_link_quality;
   m_link_stats.uplink_snr_db      = link_stats.uplink_snr;

   m_link_stats.downlink_rssi_dbm    = -static_cast<int8_t>(link_stats.downlink_rssi);
   m_link_stats.downlink_quality_pct = link_stats.downlink_link_quality;
   m_link_stats.downlink_snr_db      = link_stats.downlink_snr;

   m_link_stats.active_antenna = link_stats.active_antenna;
   m_link_stats.rf_profile     = link_stats.rf_profile;
   m_link_stats.tx_power_mw    = ::crsf::get_uplink_rf_power_mw(link_stats.uplink_rf_power);
}

const ChannelValues& CrsfDecoder::get_rc_channels() const
{
   return m_rc_channels;
}

const LinkStats& CrsfDecoder::get_link_stats() const
{
   return m_link_stats;
}

}   // namespace rc::crsf
