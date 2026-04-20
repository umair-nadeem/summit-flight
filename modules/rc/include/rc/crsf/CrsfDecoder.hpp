#pragma once

#include "rc/crsf/Channel.hpp"
#include "rc/crsf/CrsfDecoderResult.hpp"
#include "rc/crsf/LinkStats.hpp"

namespace rc::crsf
{

class CrsfDecoder
{
public:
   explicit CrsfDecoder(const ChannelConfigs& channel_configs);

   CrsfDecoderResult decode_crsf(std::span<const uint8_t> buffer);

   const ChannelValues& get_rc_channels() const;
   const LinkStats&     get_link_stats() const;

private:
   void decode_rc_channels(const ::crsf::CrsfRcChannels& rc_channels);

   void decode_link_stats(const ::crsf::CrsfLinkStatistics& link_stats);

   const ChannelConfigs& m_channel_configs;
   ::crsf::CrsfPacket    m_crsf_packet{};
   ChannelValues         m_rc_channels{};
   LinkStats             m_link_stats{};
};

}   // namespace rc::crsf
