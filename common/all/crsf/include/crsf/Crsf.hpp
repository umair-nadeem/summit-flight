#pragma once

#include <span>

#include "crsf/CrsfBattery.hpp"
#include "crsf/CrsfPacket.hpp"

namespace crsf
{

class Crsf
{
public:
   static bool parse_buffer(std::span<const uint8_t> buffer, CrsfPacket& packet);

   static std::size_t serialize_battery_telemetry(const CrsfBattery& packet, std::span<uint8_t> out);

private:
   static void process_packet(std::span<const uint8_t> frame, crsf::CrsfPacket& packet);

   static void process_rc_channels(std::span<const uint8_t> frame, crsf::CrsfPacket& packet);

   static void process_link_statistics(std::span<const uint8_t> frame, crsf::CrsfPacket& packet);

   static void process_link_statistics_tx(std::span<const uint8_t> frame, crsf::CrsfPacket& packet);

   static CrsfRcChannels       rc_channels;
   static CrsfLinkStatistics   link_statistics;
   static CrsfLinkStatisticsTx link_statistics_tx;
};

}   // namespace crsf
