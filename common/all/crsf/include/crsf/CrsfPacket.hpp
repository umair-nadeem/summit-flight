#pragma once

#include <array>
#include <variant>

#include "params.hpp"

namespace crsf
{

struct CrsfRcChannels
{
   std::array<uint16_t, rc_channel_count> channels{};
};

struct CrsfLinkStatistics
{
   uint8_t uplink_rssi_1;           // Uplink RSSI Antenna 1 (dBm * -1)
   uint8_t uplink_rssi_2;           // Uplink RSSI Antenna 2 (dBm * -1)
   uint8_t uplink_link_quality;     // Uplink Package success rate / Link quality (%)
   int8_t  uplink_snr;              // Uplink SNR (dB)
   uint8_t active_antenna;          // number of currently best antenna
   uint8_t rf_profile;              // enum {4fps = 0 , 50fps, 150fps}
   uint8_t uplink_rf_power;         // enum {0mW = 0, 10mW, 25mW, 100mW,
                                    // 500mW, 1000mW, 2000mW, 250mW, 50mW}
   uint8_t downlink_rssi;           // Downlink RSSI (dBm * -1)
   uint8_t downlink_link_quality;   // Downlink Package success rate / Link quality (%)
   int8_t  downlink_snr;            // Downlink SNR (dB)
};

struct CrsfLinkStatisticsTx
{
   uint8_t uplink_rssi_db;         // RSSI (dBm * -1)
   uint8_t uplink_rssi_pct;        // RSSI in percent
   uint8_t uplink_link_quality;    // Package success rate / Link quality (%)
   int8_t  uplink_snr;             // SNR (dB)
   uint8_t downlink_rf_power_db;   // rf power in dBm
   uint8_t uplink_fps;             // rf frames per second (fps / 10)
};

struct CrsfPacket
{
   FrameType type;

   std::variant<CrsfRcChannels, CrsfLinkStatistics, CrsfLinkStatisticsTx> data;
};

}   // namespace crsf
