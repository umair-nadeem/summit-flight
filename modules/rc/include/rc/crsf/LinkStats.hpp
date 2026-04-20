#pragma once

#include <cstdint>

namespace rc::crsf
{

struct LinkStats
{
   int8_t  uplink_rssi_1_dbm{};
   int8_t  uplink_rssi_2_dbm{};
   uint8_t uplink_quality_pct{};
   int8_t  uplink_snr_db{};

   int8_t  downlink_rssi_dbm{};
   uint8_t downlink_quality_pct{};
   int8_t  downlink_snr_db{};

   uint8_t  active_antenna{};
   uint8_t  rf_profile{};
   uint16_t tx_power_mw{};
};

}   // namespace rc::crsf
