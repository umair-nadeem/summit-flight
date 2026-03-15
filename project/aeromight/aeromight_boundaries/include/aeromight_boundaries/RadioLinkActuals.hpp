#pragma once

#include <cstdint>

namespace aeromight_boundaries
{

struct RadioLinkActuals
{
   int8_t   link_rssi_dbm{};
   uint8_t  link_quality_pct{};
   int8_t   link_snr_db{};
   uint16_t tx_power_mw{};
   bool     link_status_ok{false};
};

}   // namespace aeromight_boundaries
