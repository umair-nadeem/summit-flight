#pragma once

#include <cstdint>

namespace aeromight_boundaries
{

enum class FlightArmedState : uint8_t
{
   disarm = 0,
   arm
};

struct FlightStickInput
{
   float throttle{};   // 0.0 to 1.0
   float roll{};       // -1.0 (left) to +1.0 (right)
   float pitch{};      // -1.0 (nose-down) to +1.0 (nose-up)
   float yaw{};        // -1.0 (CCW) to +1.0 (CW)
};

struct RadioControlSetpoints
{
   FlightArmedState state{FlightArmedState::disarm};
   FlightStickInput input{};
};

struct RadioLinkStats
{
   int8_t   link_rssi_dbm{};
   uint8_t  link_quality_pct{};
   int8_t   link_snr_db{};
   uint16_t tx_power_mw{};
   bool     link_status_ok{false};
};

}   // namespace aeromight_boundaries
