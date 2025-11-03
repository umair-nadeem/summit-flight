#pragma once

#include <cstdint>

namespace aeromight_boundaries
{

enum class FlightMode : uint8_t
{
   none = 0,
   stabilized_manual,
   altitude_hold,
   auto_land
};

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

struct FlightManagerSetpoints
{
   FlightArmedState state{FlightArmedState::disarm};
   FlightMode       mode{FlightMode::none};
   FlightStickInput input{};
   bool             kill_switch_active{false};
};

struct FlightManagerActuals
{
   float link_rssi{};
   bool  link_status_ok{false};
};

}   // namespace aeromight_boundaries
