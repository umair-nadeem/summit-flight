#pragma once

#include <cstdint>

namespace aeromight_boundaries
{

enum class ControlMode : uint8_t
{
   disabled,
   idle,
   manual_rate,
   altitude_hold,
   auto_land
};

struct ControlSetpoints
{
   ControlMode mode;

   // normalized
   float throttle{};   // 0.0 to 1.0
   float roll{};       // -1.0 (left) to +1.0 (right)
   float pitch{};      // -1.0 (nose-down) to +1.0 (nose-up)
   float yaw{};        // -1.0 (CCW) to +1.0 (CW)

   // safety
   bool armed;
   bool kill;
};

}   // namespace aeromight_boundaries
