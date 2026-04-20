#pragma once

#include <cstdint>

namespace control::attitude
{

struct StickCommand
{
   // normalized
   float roll{};       // -1.0 (left) to +1.0 (right)
   float pitch{};      // -1.0 (nose-down) to +1.0 (nose-up)
   float yaw{};        // -1.0 (CCW) to +1.0 (CW)
   float throttle{};   // 0.0 to 1.0
};

}   // namespace control::attitude
