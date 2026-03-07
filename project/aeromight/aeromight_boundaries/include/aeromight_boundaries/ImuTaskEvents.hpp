#pragma once

#include <bitset>
#include <cstdint>

namespace aeromight_boundaries
{

enum class ImuTaskEvents : uint8_t
{
   tick        = 0,    // bit position 0
   rx_complete = 1u,   // bit position 1
};

using ImuNotificationFlags = std::bitset<4u>;

}   // namespace aeromight_boundaries
