#pragma once

#include <bitset>
#include <cstdint>

namespace aeromight_boundaries
{

enum class BarometerTaskEvents : uint8_t
{
   rx_complete = 0,   // bit position 0
};

using BarometerNotificationFlags = std::bitset<4u>;

}   // namespace aeromight_boundaries
