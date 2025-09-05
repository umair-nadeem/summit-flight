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

inline constexpr uint8_t pos_to_value(BarometerTaskEvents pos)
{
   return 1u << static_cast<uint8_t>(pos);
}

}   // namespace aeromight_boundaries
