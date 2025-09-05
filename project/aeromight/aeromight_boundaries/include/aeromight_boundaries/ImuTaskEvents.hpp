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

inline constexpr uint8_t pos_to_value(ImuTaskEvents pos)
{
   return 1u << static_cast<uint8_t>(pos);
}

}   // namespace aeromight_boundaries
