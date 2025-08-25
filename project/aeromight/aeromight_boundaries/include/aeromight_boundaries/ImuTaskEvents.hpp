#pragma once

#include <bitset>
#include <cstdint>

namespace aeromight_boundaries
{

enum class ImuTaskEvents : uint32_t
{
   tick        = 0,    // bit position 0
   rx_complete = 1u,   // bit position 1
};

using NotificationFlags = std::bitset<sizeof(uint32_t)>;

inline constexpr uint32_t pos_to_value(ImuTaskEvents pos)
{
   return 1u << static_cast<uint32_t>(pos);
}

}   // namespace aeromight_boundaries
