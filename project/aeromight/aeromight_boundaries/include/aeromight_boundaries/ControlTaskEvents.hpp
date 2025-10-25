#pragma once

#include <bitset>
#include <cstdint>

namespace aeromight_boundaries
{

enum class ControlTaskEvents : uint8_t
{
   start = 0,   // bit position 0
};

using ControlTaskNotificationFlags = std::bitset<4u>;

inline constexpr uint8_t pos_to_value(ControlTaskEvents pos)
{
   return 1u << static_cast<uint8_t>(pos);
}

}   // namespace aeromight_boundaries
