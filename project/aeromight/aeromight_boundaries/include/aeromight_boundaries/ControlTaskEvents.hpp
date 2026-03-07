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

}   // namespace aeromight_boundaries
