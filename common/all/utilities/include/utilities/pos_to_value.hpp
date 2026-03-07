#pragma once

#include <cstdint>

namespace utilities
{

template <typename T>
inline constexpr uint8_t pos_to_value(T pos)
{
   return 1u << static_cast<uint8_t>(pos);
}

}   // namespace utilities
