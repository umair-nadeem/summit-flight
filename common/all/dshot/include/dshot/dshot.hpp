#pragma once

#include <algorithm>

#include "dshot/params.hpp"
#include "math/constants.hpp"

namespace dshot
{

inline uint16_t thrust_to_dshot_throttle(const float thrust) noexcept
{
   if (thrust <= math::constants::epsilon)
   {
      return 0u;
   }

   const float scaled = thrust * float(dshot_max - dshot_min);
   return dshot_min + static_cast<uint16_t>(scaled);
}

inline uint16_t get_dshot_frame(const uint16_t dshot_throttle, const bool telemetry) noexcept
{
   const uint16_t throttle = std::min<uint16_t>(dshot_throttle, dshot_max);

   const uint16_t value = static_cast<uint16_t>(throttle << 1u) | static_cast<uint16_t>(telemetry ? 1u : 0u);

   const uint16_t checksum = ((value ^ (value >> 4u) ^ (value >> 8u)) & 0x0fu);

   return static_cast<uint16_t>((value << 4) | checksum);
}

}   // namespace dshot
