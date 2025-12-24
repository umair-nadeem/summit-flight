#pragma once

#include <bitset>

#include "ControlState.hpp"

namespace aeromight_boundaries
{

struct ControlHealth
{
   enum class Error : uint8_t
   {
      invalid_time_delta = 0,
      max,
   };

   using ErrorBits = std::bitset<static_cast<uint8_t>(Error::max)>;

   ErrorBits    error{0};
   ControlState state{};
};

}   // namespace aeromight_boundaries
