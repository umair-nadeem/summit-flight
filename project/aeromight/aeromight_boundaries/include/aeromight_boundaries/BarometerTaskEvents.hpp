#pragma once

#include "types/types.hpp"

namespace aeromight_boundaries
{

enum class BarometerTaskEvents : types::EventBitsType
{
   tick        = 0,    // bit position 0
   rx_complete = 1u,   // bit position 1
};

}   // namespace aeromight_boundaries
