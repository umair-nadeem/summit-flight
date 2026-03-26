#pragma once

#include "types/types.hpp"

namespace aeromight_boundaries
{

enum class ImuTaskEvents : types::EventBitsType
{
   tick        = 0,    // bit position 0
   rx_complete = 1u,   // bit position 1
   calibrate   = 2u,   // bit position 2
};

}   // namespace aeromight_boundaries
